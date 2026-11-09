#include "ExchangeSearch/Greedy.h"

#include <algorithm>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace anytime::internal {
namespace {

struct CandidateScore {
    NODE_TYPE vertex{0}; // 候选 local ID。
    std::size_t boundary_gain{0}; // 阻断后移除的 active boundary 数。
    double residual_coverage{0.0}; // 历史 residual-boundary 累计覆盖。
    double cascade_score{0.0}; // 邻居 fragility 的解析分数。
};

// 按独立 greedy 的确定性字典序比较两个候选。
// 输入：candidate 与 best 为同一 residual 的评分。输出：candidate 是否更优。
bool betterCandidate(const Component &component,
                     const CandidateScore &candidate,
                     const CandidateScore &best) {
    if (candidate.boundary_gain != best.boundary_gain) {
        return candidate.boundary_gain > best.boundary_gain;
    }
    if (candidate.residual_coverage != best.residual_coverage) {
        return candidate.residual_coverage > best.residual_coverage;
    }
    if (candidate.cascade_score != best.cascade_score) {
        return candidate.cascade_score > best.cascade_score;
    }
    if (component.boundary[candidate.vertex] !=
        component.boundary[best.vertex]) {
        return component.boundary[candidate.vertex] != 0;
    }
    return component.vertices[candidate.vertex] <
           component.vertices[best.vertex];
}

// 在当前 target-related residuals 的并集中选择下一 greedy blocker。
// 输入：workspace 保留当前 fixed point，coverage 为历史 residual 累计分数。
// 输出：按确定性字典序最优的 local candidate。
NODE_TYPE selectGreedyCandidate(
    const Component &component,
    PeelingWorkspace &workspace,
    const std::vector<GreedyResidual> &residual_components,
    const std::vector<double> &coverage,
    const std::vector<unsigned char> &selected) {
    std::vector<unsigned char> candidate_mask(
        component.size(), 0); // 当前 target-related residual union 位图。
    for (const GreedyResidual &residual : residual_components) {
        for (const NODE_TYPE vertex : residual) {
            candidate_mask[vertex] = 1;
        }
    }

    std::optional<CandidateScore> best; // 当前字典序最优候选。
    for (NODE_TYPE vertex = 0; vertex < component.size(); ++vertex) {
        if (!candidate_mask[vertex] || selected[vertex] ||
            !workspace.active(vertex)) {
            continue;
        }
        double cascade_score = 0.0; // 当前点的邻居 fragility 总和。
        for (std::size_t position = component.offsets[vertex];
             position < component.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor = component.neighbors[position];
            if (!workspace.active(neighbor)) {
                continue;
            }
            const NODE_TYPE degree = workspace.degree(neighbor);
            if (degree < component.thresholds[neighbor]) {
                throw std::logic_error(
                    "active anytime greedy vertex is below threshold");
            }
            cascade_score +=
                1.0 / static_cast<double>(
                          degree - component.thresholds[neighbor] + 1);
        }
        const CandidateScore candidate{
            vertex,
            workspace.boundaryGainIfBlocked(vertex),
            coverage[vertex],
            cascade_score};
        if (!best || betterCandidate(component, candidate, *best)) {
            best = candidate;
        }
    }
    if (!best) {
        throw std::logic_error(
            "infeasible anytime greedy state has no candidate");
    }
    return best->vertex;
}

// 检查用户中断回调；空回调表示继续。
// 输入：config 保存可选回调。输出：是否停止下一轮 greedy 选择。
bool stopRequested(const AnytimeConfig &config) {
    return config.should_stop && config.should_stop();
}

}  // namespace

std::vector<NODE_TYPE> reverseDelete(
    PeelingWorkspace &workspace,
    std::vector<NODE_TYPE> blockers,
    AnytimeComponentResult &statistics) {
    const bool initially_feasible =
        workspace.run(blockers, false).feasible(); // 输入解的完整可行性。
    ++statistics.tahp_calls;
    if (!initially_feasible) {
        throw std::logic_error(
            "reverse deletion received an infeasible blocker set");
    }
    const std::size_t bitmap_size = blockers.empty()
                                        ? 0
                                        : *std::max_element(blockers.begin(),
                                                            blockers.end()) +
                                              1; // 覆盖全部 blocker local IDs。
    std::vector<unsigned char> retained(
        bitmap_size, 0); // 当前仍保留的 blocker 位图。
    for (const NODE_TYPE blocker : blockers) {
        retained[blocker] = 1;
    }
    // 按原选择顺序的逆序逐点试删；成功删除的点不再加入后续 trial。
    // 某点若当前不能删除，后续 blockers 只会更少，按单调性无需再次检查。
    for (auto iterator = blockers.rbegin(); iterator != blockers.rend();
         ++iterator) {
        const NODE_TYPE candidate = *iterator;
        std::vector<NODE_TYPE> trial; // 撤销 candidate 后仍保留的 blockers。
        trial.reserve(blockers.size() - 1);
        for (const NODE_TYPE blocker : blockers) {
            if (blocker != candidate && retained[blocker]) {
                trial.push_back(blocker);
            }
        }
        const auto checked = workspace.run(trial, false); // 完整可行性重放。
        ++statistics.tahp_calls;
        if (checked.feasible()) {
            retained[candidate] = 0;
        }
    }
    std::vector<NODE_TYPE> result; // 删除冗余后的升序 local blockers。
    for (const NODE_TYPE blocker : blockers) {
        if (retained[blocker]) {
            result.push_back(blocker);
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::optional<std::vector<NODE_TYPE>> buildGreedyIncumbent(
    const Component &component,
    PeelingWorkspace &workspace,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config) {
    std::vector<NODE_TYPE> blockers; // 已选 local blockers。
    std::vector<unsigned char> selected(component.size(), 0); // 已选位图。
    // 该集合只服务 greedy 历史覆盖评分，与 Exact-S repair 无关。
    std::set<GreedyResidual> seen_residuals;
    std::vector<double> coverage(component.size(), 0.0); // 每点累计 coverage。

    // 1. Greedy 只从 baseline 初始化一次；后续选点永久提交到该 fixed point。
    if (stopRequested(config)) {
        return std::nullopt;
    }
    auto residual = workspace.run({}, true); // 空额外 blocker 的初始 residual。
    ++statistics.tahp_calls;
    while (!residual.feasible()) {
        if (stopRequested(config)) {
            return std::nullopt;
        }
        // 2. 只让首次出现的 residual component 增加历史覆盖分数。
        for (const GreedyResidual &residual_component :
             residual.residual_components) {
            if (!seen_residuals.insert(residual_component).second) {
                continue;
            }
            const std::size_t boundary_count =
                static_cast<std::size_t>(std::count_if(
                    residual_component.begin(), residual_component.end(),
                    [&component](const NODE_TYPE vertex) {
                        return component.boundary[vertex] != 0;
                    })); // 当前 residual component 的 boundary 数。
            const double contribution =
                static_cast<double>(boundary_count) /
                static_cast<double>(residual_component.size()); // 统一覆盖增量。
            for (const NODE_TYPE vertex : residual_component) {
                coverage[vertex] += contribution;
            }
        }
        // 3. 在当前 fixed point 上模拟候选 cascade，并永久加入最佳 blocker。
        const NODE_TYPE chosen = selectGreedyCandidate(
            component, workspace, residual.residual_components, coverage,
            selected);
        selected[chosen] = 1;
        blockers.push_back(chosen);
        residual = workspace.commitBlocker(chosen, true);
        ++statistics.greedy_incremental_updates;
    }
    return reverseDelete(workspace, std::move(blockers), statistics);
}

}  // namespace anytime::internal
