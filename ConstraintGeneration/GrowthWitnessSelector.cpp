#include "ConstraintGeneration/GrowthWitnessSelector.h"

#include <algorithm>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace constraint_generation {
namespace {

// 检查 witness 与 origin roots 的局部作用域，尽早隔离 generator 契约错误。
// 输入：候选和局部问题。输出：合法时返回；非法时抛出 invalid_argument。
void validateCandidates(const std::vector<WitnessCandidate> &candidates,
                        const ComponentProblem &problem) {
    if (problem.boundary.size() != problem.vertices.size()) {
        throw std::invalid_argument(
            "growth selector received an invalid local problem");
    }
    for (const WitnessCandidate &candidate : candidates) {
        if (candidate.witness.empty() ||
            !std::is_sorted(candidate.witness.begin(),
                            candidate.witness.end()) ||
            std::adjacent_find(candidate.witness.begin(),
                               candidate.witness.end()) !=
                candidate.witness.end()) {
            throw std::invalid_argument(
                "growth selector received a non-canonical witness");
        }
        for (const NODE_TYPE vertex : candidate.witness) {
            if (vertex >= problem.size()) {
                throw std::invalid_argument(
                    "growth selector received an invalid witness vertex");
            }
        }
        if (candidate.roots.empty()) {
            throw std::invalid_argument(
                "growth selector received a candidate without roots");
        }
        for (const NODE_TYPE root : candidate.roots) {
            if (root >= problem.size() || !problem.boundary[root]) {
                throw std::invalid_argument(
                    "growth selector received a non-boundary root");
            }
            if (!std::binary_search(candidate.witness.begin(),
                                    candidate.witness.end(), root)) {
                throw std::invalid_argument(
                    "growth selector root is outside its witness scope");
            }
        }
    }
}

}  // namespace

std::vector<WitnessCandidate> GrowthWitnessSelector::select(
    const std::vector<WitnessCandidate> &candidates,
    const ComponentProblem &problem,
    const std::size_t batch_limit) {
    validateCandidates(candidates, problem);
    if (candidates.empty()) {
        return {};
    }
    if (batch_limit == 0) {
        throw std::invalid_argument(
            "growth selector batch limit must be positive");
    }

    const std::size_t selection_limit = // 本轮最终 admission 上限。
        std::min(candidates.size(), batch_limit);
    std::vector<std::size_t> order(candidates.size()); // 候选访问顺序。
    std::iota(order.begin(), order.end(), 0);
    // 比较回调输入两个候选下标，输出 left 是否为更小的 witness。
    std::sort(order.begin(), order.end(),
              [&candidates](const std::size_t left,
                            const std::size_t right) {
                  return std::tuple(candidates[left].witness.size(),
                                    candidates[left].witness) <
                         std::tuple(candidates[right].witness.size(),
                                    candidates[right].witness);
              });

    std::vector<unsigned char> selected(candidates.size(), 0); // 接纳标记。
    std::vector<unsigned char> represented_root(
        problem.size(), 0); // 已有代表 witness 的 roots。
    std::vector<std::size_t> incidence(
        problem.size(), 0); // 已选 witness 热度。
    std::vector<WitnessCandidate> batch; // 最终 admission 结果。
    batch.reserve(selection_limit);

    // 接纳候选并同步维护 vertex incidence 与 root coverage。
    // 输入：候选下标。输出：原地追加 batch 并更新三组 admission 状态。
    const auto admit = [&](const std::size_t index) { // 接纳并更新覆盖状态。
        selected[index] = 1;
        batch.push_back(candidates[index]);
        for (const NODE_TYPE vertex : candidates[index].witness) {
            ++incidence[vertex];
        }
        for (const NODE_TYPE root : candidates[index].roots) {
            represented_root[root] = 1;
        }
    };

    // 1. Packing pass：小 witness 优先，先收集两两不交的一批强下界代理。
    for (const std::size_t index : order) {
        if (batch.size() >= selection_limit) {
            break;
        }
        const bool disjoint = std::all_of( // 是否与已接纳集合完全不交。
            candidates[index].witness.begin(),
            candidates[index].witness.end(),
            // 判定回调输入一个 local ID，输出它是否尚未出现在已选 batch。
            [&incidence](const NODE_TYPE vertex) {
                return incidence[vertex] == 0;
            });
        if (disjoint) {
            admit(index);
        }
    }

    // 2. Refinement pass：优先未覆盖 root，再惩罚大集合和高 incidence 热点。
    while (batch.size() < selection_limit) {
        std::optional<std::size_t> best; // 本轮最优未选候选下标。
        std::tuple<unsigned char, std::size_t, std::size_t, std::size_t>
            best_score; // 与 best 对应的字典序 admission 评分。
        for (const std::size_t index : order) {
            if (selected[index]) {
                continue;
            }
            const WitnessCandidate &candidate =
                candidates[index]; // 当前待评分候选。
            const bool has_unrepresented_root = std::any_of( // 是否补 root。
                candidate.roots.begin(), candidate.roots.end(),
                // 判定回调输入一个 origin root，输出它是否尚无代表约束。
                [&represented_root](const NODE_TYPE root) {
                    return !represented_root[root];
                });
            std::size_t incidence_mass = 0; // 与已选热点的总重叠次数。
            for (const NODE_TYPE vertex : candidate.witness) {
                incidence_mass += incidence[vertex];
            }
            const auto score = std::tuple( // 越小越优的 admission 评分。
                static_cast<unsigned char>(!has_unrepresented_root),
                candidate.witness.size() + incidence_mass,
                candidate.witness.size(), index);
            if (!best || score < best_score) {
                best = index;
                best_score = score;
            }
        }
        if (!best) {
            break;
        }
        admit(*best);
    }

    // 比较回调按 witness local-ID 集合恢复确定性输出顺序。
    std::sort(batch.begin(), batch.end(),
              [](const WitnessCandidate &left,
                 const WitnessCandidate &right) {
                  return left.witness < right.witness;
              });
    return batch;
}

}  // namespace constraint_generation
