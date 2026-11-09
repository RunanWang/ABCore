#include "ConstraintGeneration/GrowthWitnessGenerator.h"

#include "ConstraintGeneration/ComponentPeeling.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace constraint_generation {

GrowthWitnessGenerator::GrowthWitnessGenerator(
    const ComponentProblem &problem)
    : problem_(problem) {
    const ComponentPeeling validator(problem_); // 仅借构造检查 problem。
    (void)validator;
}

std::vector<WitnessCandidate> GrowthWitnessGenerator::generate(
    const Residual &residual,
    const std::size_t generation_limit) const {
    if (generation_limit == 0) {
        throw std::invalid_argument(
            "growth generation limit must be positive");
    }
    ComponentPeeling peeling(problem_); // residual 与 witness 统一校验器。
    peeling.validateResidual(residual);
    if (residual.feasible()) {
        return {};
    }

    // 生成与筛选彻底分离：candidate 从创建起就持有 witness 和 root provenance。
    std::vector<WitnessCandidate> candidates; // 统一候选值对象及其完整生命周期。
    std::map<Witness, std::size_t> candidate_index; // witness 到候选下标的去重索引。
    const std::vector<NODE_TYPE> roots =
        selectRoots(residual.active); // 确定性 boundary 生长顺序。
    if (roots.empty()) {
        throw std::logic_error(
            "growth found no active boundary in an infeasible residual");
    }
    for (const Policy policy : {Policy::Compact, Policy::SingleBoundary}) {
        for (const NODE_TYPE root : roots) {
            if (candidates.size() >= generation_limit) {
                break;
            }
            Witness witness =
                growWitness(root, policy, residual.active); // 当前生长结果。
            peeling.validateWitness(witness);

            // map 只索引候选位置；witness 本体始终由 WitnessCandidate 持有。
            const auto existing = candidate_index.find(witness); // 已有候选位置。
            if (existing == candidate_index.end()) {
                const std::size_t index = candidates.size(); // 新候选稳定下标。
                candidate_index.emplace(witness, index);
                candidates.push_back(
                    WitnessCandidate{std::move(witness), {root}});
                continue;
            }
            std::vector<NODE_TYPE> &origin_roots =
                candidates[existing->second].roots; // 同 support 的 provenance。
            if (std::find(origin_roots.begin(), origin_roots.end(), root) ==
                origin_roots.end()) {
                origin_roots.push_back(root);
            }
        }
        if (candidates.size() >= generation_limit) {
            break;
        }
    }

    if (candidates.empty()) {
        throw std::logic_error("growth missed an infeasible residual");
    }
    // 比较回调输入两个统一候选，输出 left 是否应更早进入 pipeline。
    std::sort(candidates.begin(), candidates.end(),
              [](const WitnessCandidate &left,
                 const WitnessCandidate &right) {
                  return left.witness < right.witness;
              });
    return candidates;
}

NODE_TYPE GrowthWitnessGenerator::activeDegree(
    const NODE_TYPE vertex,
    const std::vector<unsigned char> &active) const {
    NODE_TYPE degree = 0; // vertex 的 active 邻居数。
    for (std::size_t position = problem_.offsets[vertex];
         position < problem_.offsets[vertex + 1]; ++position) {
        degree += active[problem_.neighbors[position]];
    }
    return degree;
}

std::vector<NODE_TYPE> GrowthWitnessGenerator::selectRoots(
    const std::vector<unsigned char> &active) const {
    std::vector<NODE_TYPE> roots; // 所有 surviving boundary local IDs。
    for (NODE_TYPE vertex = 0; vertex < problem_.size(); ++vertex) {
        if (active[vertex] && problem_.boundary[vertex]) {
            roots.push_back(vertex);
        }
    }
    // 阈值小、active 度小的边界更紧，优先生成其 root-specific witness。
    // 比较回调输入两个 boundary IDs，输出 left 是否具有更高生长优先级。
    std::sort(roots.begin(), roots.end(),
              [this, &active](const NODE_TYPE left,
                              const NODE_TYPE right) {
                  return std::tuple(problem_.thresholds[left],
                                    activeDegree(left, active), left) <
                         std::tuple(problem_.thresholds[right],
                                    activeDegree(right, active), right);
              });
    return roots;
}

Witness GrowthWitnessGenerator::growWitness(
    const NODE_TYPE root,
    const Policy policy,
    const std::vector<unsigned char> &active) const {
    const NODE_TYPE size = problem_.size(); // 所有工作数组的统一维度。
    if (active.size() != size || root >= size || !active[root] ||
        !problem_.boundary[root]) {
        throw std::invalid_argument("growth root is not an active boundary");
    }

    // 1. 初始化增量 support、deficit 与插入顺序工作区。
    std::vector<unsigned char> in_witness(size, 0); // 是否已加入生长集。
    std::vector<unsigned char> deficient(size, 0);  // 集内 support 是否不足。
    std::vector<NODE_TYPE> support(size, 0);        // 当前集内诱导度。
    std::vector<NODE_TYPE> deficient_neighbors(
        size, 0); // 未入集点邻接的亏空点数量。
    std::deque<NODE_TYPE> pending; // 等待补足 support 的集内点。
    Witness insertion_order;       // 用于逆序安全裁剪。

    // 加入一个 active 新点，并增量维护 support 与 deficit 计数。
    // 输入：尚未入集的 active local ID。输出：原地更新全部 growth 工作区。
    const auto add_vertex = [&](const NODE_TYPE vertex) { // 原地扩张生长集。
        if (!active[vertex] || in_witness[vertex]) {
            throw std::logic_error(
                "growth tried to add an invalid or duplicate vertex");
        }
        in_witness[vertex] = 1;
        insertion_order.push_back(vertex);

        // 新点给所有已入集邻居增加 support，并关闭刚达标的 deficit。
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem_.neighbors[position]; // 获得新增 support 的邻居。
            const NODE_TYPE previous_support =
                support[neighbor]++; // 更新前的邻居集内度。
            if (in_witness[neighbor] && deficient[neighbor] &&
                previous_support < problem_.thresholds[neighbor] &&
                support[neighbor] >= problem_.thresholds[neighbor]) {
                deficient[neighbor] = 0;
                for (std::size_t adjacent = problem_.offsets[neighbor];
                     adjacent < problem_.offsets[neighbor + 1]; ++adjacent) {
                    const NODE_TYPE candidate =
                        problem_.neighbors[adjacent]; // deficit 计数待减的外部点。
                    if (active[candidate] && !in_witness[candidate]) {
                        if (deficient_neighbors[candidate] == 0) {
                            throw std::logic_error(
                                "growth deficit counter underflow");
                        }
                        --deficient_neighbors[candidate];
                    }
                }
            }
        }

        // 新点自身若尚未达标，则登记它对所有外部邻居的需求。
        if (support[vertex] < problem_.thresholds[vertex]) {
            deficient[vertex] = 1;
            pending.push_back(vertex);
            for (std::size_t position = problem_.offsets[vertex];
                 position < problem_.offsets[vertex + 1]; ++position) {
                const NODE_TYPE candidate =
                    problem_.neighbors[position]; // 可为新 deficit 提供 support 的点。
                if (active[candidate] && !in_witness[candidate]) {
                    ++deficient_neighbors[candidate];
                }
            }
        }
    };

    // 2. 从 root 开始，循环选择最能补足当前 deficit 的 active 邻居。
    add_vertex(root);
    while (!pending.empty()) {
        const NODE_TYPE needing_support =
            pending.front(); // 当前待补足阈值的集内点。
        pending.pop_front();
        if (!deficient[needing_support]) {
            continue;
        }

        // score 依次偏好：少引入新 boundary、净欠债低、欠债低、覆盖亏空多、
        // 当前 support 高、residual slack 低、local ID 小。
        std::optional<NODE_TYPE> selected; // 当前最佳扩张点；空表示尚未找到。
        std::tuple<unsigned char, long long, NODE_TYPE, long long, NODE_TYPE,
                   NODE_TYPE, NODE_TYPE>
            best_score; // 与 selected 对应的字典序评分。
        for (std::size_t position = problem_.offsets[needing_support];
             position < problem_.offsets[needing_support + 1]; ++position) {
            const NODE_TYPE candidate =
                problem_.neighbors[position]; // needing_support 的可选邻居。
            if (!active[candidate] || in_witness[candidate]) {
                continue;
            }
            const NODE_TYPE debt = // 候选入集后自身仍欠的 support。
                problem_.thresholds[candidate] > support[candidate]
                    ? problem_.thresholds[candidate] - support[candidate]
                    : 0;
            const NODE_TYPE gain =
                deficient_neighbors[candidate]; // 立即覆盖的 deficit 数。
            const unsigned char boundary_rank = // SingleBoundary 的边界惩罚。
                policy == Policy::SingleBoundary &&
                        problem_.boundary[candidate]
                    ? 1
                    : 0;
            const long long net_debt =
                static_cast<long long>(debt) -
                static_cast<long long>(gain); // 自身欠债减帮助收益。
            const NODE_TYPE residual_slack = // 候选在完整 residual 的余量。
                activeDegree(candidate, active) -
                problem_.thresholds[candidate];
            const auto score = std::tuple( // 越小越优的完整确定性评分。
                boundary_rank, net_debt, debt, -static_cast<long long>(gain),
                static_cast<NODE_TYPE>(
                    std::numeric_limits<NODE_TYPE>::max() -
                    support[candidate]),
                residual_slack, candidate);
            if (!selected || score < best_score) {
                selected = candidate;
                best_score = score;
            }
        }
        if (!selected) {
            throw std::logic_error(
                "growth got stuck inside a stable residual component");
        }
        add_vertex(*selected);
        if (deficient[needing_support]) {
            pending.push_back(needing_support);
        }
    }

    // 3. 逆插入序删除冗余点；其所有集内邻居删除后仍达标才可移除。
    for (auto iterator = insertion_order.rbegin();
         iterator != insertion_order.rend(); ++iterator) {
        const NODE_TYPE vertex = *iterator; // 当前逆序试删点。
        if (vertex == root || !in_witness[vertex]) {
            continue;
        }
        bool safe = true; // 删除 vertex 是否保持其余点全部稳定。
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem_.neighbors[position]; // 删除时失去一条边的集内邻居。
            if (in_witness[neighbor] &&
                support[neighbor] <= problem_.thresholds[neighbor]) {
                safe = false;
                break;
            }
        }
        if (!safe) {
            continue;
        }
        in_witness[vertex] = 0;
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem_.neighbors[position]; // 实际更新 support 的集内邻居。
            if (in_witness[neighbor]) {
                --support[neighbor];
            }
        }
    }

    // 4. 安全裁剪可能断开集合；最终只保留仍含 root 的连通分量。
    std::vector<unsigned char> reached(size, 0); // root 分量 BFS 标记。
    std::deque<NODE_TYPE> queue; // root 分量 BFS 队列。
    reached[root] = 1;
    queue.push_back(root);
    Witness witness; // 裁剪后最终保留的 root 连通分量。
    while (!queue.empty()) {
        const NODE_TYPE vertex = queue.front(); // 当前扩展的 witness 点。
        queue.pop_front();
        witness.push_back(vertex);
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem_.neighbors[position]; // 待访问的保留邻居。
            if (in_witness[neighbor] && !reached[neighbor]) {
                reached[neighbor] = 1;
                queue.push_back(neighbor);
            }
        }
    }
    std::sort(witness.begin(), witness.end());
    return witness;
}

}  // namespace constraint_generation
