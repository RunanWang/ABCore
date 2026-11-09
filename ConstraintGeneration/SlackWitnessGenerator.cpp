#include "ConstraintGeneration/SlackWitnessGenerator.h"

#include "ConstraintGeneration/ComponentPeeling.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <numeric>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace constraint_generation {
namespace {

// 计算 generalized core depth：depth(v) 是 v 可承受的最大额外统一阈值。
// 输入：合法 fixed point。输出：每个 local vertex 的非负 slack depth。
std::vector<NODE_TYPE> computeDepths(const ComponentProblem &problem,
                                     const Residual &residual) {
    using HeapEntry =
        std::pair<std::int64_t, NODE_TYPE>; // 动态 key 与 local ID。
    std::priority_queue<HeapEntry, std::vector<HeapEntry>,
                        std::greater<HeapEntry>>
        heap; // 当前未删除点的最小 key 堆，允许 stale entries。
    std::vector<std::int64_t> keys(problem.size(), 0); // 动态 degree-delta。
    std::vector<unsigned char> removed(problem.size(), 0); // 分解删除标记。
    std::vector<NODE_TYPE> depths(problem.size(), 0); // 最终 maximal slack。

    // 1. 仅把 active 顶点加入最小堆；inactive 点视为已删除。
    for (NODE_TYPE vertex = 0; vertex < problem.size(); ++vertex) {
        if (!residual.active[vertex]) {
            removed[vertex] = 1;
            continue;
        }
        keys[vertex] =
            static_cast<std::int64_t>(residual.degrees[vertex]) -
            static_cast<std::int64_t>(problem.thresholds[vertex]);
        heap.emplace(keys[vertex], vertex);
    }

    // 2. 按最小 key 剥离；stale heap entry 代替隐式 decrease-key。
    std::int64_t current_depth = 0; // 已弹出顶点的非递减 core 层。
    while (!heap.empty()) {
        const auto [key, vertex] = heap.top(); // 当前最小候选 entry。
        heap.pop();
        if (removed[vertex] || key != keys[vertex]) {
            continue;
        }
        removed[vertex] = 1;
        current_depth = std::max(current_depth, key);
        depths[vertex] = static_cast<NODE_TYPE>(current_depth);
        for (std::size_t position = problem.offsets[vertex];
             position < problem.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem.neighbors[position]; // 删除后 key 减一的邻居。
            if (!removed[neighbor]) {
                --keys[neighbor];
                heap.emplace(keys[neighbor], neighbor);
            }
        }
    }
    return depths;
}

class DescendingComponents {
public:
    // 初始化尚未激活的并查集；成员表用于直接导出 canonical component。
    // 输入：local vertex 数。输出：空的 descending-level 组件维护器。
    explicit DescendingComponents(const NODE_TYPE size)
        : parent_(size), sizes_(size, 1), active_(size, 0), members_(size) {
        std::iota(parent_.begin(), parent_.end(), NODE_TYPE{0});
    }

    // 激活一个顶点并建立单点分量。
    // 输入：尚未激活的 local ID。输出：内部状态原地更新。
    void activate(const NODE_TYPE vertex) {
        active_[vertex] = 1;
        parent_[vertex] = vertex;
        sizes_[vertex] = 1;
        members_[vertex] = {vertex};
    }

    // 判断顶点是否已进入当前及更高 slack 层。
    // 输入：local ID。输出：已激活时为 true。
    bool active(const NODE_TYPE vertex) const noexcept {
        return active_[vertex] != 0;
    }

    // 查找并路径压缩一个 active 顶点的根。
    // 输入：active local ID。输出：并查集根。
    NODE_TYPE find(NODE_TYPE vertex) {
        while (parent_[vertex] != vertex) {
            parent_[vertex] = parent_[parent_[vertex]];
            vertex = parent_[vertex];
        }
        return vertex;
    }

    // 读取给定 active 顶点所在分量的内部成员，不复制也不保证顺序。
    // 输入：active local ID。输出：下一次 unite 前有效的只读成员引用。
    const Witness &members(const NODE_TYPE vertex) {
        return members_[find(vertex)];
    }

    // 合并两个 active 顶点所在分量。
    // 输入：两个 local IDs。输出：按规模合并后的内部状态。
    void unite(const NODE_TYPE left, const NODE_TYPE right) {
        NODE_TYPE left_root = find(left); // 合并前左分量根。
        NODE_TYPE right_root = find(right); // 合并前右分量根。
        if (left_root == right_root) {
            return;
        }
        if (sizes_[left_root] < sizes_[right_root]) {
            std::swap(left_root, right_root);
        }
        parent_[right_root] = left_root;
        sizes_[left_root] += sizes_[right_root];
        members_[left_root].insert(members_[left_root].end(),
                                   members_[right_root].begin(),
                                   members_[right_root].end());
        members_[right_root].clear();
    }

    // 导出给定 active 顶点所在的有序分量。
    // 输入：active local ID。输出：升序、无重复的 local IDs。
    Witness component(const NODE_TYPE vertex) {
        Witness result = members_[find(vertex)]; // 当前层显式分量副本。
        std::sort(result.begin(), result.end());
        return result;
    }

private:
    std::vector<NODE_TYPE> parent_;              // 并查集父指针。
    std::vector<std::size_t> sizes_;             // 根节点对应的分量规模。
    std::vector<unsigned char> active_;          // 是否已加入 descending 层。
    std::vector<std::vector<NODE_TYPE>> members_; // 根节点持有的显式成员。
};

}  // namespace

SlackWitnessGenerator::SlackWitnessGenerator(const ComponentProblem &problem)
    : problem_(problem) {
    const ComponentPeeling validator(problem_); // 仅借构造检查 problem。
    (void)validator;
}

SlackGenerationResult SlackWitnessGenerator::generate(
    const Residual &residual) const {
    ComponentPeeling peeling(problem_); // 输入 residual 的结构与 fixed-point 校验器。
    peeling.validateResidual(residual);
    if (residual.feasible()) {
        return {};
    }

    // 1. 计算 active 顶点的 maximal slack，并建立 descending 顺序。
    const std::vector<NODE_TYPE> depths =
        computeDepths(problem_, residual); // 每点 maximal slack。
    std::vector<NODE_TYPE> order; // active 点的 descending-depth 顺序。
    order.reserve(residual.active_vertices);
    for (NODE_TYPE vertex = 0; vertex < problem_.size(); ++vertex) {
        if (residual.active[vertex]) {
            order.push_back(vertex);
        }
    }
    // 比较回调输入两个 local ID，输出 left 是否应更早进入 descending 顺序。
    std::sort(order.begin(), order.end(),
              [&depths](const NODE_TYPE left, const NODE_TYPE right) {
                  if (depths[left] != depths[right]) {
                      return depths[left] > depths[right];
                  }
                  return left < right;
              });

    // 2. 逐层激活 depth >= s 的点；零层只会退化为整块 residual。
    DescendingComponents components(problem_.size()); // 当前 depth 层并查集。
    SlackGenerationResult result; // 逐 boundary 的正 depth cuts 与最大值。
    for (std::size_t begin = 0; begin < order.size();) {
        std::size_t end = begin + 1; // 当前等深度层的右开边界。
        const NODE_TYPE depth = depths[order[begin]]; // 当前 slack 层。
        if (depth == 0) {
            // descending 顺序保证后续也全是零层，无需再构造重复 support。
            break;
        }
        while (end < order.size() && depths[order[end]] == depth) {
            ++end;
        }

        // 激活当前层，再与所有已激活邻居合并。
        for (std::size_t position = begin; position < end; ++position) {
            components.activate(order[position]);
        }
        for (std::size_t position = begin; position < end; ++position) {
            const NODE_TYPE vertex = order[position]; // 当前层待合并的点。
            for (std::size_t edge_position = problem_.offsets[vertex];
                 edge_position < problem_.offsets[vertex + 1];
                 ++edge_position) {
                const NODE_TYPE neighbor =
                    problem_.neighbors[edge_position]; // 可能已激活的邻居。
                if (components.active(neighbor)) {
                    components.unite(vertex, neighbor);
                }
            }
        }

        // 先按 component 去重，再从每个 component 中找本层 boundary。
        std::set<NODE_TYPE> component_roots; // 含本层新激活点的唯一并查集根。
        for (std::size_t position = begin; position < end; ++position) {
            component_roots.insert(
                components.find(order[position])); // 合并后的稳定分量标识。
        }
        for (const NODE_TYPE component_root : component_roots) {
            std::vector<NODE_TYPE> boundaries; // 当前分量中 maximal depth 恰为本层的边界。
            for (const NODE_TYPE vertex : components.members(component_root)) {
                if (problem_.boundary[vertex] && depths[vertex] == depth) {
                    boundaries.push_back(vertex);
                }
            }
            if (boundaries.empty()) {
                continue;
            }
            std::sort(boundaries.begin(), boundaries.end());
            Witness component =
                components.component(component_root); // 有本层 boundary 时只导出一次。
            for (std::size_t index = 0; index < boundaries.size(); ++index) {
                // 同 support 的 boundary 保留不同加权系数；最后一条复用原 component。
                Witness support = index + 1 == boundaries.size()
                                      ? std::move(component)
                                      : component;
                result.cuts.push_back(
                    SlackCut{boundaries[index], depth, std::move(support)});
            }
            result.max_slack = std::max(result.max_slack, depth);
        }
        begin = end;
    }

    // 3. 只调整输出顺序，不再跨 boundary 合并或丢弃当前层 slack。
    // 比较回调输入两条完整 cuts，输出 left 是否应更早进入确定性结果。
    std::sort(result.cuts.begin(), result.cuts.end(),
              [](const SlackCut &left, const SlackCut &right) {
                  if (left.boundary != right.boundary) {
                      return left.boundary < right.boundary;
                  }
                  if (left.slack != right.slack) {
                      return left.slack < right.slack;
                  }
                  return left.component < right.component;
              });
    return result;
}

}  // namespace constraint_generation
