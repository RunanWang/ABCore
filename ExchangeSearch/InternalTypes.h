#ifndef ABCORE_EXCHANGE_SEARCH_INTERNAL_TYPES_H
#define ABCORE_EXCHANGE_SEARCH_INTERNAL_TYPES_H

#include "constant.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace anytime::internal {

// Greedy 阶段只用 residual connected components 累计候选覆盖分数。
// Repair 阶段不读取该类型，也不维护 witness pool。
using GreedyResidual = std::vector<NODE_TYPE>;

constexpr std::size_t kNoIndex =
    std::numeric_limits<std::size_t>::max(); // 不存在局部编号或变量列。

struct Component {
    NODE_TYPE index{0}; // 从一开始的稳定 component 编号。
    std::vector<NODE_TYPE> vertices; // local ID 到 global ID 的映射。
    std::vector<std::size_t> offsets; // 长度 |V|+1 的局部 CSR 邻接起点。
    std::vector<NODE_TYPE> neighbors; // 局部 CSR 的邻居 local IDs。
    std::vector<NODE_TYPE> thresholds; // 每个外部点的异质 peeling 阈值。
    std::vector<unsigned char> boundary; // 是否与目标子图相邻。
    // forced-only fixed point 内的局部度数，等于 CSR 中对应邻接段长度。
    std::vector<NODE_TYPE> baseline_degrees;

    NODE_TYPE size() const noexcept {
        return static_cast<NODE_TYPE>(vertices.size());
    }
};

struct PreprocessingResult {
    std::vector<NODE_TYPE> forced_blockers; // mandatory global blocker IDs。
    std::vector<Component> components; // forced-only target-related components。
};

struct ExchangeRunResult {
    std::vector<NODE_TYPE> blockers; // 最终 local incumbent。
    bool interrupted{false}; // 是否因回调或组合预算提前结束。
};

}  // namespace anytime::internal

#endif  // ABCORE_EXCHANGE_SEARCH_INTERNAL_TYPES_H
