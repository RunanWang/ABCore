#ifndef ABCORE_EXCHANGE_SEARCH_GREEDY_H
#define ABCORE_EXCHANGE_SEARCH_GREEDY_H

#include "ExchangeSearch/Solver.h"
#include "ExchangeSearch/InternalTypes.h"
#include "ExchangeSearch/Peeling.h"

#include <optional>
#include <vector>

namespace anytime::internal {

// 对可行 blocker 集执行一次反向冗余删除。
// 输入：blockers 为 local IDs 且已可行。输出：仍可行的 inclusion-minimal 子集。
std::vector<NODE_TYPE> reverseDelete(
    PeelingWorkspace &workspace,
    std::vector<NODE_TYPE> blockers,
    AnytimeComponentResult &statistics);

// 从空集独立构造一个 WG-style greedy 可行解。
// 输入：component、可复用 workspace 和中断配置。输出：中断时返回空 optional。
std::optional<std::vector<NODE_TYPE>> buildGreedyIncumbent(
    const Component &component,
    PeelingWorkspace &workspace,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config);

}  // namespace anytime::internal

#endif  // ABCORE_EXCHANGE_SEARCH_GREEDY_H

