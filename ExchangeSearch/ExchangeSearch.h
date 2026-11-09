#ifndef ABCORE_EXCHANGE_SEARCH_EXCHANGE_SEARCH_H
#define ABCORE_EXCHANGE_SEARCH_EXCHANGE_SEARCH_H

#include "ExchangeSearch/Solver.h"
#include "ExchangeSearch/InternalTypes.h"
#include "ExchangeSearch/Peeling.h"

#include <cstddef>
#include <functional>
#include <vector>

namespace anytime::internal {

// 对一个 component 按配置的组合粒度持续执行 Exact-S exchanges。
// 输入：initial 为 inclusion-minimal 可行解，global_combinations 为跨分量预算计数器；
// publish 在严格改进后同步更新全局状态。
// 输出：最终 local incumbent；statistics 原位累计完成半径和搜索工作量。
ExchangeRunResult runExchangeSearch(
    const Component &component,
    PeelingWorkspace &workspace,
    std::vector<NODE_TYPE> initial,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config,
    std::size_t &global_combinations,
    const std::function<void(const std::vector<NODE_TYPE> &, std::size_t)> &
        publish);

}  // namespace anytime::internal

#endif  // ABCORE_EXCHANGE_SEARCH_EXCHANGE_SEARCH_H
