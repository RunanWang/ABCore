#ifndef ABCORE_EXCHANGE_SEARCH_REPAIR_ENUMERATOR_H
#define ABCORE_EXCHANGE_SEARCH_REPAIR_ENUMERATOR_H

#include "ExchangeSearch/Solver.h"
#include "ExchangeSearch/InternalTypes.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace anytime::internal {

struct RepairEnumerationResult {
    std::optional<std::vector<NODE_TYPE>> repair; // 找到的 local repair blockers。
    bool interrupted{false}; // 是否在穷尽候选前收到中断。
};

// 在固定 blockers 产生的 residual 上枚举至多 budget 个 repair blockers。
// 输入：incumbent 用于排除旧 blocker，fixed 为 B-S，budget 为 |S|-1。
// 输出：首个消除全部 active boundary 的 repair，或穷尽/中断状态。
RepairEnumerationResult enumerateRepair(
    const Component &component,
    const std::vector<NODE_TYPE> &incumbent,
    const std::vector<NODE_TYPE> &fixed,
    std::size_t budget,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config);

}  // namespace anytime::internal

#endif  // ABCORE_EXCHANGE_SEARCH_REPAIR_ENUMERATOR_H

