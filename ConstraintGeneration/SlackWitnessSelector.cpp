#include "ConstraintGeneration/SlackWitnessSelector.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace constraint_generation {
namespace {

// 按高 depth、小 support 与完整 key 排序并截断候选。
// 输入：cuts 为已通过语义筛选的候选，batch_limit 为正上限。
// 输出：最多 batch_limit 条可复现的高优先级 cuts。
std::vector<SlackCut> selectCapped(std::vector<SlackCut> cuts,
                                   const std::size_t batch_limit) {
    if (batch_limit == 0) {
        throw std::invalid_argument(
            "Slack selection requires a positive batch limit");
    }
    // 比较回调先保留更强的 depth，再用小 support 和完整 key 破平局。
    std::sort(cuts.begin(), cuts.end(),
              [](const SlackCut &left, const SlackCut &right) {
                  if (left.slack != right.slack) {
                      return left.slack > right.slack;
                  }
                  if (left.component.size() != right.component.size()) {
                      return left.component.size() < right.component.size();
                  }
                  if (left.boundary != right.boundary) {
                      return left.boundary < right.boundary;
                  }
                  return left.component < right.component;
              });
    if (cuts.size() > batch_limit) {
        cuts.resize(batch_limit);
    }
    return cuts;
}

}  // namespace

std::vector<SlackCut> SlackWitnessSelector::selectForInitial(
    std::vector<SlackCut> cuts, const std::size_t batch_limit) {
    return selectCapped(std::move(cuts), batch_limit);
}

std::vector<SlackCut> SlackWitnessSelector::selectForSeparation(
    std::vector<SlackCut> cuts, const std::size_t batch_limit) {
    return selectCapped(std::move(cuts), batch_limit);
}

}  // namespace constraint_generation
