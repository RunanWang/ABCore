#ifndef ABCORE_CONSTRAINT_GENERATION_SLACK_WITNESS_SELECTOR_H
#define ABCORE_CONSTRAINT_GENERATION_SLACK_WITNESS_SELECTOR_H

#include "ConstraintGeneration/Types.h"

#include <vector>

namespace constraint_generation {

class SlackWitnessSelector {
public:
    // 按高 depth 优先与 batch 上限筛选首轮 Slack cuts。
    // 输入：完整 cuts 和接纳上限。
    // 输出：按 depth 降序、support 规模升序确定性选出的子集。
    static std::vector<SlackCut> selectForInitial(
        std::vector<SlackCut> cuts,
        std::size_t batch_limit);

    // 为后续 separation 选择有限批高 depth Slack cuts。
    // 输入：当前 residual 的正 depth cuts 和 batch 上限。
    // 输出：按 depth 降序、support 规模升序和作用域确定性选出的 cuts。
    static std::vector<SlackCut> selectForSeparation(
        std::vector<SlackCut> cuts, std::size_t batch_limit);
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_SLACK_WITNESS_SELECTOR_H
