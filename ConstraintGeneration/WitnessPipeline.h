#ifndef ABCORE_CONSTRAINT_GENERATION_WITNESS_PIPELINE_H
#define ABCORE_CONSTRAINT_GENERATION_WITNESS_PIPELINE_H

#include "ConstraintGeneration/ComponentPeeling.h"
#include "ConstraintGeneration/ExactConfig.h"
#include "ConstraintGeneration/GrowthWitnessGenerator.h"
#include "ConstraintGeneration/SlackWitnessGenerator.h"
#include "ConstraintGeneration/Types.h"

#include <vector>

namespace constraint_generation {

class WitnessPipeline {
public:
    // 组合 Slack 初始化与 Slack/Growth 混合分离。
    // 输入：局部问题和已校验配置。输出：component 专属 witness pipeline。
    WitnessPipeline(const ComponentProblem &problem, ExactConfig config);

    // 检查 forced-only residual，并构造第一轮 restricted master 约束。
    // 输入：构造时的局部问题。输出：显式可行状态或非空初始 batch。
    PipelineResult initial() const;

    // 检查 master blockers，并生成 Growth 与 Slack 组合分离约束。
    // 输入：当前 local blockers。输出：显式可行状态或非空未命中 batch。
    PipelineResult separate(const std::vector<NODE_TYPE> &blockers) const;

private:
    const ComponentProblem &problem_;       // pipeline 绑定的局部问题。
    ExactConfig config_;                   // 策略与预算的值语义快照。
    ComponentPeeling peeling_;             // 所有阶段共享的 fixed-point 语义。
    SlackWitnessGenerator slack_generator_; // 首轮和后续 residual 共用。
    GrowthWitnessGenerator growth_generator_; // 后续 Growth separator。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_WITNESS_PIPELINE_H
