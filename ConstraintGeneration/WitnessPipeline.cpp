#include "ConstraintGeneration/WitnessPipeline.h"

#include "ConstraintGeneration/GrowthWitnessSelector.h"
#include "ConstraintGeneration/SlackWitnessSelector.h"

#include <stdexcept>
#include <utility>
#include <vector>

namespace constraint_generation {

WitnessPipeline::WitnessPipeline(const ComponentProblem &problem,
                                 ExactConfig config)
    : problem_(problem),
      config_(std::move(config)),
      peeling_(problem_),
      slack_generator_(problem_),
      growth_generator_(problem_) {
    config_.validate();
}

PipelineResult WitnessPipeline::initial() const {
    const Residual residual = peeling_.run({}); // forced-only fixed point。
    if (residual.feasible()) {
        return PipelineResult::feasibleResidual();
    }

    ConstraintBatch batch; // 第一轮提交给 master 的 typed 约束。
    SlackGenerationResult generated = slack_generator_.generate(residual); // 首轮 Slack cuts。
    if (generated.cuts.empty()) {
        batch.slack_fallback_witnesses = peeling_.targetComponents(residual, 1);

    } else {

        batch.slack_cuts = SlackWitnessSelector::selectForInitial(
            std::move(generated.cuts), config_.slack.initial_batch_limit);

    }

    return PipelineResult::violatedResidual(std::move(batch));
}

PipelineResult WitnessPipeline::separate(
    const std::vector<NODE_TYPE> &blockers) const {
    const Residual residual = peeling_.run(blockers); // 当前完整 fixed point。
    if (residual.feasible()) {
        return PipelineResult::feasibleResidual();
    }

    ConstraintBatch batch; // 本轮按 separation 策略提交的 typed 约束。
    // 1. 分离当前 residual 的正 depth Slack cuts。
    SlackGenerationResult generated_slack = slack_generator_.generate(residual); // 当前解违反的 cuts。

    batch.slack_cuts = SlackWitnessSelector::selectForSeparation(
        std::move(generated_slack.cuts), config_.slack.separation_batch_limit);

    // 2. Growth 独立生成候选，并按 packing 与 coverage 筛选。
    const auto generated = growth_generator_.generate(
        residual, config_.growth.generation_limit); // admission 前候选池。
    auto admitted = GrowthWitnessSelector::select(
        generated, problem_, config_.growth.batch_limit); // 接纳的候选。
    if (admitted.empty()) {
        throw std::logic_error("growth selector rejected every infeasibility witness");
    }

    for (WitnessCandidate &candidate : admitted) {
        batch.witnesses.push_back(std::move(candidate.witness));
    }

    // 3. 不可行结果必须携带当前候选违反的分离约束。

    return PipelineResult::violatedResidual(std::move(batch));
}

}  // namespace constraint_generation
