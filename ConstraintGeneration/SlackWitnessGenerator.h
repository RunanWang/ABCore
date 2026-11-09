#ifndef ABCORE_CONSTRAINT_GENERATION_SLACK_WITNESS_GENERATOR_H
#define ABCORE_CONSTRAINT_GENERATION_SLACK_WITNESS_GENERATOR_H

#include "ConstraintGeneration/Types.h"

#include <vector>

namespace constraint_generation {

struct SlackGenerationResult {
    std::vector<SlackCut> cuts; // 每个正 depth boundary 的 maximal-slack cut。
    NODE_TYPE max_slack{0};     // 所有 boundary 的最大 slack 深度。
};

class SlackWitnessGenerator {
public:
    // 绑定局部问题；生成过程不会改变 problem 或 residual。
    // 输入：预处理产生的局部问题。输出：可复用的 slack 生成器。
    explicit SlackWitnessGenerator(const ComponentProblem &problem);

    // 对 fixed-point residual 做 generalized core/slack 分解。
    // 输入：通常为 forced-only residual。输出：仅保留正 depth 加权 cuts。
    SlackGenerationResult generate(const Residual &residual) const;

private:
    const ComponentProblem &problem_; // decomposition 使用的异质阈值图。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_SLACK_WITNESS_GENERATOR_H
