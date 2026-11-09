#ifndef ABCORE_CONSTRAINT_GENERATION_GROWTH_WITNESS_SELECTOR_H
#define ABCORE_CONSTRAINT_GENERATION_GROWTH_WITNESS_SELECTOR_H

#include "ConstraintGeneration/ExactConfig.h"
#include "ConstraintGeneration/Types.h"

#include <cstddef>
#include <vector>

namespace constraint_generation {

class GrowthWitnessSelector {
public:
    // 对已生成候选做纯 admission；只读取 problem 以校验 root 作用域。
    // 输入：候选、局部问题、策略与上限。输出：被接纳的候选副本。
    static std::vector<WitnessCandidate> select(
        const std::vector<WitnessCandidate> &candidates,
        const ComponentProblem &problem,
        std::size_t batch_limit);
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_GROWTH_WITNESS_SELECTOR_H
