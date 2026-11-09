#ifndef ABCORE_CONSTRAINT_GENERATION_COMPONENT_PEELING_H
#define ABCORE_CONSTRAINT_GENERATION_COMPONENT_PEELING_H

#include "ConstraintGeneration/Types.h"

#include <cstddef>
#include <vector>

namespace constraint_generation {

class ComponentPeeling {
public:
    // 绑定一个不可变局部问题并检查其 CSR 与属性维度。
    // 输入：预处理产生的局部问题。输出：可复用的 peeling 运算器。
    explicit ComponentPeeling(const ComponentProblem &problem);

    // 删除给定 blockers 后执行异质阈值 fixed-point peeling。
    // 输入：local blocker IDs，可无序且可重复。输出：完整 residual 快照。
    Residual run(const std::vector<NODE_TYPE> &blockers) const;

    // 从 residual 中提取仍含 boundary 的 active 连通分量。
    // 输入：合法 fixed point 与数量上限。输出：canonical local-ID witnesses。
    std::vector<Witness> targetComponents(
        const Residual &residual,
        std::size_t limit = static_cast<std::size_t>(-1)) const;

    // 检查 residual 的维度、度数、计数及 fixed-point 不变量。
    // 输入：待检查快照。输出：合法时返回；非法时抛出 invalid_argument。
    void validateResidual(const Residual &residual) const;

    // 检查 witness 是否非空、连通、含 boundary 且满足全部阈值。
    // 输入：canonical local-ID 集合。输出：合法时返回；非法时抛出。
    void validateWitness(const Witness &witness) const;

private:
    const ComponentProblem &problem_; // 所有操作共享的不可变局部问题。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_COMPONENT_PEELING_H
