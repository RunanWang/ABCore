#ifndef ABCORE_CONSTRAINT_GENERATION_GROWTH_WITNESS_GENERATOR_H
#define ABCORE_CONSTRAINT_GENERATION_GROWTH_WITNESS_GENERATOR_H

#include "ConstraintGeneration/Types.h"

#include <cstddef>
#include <vector>

namespace constraint_generation {

class GrowthWitnessGenerator {
public:
    // 绑定局部问题，保留 growth 所需的 CSR、阈值与 boundary 属性。
    // 输入：不可变局部问题。输出：可复用的 root-specific 生成器。
    explicit GrowthWitnessGenerator(const ComponentProblem &problem);

    // 从每个 surviving boundary 按两种策略生长并去重 witness。
    // 输入：当前 fixed-point residual 与候选上限。输出：集合及其生成 roots。
    std::vector<WitnessCandidate> generate(
        const Residual &residual,
        std::size_t generation_limit) const;

private:
    enum class Policy : unsigned char {
        Compact,        // 不区分 boundary 的紧凑生长。
        SingleBoundary, // 尽量避免引入第二个 boundary。
    };

    // 计算一个点在给定 active 集合中的诱导度。
    // 输入：local ID 与 active 位图。输出：active 邻居数。
    NODE_TYPE activeDegree(
        NODE_TYPE vertex,
        const std::vector<unsigned char> &active) const;

    // 按“更紧的边界优先”确定 growth roots 的稳定顺序。
    // 输入：active 位图。输出：排序后的 active boundary IDs。
    std::vector<NODE_TYPE> selectRoots(
        const std::vector<unsigned char> &active) const;

    // 从一个 boundary root 生长出满足全部局部阈值的连通 witness。
    // 输入：root、候选偏好和 active 位图。输出：canonical witness。
    Witness growWitness(NODE_TYPE root,
                        Policy policy,
                        const std::vector<unsigned char> &active) const;

    const ComponentProblem &problem_; // growth 使用的不可变局部问题。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_GROWTH_WITNESS_GENERATOR_H
