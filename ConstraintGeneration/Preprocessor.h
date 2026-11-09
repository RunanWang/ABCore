#ifndef ABCORE_CONSTRAINT_GENERATION_PREPROCESSOR_H
#define ABCORE_CONSTRAINT_GENERATION_PREPROCESSOR_H

#include "ConstraintGeneration/Types.h"

class Subgraph;
class Graph;

namespace constraint_generation {

class Preprocessor {
public:
    // 构造固定 (G,C,k) 的独立预处理器。
    // 输入：只读图、目标子图与正整数阈值。输出：可重复调用 run 的对象。
    Preprocessor(const Graph &graph,
                 const Subgraph &target_subgraph,
                 NODE_TYPE k);

    // 计算无条件必要的 blocker，并按 residual 连通分量建立局部问题。
    // 输入：构造时给定的实例。输出：global forced blockers 与局部 CSR 问题。
    PreprocessingResult run() const;

private:
    const Graph &graph_;                       // 仅通过 CSR 公共接口读图。
    const Subgraph &target_subgraph_;       // 待隔离的目标子图。
    NODE_TYPE k_{0};                          // 固定 core 阈值。
};

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_PREPROCESSOR_H
