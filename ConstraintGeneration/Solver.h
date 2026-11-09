#pragma once
#include "ConstraintGeneration/ExactConfig.h"
#include "ConstraintGeneration/Types.h"
class Subgraph;
class Graph;
namespace constraint_generation {
class Solver {
public:
    // 求解固定实例的最小外部 blockers。
    // 输入：无向图 graph、连通目标 target、正阈值 k 和候选预算 config。
    // 输出：已证明的全局下界及最优性状态；超时保留当前下界。
    SolveResult solve(const Graph &graph, const Subgraph &target, NODE_TYPE k,
                      ExactConfig config = {}) const;
};
}
