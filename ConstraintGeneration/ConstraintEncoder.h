#ifndef ABCORE_CONSTRAINT_GENERATION_CONSTRAINT_ENCODER_H
#define ABCORE_CONSTRAINT_GENERATION_CONSTRAINT_ENCODER_H

#include "ConstraintGeneration/Types.h"

namespace constraint_generation {

// 把普通 witness 编码为全系数 1、下界 1 的稀疏 master 行。
// 输入：canonical witness 与局部变量数。输出：求解器无关的 MasterRow。
MasterRow encodeWitnessRow(const Witness &witness, NODE_TYPE vertex_count);

// 把 maximal-slack cut 编码为 boundary 系数和 RHS 均为 s+1 的稀疏行。
// 输入：保留 component/boundary/slack 的 cut 与变量数。输出：MasterRow。
MasterRow encodeSlackCut(const SlackCut &cut, NODE_TYPE vertex_count);

}  // namespace constraint_generation

#endif  // ABCORE_CONSTRAINT_GENERATION_CONSTRAINT_ENCODER_H
