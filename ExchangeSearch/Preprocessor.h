#ifndef ABCORE_EXCHANGE_SEARCH_PREPROCESSOR_H
#define ABCORE_EXCHANGE_SEARCH_PREPROCESSOR_H

#include "ExchangeSearch/InternalTypes.h"
#include "DataStructure/Subgraph.h"
#include "DataStructure/Graph.h"

namespace anytime::internal {

// 独立计算 forced blockers、peeling fixed point 和目标相关局部 CSR。
// 输入：graph 为原图，target 为目标子图，k 为固定 core 参数。
// 输出：返回只包含 target-related residual components 的预处理结果。
PreprocessingResult preprocess(const Graph &graph,
                               const Subgraph &target,
                               NODE_TYPE k);

}  // namespace anytime::internal

#endif  // ABCORE_EXCHANGE_SEARCH_PREPROCESSOR_H

