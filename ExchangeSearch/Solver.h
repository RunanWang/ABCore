#ifndef ABCORE_EXCHANGE_SEARCH_ANYTIME_SOLVER_H
#define ABCORE_EXCHANGE_SEARCH_ANYTIME_SOLVER_H

#include "DataStructure/Subgraph.h"
#include "DataStructure/Graph.h"
#include "constant.h"

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace anytime {

struct AnytimeIncumbent {
    std::vector<NODE_TYPE> blockers; // 当前经过完整 peeling 验证的全局 blocker IDs。
    std::size_t exchange_size{0}; // 产生该解的组合粒度；零表示安全解或初始 greedy。
    double elapsed_time_ms{0.0}; // 从 solveExchangeSearch 起点累计的墙钟毫秒数。
};

struct AnytimeConfig {
    double time_limit_seconds{300.0}; // 包含预处理的 ES 独立墙钟预算，单位秒。
    // 每个 incumbent 首先遍历的组合粒度；粒度一由 reverse deletion 覆盖。
    std::size_t min_exchange_size{2};
    std::size_t max_exchange_size{2}; // 每个 incumbent 最后遍历的组合粒度。
    std::size_t max_combinations{
        std::numeric_limits<std::size_t>::max()}; // 全局最多精确检查的组合数。
    std::function<bool()> should_stop; // 返回 true 时在安全检查点中断搜索。
    std::function<void(const AnytimeIncumbent &)>
        on_incumbent; // 发布首个安全解和后续严格缩小的全局 incumbent。
    std::function<void(std::size_t)> on_radius_complete; // 某半径完成时通知调用方。
};

struct AnytimeComponentResult {
    NODE_TYPE component_index{0}; // 预处理后的稳定 component 编号。
    std::size_t candidate_vertices{0}; // component 内可阻断顶点数。
    std::vector<NODE_TYPE> blockers; // 最终额外 blockers，使用全局顶点 ID。
    std::size_t safe_blocker_count{0}; // 全 boundary 安全初始解大小。
    std::size_t greedy_blocker_count{0}; // 独立 greedy 与 reverse deletion 后的大小。
    std::size_t greedy_incremental_updates{0}; // greedy 正式提交 blocker 的次数。
    std::size_t combinations_checked{0}; // 实际进入 Exact-S repair 的组合数。
    std::size_t combinations_exhausted{0}; // 未找到 repair 而完整穷尽的组合数。
    std::size_t base_residual_builds{0}; // 为 B-S 完整构造 residual 的次数。
    std::size_t repair_search_nodes{0}; // 内层 repair DFS 访问的状态数。
    std::size_t repair_vertices_tried{0}; // DFS 实际尝试标记的 repair 顶点数。
    std::size_t cascade_vertices_removed{0}; // repair DFS 增量移除的顶点总次数。
    // 从 baseline 完整重放的 peeling 次数；不含 greedy 增量提交。
    std::size_t tahp_calls{0};
    std::size_t improvements{0}; // 严格缩小 component incumbent 的次数。
    // 最终 incumbent 已穷尽的最后一个配置粒度；min>2 时不表示更小粒度已覆盖。
    std::size_t completed_exchange_size{0};
    bool proved_optimal{false}; // 是否已覆盖当前解全部非空撤销规模。
};

struct AnytimeSolveResult {
    std::vector<NODE_TYPE> blockers; // forced 与 additional 的最终全局并集。
    std::vector<NODE_TYPE> forced_blockers; // 预处理证明 mandatory 的 blockers。
    std::vector<NODE_TYPE> additional_blockers; // 各 residual component 的 blockers。
    std::vector<AnytimeComponentResult> components; // 分量级搜索统计。
    std::vector<AnytimeIncumbent> incumbent_history; // 单调缩小的已验证解序列。
    std::size_t combinations_checked{0}; // 全局实际检查的组合数。
    bool interrupted{false}; // should_stop、时间或组合预算是否提前终止搜索。
    bool timed_out{false}; // 是否触发独立墙钟预算。
    bool proved_optimal{false}; // 是否已对所有 component 完成全半径认证。
};

// 独立求解固定 (G,C,k) 的 anytime blocker 问题。
// 输入：config 指定遍历的组合粒度、组合预算和可选中断/发布回调。
// 输出：始终返回经过完整 peeling 验证的 blocker，并报告已完成的交换认证。
AnytimeSolveResult solveExchangeSearch(const Graph &graph,
                                const Subgraph &target_subgraph,
                                NODE_TYPE k,
                                const AnytimeConfig &config = {});

}  // namespace anytime

#endif  // ABCORE_EXCHANGE_SEARCH_ANYTIME_SOLVER_H
