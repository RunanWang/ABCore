#ifndef ABCORE_EXCHANGE_SEARCH_PEELING_H
#define ABCORE_EXCHANGE_SEARCH_PEELING_H

#include "ExchangeSearch/InternalTypes.h"

#include <cstddef>
#include <vector>

namespace anytime::internal {

// 可重复从 forced-only component baseline 执行完整 peeling。
// run()/commitBlocker() 结束后 active()/degree() 暴露最近 fixed point，供 greedy
// 候选评分读取；boundaryGainIfBlocked() 只使用临时数组模拟，不改变该 fixed point。
class PeelingWorkspace {
public:
    struct Result {
        std::size_t active_boundaries{0}; // fixed point 中仍存活的 boundary 数。
        // Greedy 评分使用的 target-related residual connected components。
        std::vector<GreedyResidual> residual_components;

        bool feasible() const noexcept {
            return active_boundaries == 0;
        }
    };

    // 为一个 component 建立可重复重放的独立 peeling 工作区。
    // 输入：component 保存局部 CSR 和 baseline。输出：初始化全部临时数组。
    explicit PeelingWorkspace(const Component &component);

    // 从 baseline 重放 blockers 并 peeling 到 fixed point。
    // 输入：blockers 使用 local IDs；extract_components 控制是否为 greedy 枚举
    // target-related residual connected components。
    // 输出：feasibility、active boundary 数和可选 residual components。
    Result run(const std::vector<NODE_TYPE> &blockers,
               bool extract_components);

    // 在当前 fixed point 上永久增加一个 blocker，并只传播新增 degree loss。
    // 输入：blocker 必须是 active local ID；extract_components 控制 greedy residual
    // 枚举。输出：更新后的 feasibility、active boundary 数和可选 residuals。
    Result commitBlocker(NODE_TYPE blocker, bool extract_components);

    bool active(NODE_TYPE vertex) const;
    NODE_TYPE degree(NODE_TYPE vertex) const;

    // 在当前 fixed point 上增量模拟阻断一个 active candidate。
    // 输入：candidate 为 local ID。输出：直接与级联移除的 boundary 数。
    std::size_t boundaryGainIfBlocked(NODE_TYPE candidate);

private:
    // 恢复 baseline 并施加完整 local blocker 集。
    void reset(const std::vector<NODE_TYPE> &blockers);
    // 将当前低于 threshold 且尚未入队的点加入 peeling 队列。
    void enqueueIfUnstable(NODE_TYPE vertex);
    // 传播队列中的 degree loss 直到 fixed point。
    void peelToFixedPoint();
    // 汇总当前 fixed point；按需为 greedy 枚举 residual components。
    Result summarize(bool extract_components);
    // 枚举当前 residual 内全部含 active boundary 的 connected components。
    std::vector<GreedyResidual> extractResidualComponents();

    const Component &component_; // 当前局部问题。
    std::vector<unsigned char> active_; // 最近一次 TAHP fixed point 位图。
    std::vector<NODE_TYPE> degrees_; // 最近一次 fixed point 的活动度数。
    std::vector<unsigned char> queued_; // 本轮 peeling 入队去重位图。
    std::vector<NODE_TYPE> peel_queue_; // 完整 peeling 队列。
    std::vector<unsigned char> visited_; // residual component BFS 访问位图。
    std::vector<NODE_TYPE> traversal_queue_; // residual component BFS 队列。
    std::vector<unsigned char> trial_removed_; // 单候选模拟的临时移除位图。
    std::vector<NODE_TYPE> trial_losses_; // 单候选模拟的临时 degree loss。
    std::vector<NODE_TYPE> trial_queue_; // 单候选级联队列。
    std::vector<NODE_TYPE> trial_touched_; // 需要恢复 trial loss 的顶点。
};

}  // namespace anytime::internal

#endif  // ABCORE_EXCHANGE_SEARCH_PEELING_H
