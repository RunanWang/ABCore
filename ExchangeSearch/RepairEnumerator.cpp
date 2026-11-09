#include "ExchangeSearch/RepairEnumerator.h"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace anytime::internal {
namespace {

// 固定一个撤销集合 S 后，维护 H_K=Peel(R-K) 上的单调删除状态。
// DFS 向下只会删除顶点；DFS 中每条 active 邻边造成的 degree decrement 和每个
// 删除顶点分别记入日志，因此兄弟分支可以恢复到完全相同的 fixed point。
class RollbackPeelingState {
public:
    struct Checkpoint {
        std::size_t removed_size{0}; // removed_log_ 的恢复位置。
        std::size_t decrement_size{0}; // degree_decrement_log_ 的恢复位置。
        std::size_t active_boundaries{0}; // 分支前的 active boundary 数。
    };

    // 从 component baseline 施加 fixed blockers 并 peeling 到不动点。
    // 输入：fixed 为当前组合保留的 B-S。输出：可供递归删除和回滚的 residual 状态。
    RollbackPeelingState(const Component &component,
                         const std::vector<NODE_TYPE> &fixed)
        : component_(component),
          active_(component.size(), 1),
          degrees_(component.baseline_degrees) {
        initialize(fixed);
    }

    bool active(const NODE_TYPE vertex) const {
        return active_.at(vertex) != 0;
    }

    bool feasible() const noexcept {
        return active_boundaries_ == 0;
    }

    // 保存当前递归状态的回滚位置。
    // 输入：当前日志。输出：之后可传给 rollback() 的轻量 checkpoint。
    Checkpoint checkpoint() const noexcept {
        return {removed_log_.size(), degree_decrement_log_.size(),
                active_boundaries_};
    }

    // 阻断一个 active 点并增量传播全部 cascade peeling。
    // 输入：vertex 为当前 repair candidate。输出：本分支新移除的顶点数。
    std::size_t blockAndCascade(const NODE_TYPE vertex) {
        if (!active(vertex)) {
            throw std::invalid_argument(
                "repair enumeration candidate is not active");
        }
        queue_.clear();
        removeAndQueue(vertex);
        for (std::size_t head = 0; head < queue_.size(); ++head) {
            const NODE_TYPE removed = queue_[head];
            for (std::size_t position = component_.offsets[removed];
                 position < component_.offsets[removed + 1]; ++position) {
                const NODE_TYPE neighbor = component_.neighbors[position];
                if (!active_[neighbor]) {
                    continue;
                }
                degree_decrement_log_.push_back(neighbor);
                --degrees_[neighbor];
                if (degrees_[neighbor] < component_.thresholds[neighbor]) {
                    removeAndQueue(neighbor);
                }
            }
        }
        return queue_.size();
    }

    // 将状态恢复到指定递归 checkpoint。
    // 输入：checkpoint 必须来自当前状态的祖先。输出：撤销其后的 active/degree 变化。
    void rollback(const Checkpoint &checkpoint) {
        if (checkpoint.removed_size > removed_log_.size() ||
            checkpoint.decrement_size > degree_decrement_log_.size()) {
            throw std::invalid_argument(
                "repair rollback checkpoint is not an ancestor");
        }
        // 先恢复所有度数，再重新激活顶点；这样恢复期间无需临时维护邻接一致性。
        while (degree_decrement_log_.size() > checkpoint.decrement_size) {
            ++degrees_[degree_decrement_log_.back()];
            degree_decrement_log_.pop_back();
        }
        while (removed_log_.size() > checkpoint.removed_size) {
            active_[removed_log_.back()] = 1;
            removed_log_.pop_back();
        }
        active_boundaries_ = checkpoint.active_boundaries;
    }

private:
    // 从 baseline 完整构造 fixed blockers 对应的 residual。
    // 输入：fixed 使用 local IDs。输出：active_、degrees_ 和 active_boundaries_。
    void initialize(const std::vector<NODE_TYPE> &fixed) {
        std::vector<NODE_TYPE> initial_queue; // fixed 后的初始 peeling 队列。
        std::vector<unsigned char> queued(component_.size(), 0); // 入队位图。

        // 1. 从 baseline 移除 K，并据此修正其余 active 顶点的初始度数。
        for (const NODE_TYPE blocker : fixed) {
            if (blocker >= component_.size()) {
                throw std::invalid_argument(
                    "fixed repair blocker local id is out of range");
            }
            active_[blocker] = 0;
        }
        for (NODE_TYPE vertex = 0; vertex < component_.size(); ++vertex) {
            if (!active_[vertex]) {
                continue;
            }
            for (std::size_t position = component_.offsets[vertex];
                 position < component_.offsets[vertex + 1]; ++position) {
                degrees_[vertex] -= !active_[component_.neighbors[position]];
            }
            if (degrees_[vertex] < component_.thresholds[vertex]) {
                queued[vertex] = 1;
                initial_queue.push_back(vertex);
            }
        }

        // 2. 传播所有低于异质阈值的顶点，得到唯一 base residual H_K。
        for (std::size_t head = 0; head < initial_queue.size(); ++head) {
            const NODE_TYPE vertex = initial_queue[head];
            if (!active_[vertex]) {
                continue;
            }
            active_[vertex] = 0;
            for (std::size_t position = component_.offsets[vertex];
                 position < component_.offsets[vertex + 1]; ++position) {
                const NODE_TYPE neighbor = component_.neighbors[position];
                if (!active_[neighbor]) {
                    continue;
                }
                --degrees_[neighbor];
                if (!queued[neighbor] &&
                    degrees_[neighbor] < component_.thresholds[neighbor]) {
                    queued[neighbor] = 1;
                    initial_queue.push_back(neighbor);
                }
            }
        }

        // 3. 初始化 DFS 的 O(1) feasibility 判定计数器。
        for (NODE_TYPE vertex = 0; vertex < component_.size(); ++vertex) {
            if (active_[vertex]) {
                active_boundaries_ += component_.boundary[vertex];
            }
        }
    }

    // 将一个 active 点记入删除日志和当前 cascade 队列。
    // 输入：vertex 尚未删除。输出：active boundary 计数同步减少。
    void removeAndQueue(const NODE_TYPE vertex) {
        active_[vertex] = 0;
        removed_log_.push_back(vertex);
        queue_.push_back(vertex);
        active_boundaries_ -= component_.boundary[vertex];
    }

    const Component &component_; // 当前局部问题。
    std::vector<unsigned char> active_; // 当前 DFS 分支的 residual 位图。
    std::vector<NODE_TYPE> degrees_; // 当前 DFS 分支的 active degree。
    std::size_t active_boundaries_{0}; // 当前仍存活的 boundary 数。
    std::vector<NODE_TYPE> removed_log_; // DFS 当前路径移除的顶点日志。
    // 每次 active 邻边引起一次记录，同一顶点可以重复出现。
    std::vector<NODE_TYPE> degree_decrement_log_;
    std::vector<NODE_TYPE> queue_; // 单次 blockAndCascade 的传播队列。
};

enum class SearchStatus : unsigned char {
    Found,
    Exhausted,
    Interrupted,
};

// 检查用户中断回调；空回调表示继续。
// 输入：config 保存可选回调。输出：是否停止当前 DFS。
bool stopRequested(const AnytimeConfig &config) {
    return config.should_stop && config.should_stop();
}

// 深度优先枚举 canonical repair subsets，并复用前缀 residual 状态。
// 输入：start 保证组合不重复，remaining 为剩余 blocker 预算。
// 输出：找到可行 repair、穷尽当前子树或收到中断。
SearchStatus searchRepair(
    RollbackPeelingState &state,
    const std::vector<NODE_TYPE> &candidates,
    const std::size_t start,
    const std::size_t remaining,
    std::vector<NODE_TYPE> &selected,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config) {
    ++statistics.repair_search_nodes;
    if (state.feasible()) {
        return SearchStatus::Found;
    }
    if (stopRequested(config)) {
        return SearchStatus::Interrupted;
    }
    if (remaining == 0) {
        return SearchStatus::Exhausted;
    }

    // 静态下标递增保证每个 T 只枚举一次；已级联删除的候选没有额外作用。
    for (std::size_t index = start; index < candidates.size(); ++index) {
        const NODE_TYPE candidate = candidates[index];
        if (!state.active(candidate)) {
            continue;
        }
        const auto checkpoint = state.checkpoint(); // 当前候选分支的恢复点。
        selected.push_back(candidate);
        ++statistics.repair_vertices_tried;
        statistics.cascade_vertices_removed +=
            state.blockAndCascade(candidate);
        const SearchStatus status = searchRepair(
            state, candidates, index + 1, remaining - 1, selected,
            statistics, config);
        if (status == SearchStatus::Found) {
            return status;
        }
        selected.pop_back();
        state.rollback(checkpoint);
        if (status == SearchStatus::Interrupted) {
            return status;
        }
    }
    return SearchStatus::Exhausted;
}

}  // namespace

RepairEnumerationResult enumerateRepair(
    const Component &component,
    const std::vector<NODE_TYPE> &incumbent,
    const std::vector<NODE_TYPE> &fixed,
    const std::size_t budget,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config) {
    // 1. 显式校验 Exact-S 不变量：K subset B，且两个集合内部都没有重复点。
    std::vector<unsigned char> in_incumbent(
        component.size(), 0); // 禁止 repair 重新选择 B 中的点。
    for (const NODE_TYPE blocker : incumbent) {
        if (blocker >= component.size() || in_incumbent[blocker]) {
            throw std::invalid_argument(
                "repair incumbent must contain unique valid local ids");
        }
        in_incumbent[blocker] = 1;
    }
    std::vector<unsigned char> in_fixed(
        component.size(), 0); // K 的成员与去重校验位图。
    for (const NODE_TYPE blocker : fixed) {
        if (blocker >= component.size() || !in_incumbent[blocker] ||
            in_fixed[blocker]) {
            throw std::invalid_argument(
                "fixed repair blockers must be a unique incumbent subset");
        }
        in_fixed[blocker] = 1;
    }

    // 2. 每个 S 只从 baseline 构造一次 H_K，后续分支全部增量删除并回滚。
    RollbackPeelingState state(component, fixed); // B-S 对应的 base residual。
    ++statistics.base_residual_builds;
    ++statistics.tahp_calls;

    // 3. 只枚举 H_K-B；base residual 外的顶点即使标记也不会改变 peeling 结果。
    std::vector<NODE_TYPE> candidates; // base residual 中可加入 T 的 local IDs。
    for (NODE_TYPE vertex = 0; vertex < component.size(); ++vertex) {
        if (state.active(vertex) && !in_incumbent[vertex]) {
            candidates.push_back(vertex);
        }
    }

    std::vector<NODE_TYPE> selected; // 当前 DFS 分支选择的 repair blockers T。
    const SearchStatus status = searchRepair(
        state, candidates, 0, budget, selected, statistics, config);
    if (status == SearchStatus::Found) {
        return {std::move(selected), false};
    }
    return {std::nullopt, status == SearchStatus::Interrupted};
}

}  // namespace anytime::internal
