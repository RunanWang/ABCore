#include "ExchangeSearch/Peeling.h"

#include <algorithm>
#include <stdexcept>

namespace anytime::internal {

PeelingWorkspace::PeelingWorkspace(const Component &component)
    : component_(component),
      active_(component.size(), 1),
      degrees_(component.baseline_degrees),
      queued_(component.size(), 0),
      visited_(component.size(), 0),
      trial_removed_(component.size(), 0),
      trial_losses_(component.size(), 0) {}

PeelingWorkspace::Result PeelingWorkspace::run(
    const std::vector<NODE_TYPE> &blockers,
    const bool extract_components) {
    // 1. 恢复 baseline，施加完整 blocker 集并初始化受影响度数。
    reset(blockers);
    // 2. 传播全部 degree loss，直到达到唯一 fixed point。
    peelToFixedPoint();
    // 3. 汇总 boundary；greedy 请求时枚举含 boundary 的 residual components。
    return summarize(extract_components);
}

PeelingWorkspace::Result PeelingWorkspace::commitBlocker(
    const NODE_TYPE blocker,
    const bool extract_components) {
    if (!active(blocker)) {
        throw std::invalid_argument(
            "committed anytime blocker must be active");
    }

    // 1. 当前状态已是 fixed point，只需从新增 blocker 的 active 邻居传播损失。
    active_[blocker] = 0;
    peel_queue_.clear();
    for (std::size_t position = component_.offsets[blocker];
         position < component_.offsets[blocker + 1]; ++position) {
        const NODE_TYPE neighbor = component_.neighbors[position];
        if (!active_[neighbor]) {
            continue;
        }
        --degrees_[neighbor];
        enqueueIfUnstable(neighbor);
    }

    // 2. 完成 cascade 并返回新 fixed point 的 greedy 视图。
    peelToFixedPoint();
    return summarize(extract_components);
}

PeelingWorkspace::Result PeelingWorkspace::summarize(
    const bool extract_components) {
    Result result; // 当前 fixed point 的公开摘要。
    for (NODE_TYPE vertex = 0; vertex < component_.size(); ++vertex) {
        if (active_[vertex]) {
            result.active_boundaries += component_.boundary[vertex];
        }
    }
    if (extract_components && !result.feasible()) {
        result.residual_components = extractResidualComponents();
    }
    return result;
}

bool PeelingWorkspace::active(const NODE_TYPE vertex) const {
    return active_.at(vertex) != 0;
}

NODE_TYPE PeelingWorkspace::degree(const NODE_TYPE vertex) const {
    return degrees_.at(vertex);
}

std::size_t PeelingWorkspace::boundaryGainIfBlocked(
    const NODE_TYPE candidate) {
    if (!active(candidate)) {
        throw std::invalid_argument(
            "anytime greedy trial candidate is not active");
    }
    trial_queue_.clear();
    trial_touched_.clear();
    trial_removed_[candidate] = 1;
    trial_queue_.push_back(candidate);
    std::size_t gain = component_.boundary[candidate]; // 本次 boundary 收益。

    // 1. 在临时 loss/removed 数组上模拟 candidate 引发的完整 cascade。
    for (std::size_t head = 0; head < trial_queue_.size(); ++head) {
        const NODE_TYPE vertex = trial_queue_[head];
        for (std::size_t position = component_.offsets[vertex];
             position < component_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor = component_.neighbors[position];
            if (!active(neighbor) || trial_removed_[neighbor]) {
                continue;
            }
            if (trial_losses_[neighbor] == 0) {
                trial_touched_.push_back(neighbor);
            }
            ++trial_losses_[neighbor];
            if (degrees_[neighbor] - trial_losses_[neighbor] >=
                component_.thresholds[neighbor]) {
                continue;
            }
            trial_removed_[neighbor] = 1;
            trial_queue_.push_back(neighbor);
            gain += component_.boundary[neighbor];
        }
    }

    // 2. 只清理本次触及的位置，保留 workspace 的真实 fixed point 不变。
    trial_removed_[candidate] = 0;
    for (const NODE_TYPE vertex : trial_touched_) {
        trial_removed_[vertex] = 0;
        trial_losses_[vertex] = 0;
    }
    return gain;
}

void PeelingWorkspace::reset(const std::vector<NODE_TYPE> &blockers) {
    std::fill(active_.begin(), active_.end(), 1);
    std::fill(queued_.begin(), queued_.end(), 0);
    degrees_ = component_.baseline_degrees;
    peel_queue_.clear();
    for (const NODE_TYPE blocker : blockers) {
        if (blocker >= component_.size()) {
            throw std::invalid_argument(
                "anytime blocker local id is out of range");
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
        enqueueIfUnstable(vertex);
    }
}

void PeelingWorkspace::enqueueIfUnstable(const NODE_TYPE vertex) {
    if (active_[vertex] && !queued_[vertex] &&
        degrees_[vertex] < component_.thresholds[vertex]) {
        queued_[vertex] = 1;
        peel_queue_.push_back(vertex);
    }
}

void PeelingWorkspace::peelToFixedPoint() {
    for (std::size_t head = 0; head < peel_queue_.size(); ++head) {
        const NODE_TYPE vertex = peel_queue_[head];
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
            enqueueIfUnstable(neighbor);
        }
    }
}

std::vector<GreedyResidual> PeelingWorkspace::extractResidualComponents() {
    std::fill(visited_.begin(), visited_.end(), 0);
    std::vector<GreedyResidual> components; // 本轮全部 target-related residuals。
    for (NODE_TYPE start = 0; start < component_.size(); ++start) {
        if (!active_[start] || visited_[start]) {
            continue;
        }
        traversal_queue_.clear();
        traversal_queue_.push_back(start);
        visited_[start] = 1;
        bool contains_boundary = false; // 当前连通块是否含 active boundary。
        GreedyResidual residual; // 当前 active 连通块的 local IDs。
        for (std::size_t head = 0; head < traversal_queue_.size(); ++head) {
            const NODE_TYPE vertex = traversal_queue_[head];
            residual.push_back(vertex);
            contains_boundary =
                contains_boundary || component_.boundary[vertex];
            for (std::size_t position = component_.offsets[vertex];
                 position < component_.offsets[vertex + 1]; ++position) {
                const NODE_TYPE neighbor = component_.neighbors[position];
                if (active_[neighbor] && !visited_[neighbor]) {
                    visited_[neighbor] = 1;
                    traversal_queue_.push_back(neighbor);
                }
            }
        }
        if (contains_boundary) {
            std::sort(residual.begin(), residual.end());
            components.push_back(std::move(residual));
        }
    }
    return components;
}

}  // namespace anytime::internal
