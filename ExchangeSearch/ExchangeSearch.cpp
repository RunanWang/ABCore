#include "ExchangeSearch/ExchangeSearch.h"

#include "ExchangeSearch/Greedy.h"
#include "ExchangeSearch/RepairEnumerator.h"

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace anytime::internal {
namespace {

struct RemovalSearchResult {
    std::optional<std::vector<NODE_TYPE>> improved; // 新的 local incumbent。
    bool interrupted{false}; // 是否在 repair 枚举完成前收到中断。
};

// 检查用户中断回调；空回调表示继续。
// 输入：config 保存可选回调。输出：是否停止当前组合搜索。
bool stopRequested(const AnytimeConfig &config) {
    return config.should_stop && config.should_stop();
}

// 为固定撤销组合 S 构造 K=B-S，并枚举至多 |S|-1 个 repair blockers T。
// 输入：removed_positions 指向 incumbent B 中被撤销的位置。
// 输出：找到严格改进、穷尽该组合，或在安全检查点中断。
RemovalSearchResult searchRemovalSet(
    const Component &component,
    const std::vector<NODE_TYPE> &incumbent,
    const std::vector<std::size_t> &removed_positions,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config) {
    std::vector<unsigned char> in_removed_set(
        component.size(), 0); // S 在 local vertex domain 上的位图。
    for (const std::size_t position : removed_positions) {
        in_removed_set[incumbent.at(position)] = 1;
    }
    std::vector<NODE_TYPE> fixed; // K=B-S，在整个 repair DFS 中固定阻断。
    for (const NODE_TYPE blocker : incumbent) {
        if (!in_removed_set[blocker]) {
            fixed.push_back(blocker);
        }
    }

    const auto enumerated = enumerateRepair(
        component, incumbent, fixed, removed_positions.size() - 1,
        statistics, config);
    if (enumerated.interrupted) {
        return {std::nullopt, true};
    }
    if (!enumerated.repair) {
        ++statistics.combinations_exhausted;
        return {};
    }
    std::vector<NODE_TYPE> improved = fixed; // K 与找到的 T 的并集。
    improved.insert(improved.end(), enumerated.repair->begin(),
                    enumerated.repair->end());
    std::sort(improved.begin(), improved.end());
    return {std::move(improved), false};
}

// 生成固定大小组合的字典序后继。
// 输入：positions 为 [0,n) 中严格递增的组合。输出：成功更新时 true，末尾 false。
bool nextCombination(std::vector<std::size_t> &positions,
                     const std::size_t n) {
    for (std::size_t reverse = positions.size(); reverse > 0; --reverse) {
        const std::size_t index = reverse - 1; // 当前尝试右移的位置。
        const std::size_t maximum = n - positions.size() + index; // 该位上界。
        if (positions[index] == maximum) {
            continue;
        }
        ++positions[index];
        for (std::size_t suffix = index + 1; suffix < positions.size();
             ++suffix) {
            positions[suffix] = positions[suffix - 1] + 1;
        }
        return true;
    }
    return false;
}

}  // namespace

ExchangeRunResult runExchangeSearch(
    const Component &component,
    PeelingWorkspace &workspace,
    std::vector<NODE_TYPE> initial,
    AnytimeComponentResult &statistics,
    const AnytimeConfig &config,
    std::size_t &global_combinations,
    const std::function<void(const std::vector<NODE_TYPE> &, std::size_t)> &
        publish) {
    std::vector<NODE_TYPE> incumbent = std::move(initial); // 当前 local UB。
    while (true) {
        // 1. 每轮都围绕最新 incumbent 搜索；改进后旧半径证书全部失效。
        if (stopRequested(config)) {
            return {std::move(incumbent), true};
        }
        if (incumbent.size() <= 1) {
            statistics.proved_optimal = true;
            return {std::move(incumbent), false};
        }
        bool improved = false; // 当前 incumbent 是否在本轮找到严格改进。
        statistics.completed_exchange_size = 1;

        // 2. 按递增半径 r 枚举 S subset B；每个 S 的 repair budget 为 r-1。
        const std::size_t first_size =
            std::max<std::size_t>(2, config.min_exchange_size);
        const std::size_t last_size =
            std::min(config.max_exchange_size, incumbent.size());
        for (std::size_t size = first_size; size <= last_size; ++size) {
            std::vector<std::size_t> positions(size); // 当前 S 的 incumbent 下标。
            for (std::size_t index = 0; index < size; ++index) {
                positions[index] = index;
            }
            do {
                if (global_combinations >= config.max_combinations ||
                    stopRequested(config)) {
                    return {std::move(incumbent), true};
                }
                ++global_combinations;
                ++statistics.combinations_checked;
                const auto searched = searchRemovalSet(
                    component, incumbent, positions, statistics, config);
                if (searched.interrupted) {
                    return {std::move(incumbent), true};
                }
                if (searched.improved) {
                    incumbent = reverseDelete(
                        workspace, std::move(*searched.improved), statistics);
                    ++statistics.improvements;
                    statistics.completed_exchange_size = 1;
                    publish(incumbent, size);
                    improved = true;
                    break;
                }
            } while (nextCombination(positions, incumbent.size()));
            if (improved) {
                break;
            }
            statistics.completed_exchange_size = size;
            if (config.on_radius_complete) {
                config.on_radius_complete(size);
            }
        }
        if (improved) {
            continue;
        }
        // 3. 仅当 reverse deletion 和 2..|B| 均穷尽时才能声明全局最优。
        statistics.proved_optimal =
            config.min_exchange_size <= 2 &&
            statistics.completed_exchange_size >= incumbent.size();
        return {std::move(incumbent), false};
    }
}

}  // namespace anytime::internal
