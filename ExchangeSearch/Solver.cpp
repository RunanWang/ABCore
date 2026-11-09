#include "ExchangeSearch/Solver.h"

#include "ExchangeSearch/ExchangeSearch.h"
#include "ExchangeSearch/Greedy.h"
#include "ExchangeSearch/InternalTypes.h"
#include "ExchangeSearch/Peeling.h"
#include "ExchangeSearch/Preprocessor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace anytime {
namespace {

using Clock = std::chrono::steady_clock;
using internal::Component;

// 检查用户中断回调；空回调表示继续。
// 输入：config 保存可选回调。输出：是否停止启动下一 greedy/exchange 阶段。
bool stopRequested(const AnytimeConfig &config) {
    return config.should_stop && config.should_stop();
}

// 把一组 local blocker IDs 映射为升序 global IDs。
// 输入：component 提供映射。输出：canonical global blocker 集。
std::vector<NODE_TYPE> toGlobal(const Component &component,
                                const std::vector<NODE_TYPE> &local) {
    std::vector<NODE_TYPE> global; // 映射后的 global IDs。
    global.reserve(local.size());
    for (const NODE_TYPE vertex : local) {
        global.push_back(component.vertices.at(vertex));
    }
    std::sort(global.begin(), global.end());
    return global;
}

// 合并 forced blockers 与各 component local incumbents。
// 输入：components 和一一对应的 local solutions。输出：canonical global blocker 集。
std::vector<NODE_TYPE> combineGlobal(
    const std::vector<NODE_TYPE> &forced,
    const std::vector<Component> &components,
    const std::vector<std::vector<NODE_TYPE>> &local_solutions) {
    std::vector<NODE_TYPE> combined = forced; // 正在合并的全局解。
    for (std::size_t index = 0; index < components.size(); ++index) {
        const auto global = toGlobal(components[index], local_solutions[index]);
        combined.insert(combined.end(), global.begin(), global.end());
    }
    std::sort(combined.begin(), combined.end());
    return combined;
}

}  // namespace

AnytimeSolveResult solveExchangeSearch(const Graph &graph,
                                const Subgraph &target_subgraph,
                                const NODE_TYPE k,
                                const AnytimeConfig &input_config) {
    const auto started = Clock::now(); // ES 独立墙钟起点，包含预处理。
    AnytimeConfig config = input_config; // 为内部检查组合限时与用户中断回调。
    if (!std::isfinite(config.time_limit_seconds) || config.time_limit_seconds <= 0)
        throw std::invalid_argument("ES time limit must be positive and finite");
    bool timed_out = false; // 记录是否实际触发时间预算。
    // 输入为捕获的计时配置；输出是否应在安全检查点停止。
    config.should_stop = [&]() {
        if (std::chrono::duration<double>(Clock::now() - started).count() >= config.time_limit_seconds) {
            timed_out = true;
            return true;
        }
        return input_config.should_stop && input_config.should_stop();
    };
    if (config.min_exchange_size == 0 ||
        config.max_exchange_size < config.min_exchange_size) {
        throw std::invalid_argument(
            "anytime exchange sizes must form a positive interval");
    }
    internal::PreprocessingResult preprocessing =
        internal::preprocess(graph, target_subgraph, k); // 独立预处理产物。
    AnytimeSolveResult result; // 最终公开返回值。
    result.forced_blockers = preprocessing.forced_blockers;

    // 与 preprocessing.components 按下标对齐的 local-ID incumbents。
    std::vector<std::vector<NODE_TYPE>> local_solutions;
    local_solutions.reserve(preprocessing.components.size());
    result.components.reserve(preprocessing.components.size());
    for (const Component &component : preprocessing.components) {
        std::vector<NODE_TYPE> safe; // 阻断全部 active boundary 的局部安全解。
        for (NODE_TYPE vertex = 0; vertex < component.size(); ++vertex) {
            if (component.boundary[vertex]) {
                safe.push_back(vertex);
            }
        }
        local_solutions.push_back(std::move(safe));
        AnytimeComponentResult statistics; // 当前 component 的累计统计。
        statistics.component_index = component.index;
        statistics.candidate_vertices = component.vertices.size();
        statistics.safe_blocker_count = local_solutions.back().size();
        statistics.greedy_blocker_count = local_solutions.back().size();
        result.components.push_back(std::move(statistics));
    }

    // 将当前各分量解合并为全局 certificate；仅发布严格缩小的已验证 UB。
    // 输入：exchange_size 为产生本次改进的 |S|，零表示 safe 或 greedy 阶段。
    // 输出：追加历史记录并同步调用用户回调。
    auto publish = [&](const std::size_t exchange_size) {
        AnytimeIncumbent incumbent; // 本次发布的全局可行解快照。
        incumbent.blockers = combineGlobal(
            preprocessing.forced_blockers, preprocessing.components,
            local_solutions);
        incumbent.exchange_size = exchange_size;
        incumbent.elapsed_time_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - started)
                .count();
        if (!result.incumbent_history.empty() &&
            incumbent.blockers.size() >=
                result.incumbent_history.back().blockers.size()) {
            return;
        }
        result.incumbent_history.push_back(incumbent);
        if (config.on_incumbent) {
            config.on_incumbent(result.incumbent_history.back());
        }
    };
    publish(0);

    // 1. 每个 component 独立构造 greedy incumbent；中断时保留安全解。
    for (std::size_t index = 0;
         index < preprocessing.components.size(); ++index) {
        if (stopRequested(config)) {
            result.interrupted = true;
            break;
        }
        const Component &component = preprocessing.components[index];
        internal::PeelingWorkspace workspace(component); // 当前分量工作区。
        auto greedy = internal::buildGreedyIncumbent(
            component, workspace, result.components[index], config);
        if (!greedy) {
            result.interrupted = true;
            break;
        }
        result.components[index].greedy_blocker_count = greedy->size();
        if (greedy->size() < local_solutions[index].size()) {
            local_solutions[index] = std::move(*greedy);
            publish(0);
        } else {
            local_solutions[index] = std::move(*greedy);
        }
    }

    // 2. 在未中断时逐 component 遍历配置的 Exact-S 组合粒度。
    if (!result.interrupted) {
        for (std::size_t index = 0; index < preprocessing.components.size();
             ++index) {
            if (stopRequested(config)) {
                result.interrupted = true;
                break;
            }
            const Component &component = preprocessing.components[index];
            internal::PeelingWorkspace workspace(component); // exchange 工作区。
            // 将 component-local 改进写回对齐数组，再由 publish 合并全局解。
            const auto exchange = internal::runExchangeSearch(
                component, workspace, local_solutions[index],
                result.components[index], config, result.combinations_checked,
                [&](const std::vector<NODE_TYPE> &updated,
                    const std::size_t exchange_size) {
                    local_solutions[index] = updated;
                    publish(exchange_size);
                });
            local_solutions[index] = exchange.blockers;
            if (exchange.interrupted) {
                result.interrupted = true;
                break;
            }
        }
    }

    // 3. 汇总最终全局解与分量统计；仅在全部分量认证后声明全局最优。
    result.proved_optimal = !result.interrupted;
    for (std::size_t index = 0; index < preprocessing.components.size();
         ++index) {
        result.components[index].blockers =
            toGlobal(preprocessing.components[index], local_solutions[index]);
        result.proved_optimal =
            result.proved_optimal && result.components[index].proved_optimal;
        result.additional_blockers.insert(
            result.additional_blockers.end(),
            result.components[index].blockers.begin(),
            result.components[index].blockers.end());
    }
    std::sort(result.additional_blockers.begin(),
              result.additional_blockers.end());
    result.blockers = result.forced_blockers;
    result.blockers.insert(result.blockers.end(),
                           result.additional_blockers.begin(),
                           result.additional_blockers.end());
    std::sort(result.blockers.begin(), result.blockers.end());
    result.timed_out = timed_out;
    return result;
}

}  // namespace anytime
