#include "ExchangeSearch/Preprocessor.h"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace anytime::internal {
namespace {

enum class GlobalVertexState : unsigned char {
    Subgraph, // 目标子图内部点，不进入 blocker domain。
    Residual, // forced-only peeling 后仍可能存活的外部点。
    Unstable, // 已进入 peeling 队列、尚未完成邻居更新的外部点。
    Forced, // threshold=0 的 boundary，必须直接阻断。
    Peeled, // forced-only peeling 已删除的外部点。
};

class Preprocessor {
public:
    // 初始化独立预处理状态。
    // 输入：graph 为原图，target 为目标子图，k 为固定 core 参数。
    // 输出：构造函数只建立缓冲区，run() 执行实际预处理。
    Preprocessor(const Graph &graph,
                 const Subgraph &target,
                 const NODE_TYPE k)
        : graph_(graph),
          k_(k),
          states_(graph.nodesNum, GlobalVertexState::Residual),
          boundary_(graph.nodesNum, 0),
          thresholds_(graph.nodesNum, 0),
          degrees_(graph.nodesNum, 0),
          local_index_(graph.nodesNum, kNoIndex) {
        target.validate(graph);
        for (const NODE_TYPE vertex : target.vertices()) {
            states_[vertex] = GlobalVertexState::Subgraph;
        }
    }

    // 执行 forced、peeling 和 component extraction 三阶段预处理。
    // 输入：构造时保存的 (G,C,k)。输出：局部 CSR 与 mandatory blockers。
    PreprocessingResult run() {
        // 1. 计算阈值并识别 threshold 为零的 mandatory boundary blockers。
        initializeThresholds();
        // 2. 在剩余外部图上建立度数并执行 forced-only peeling。
        initializeDegrees();
        peelForcedOnly();
        // 3. 仅保留 fixed point 中包含 active boundary 的连通分量。
        extractComponents();
        return {std::move(forced_blockers_), std::move(components_)};
    }

private:
    // 读取并校验一个 CSR 邻居。
    // 输入：position 为 graph.edges 下标。输出：合法 global vertex ID。
    NODE_TYPE checkedNeighbor(const EDGE_TYPE position) const {
        const NODE_TYPE neighbor = graph_.edges[position];
        if (neighbor >= graph_.nodesNum) {
            throw std::runtime_error(
                "anytime preprocessing encountered an invalid CSR neighbor");
        }
        return neighbor;
    }

    // 计算外部阈值、boundary 标记与 forced blockers。
    // 输入：目标子图状态和 k。输出：更新 thresholds_、boundary_ 和 states_。
    void initializeThresholds() {
        for (NODE_TYPE vertex = 0; vertex < graph_.nodesNum; ++vertex) {
            if (states_[vertex] == GlobalVertexState::Subgraph) {
                continue;
            }
            NODE_TYPE subgraph_degree = 0; // 当前点连接目标子图的边数。
            for (EDGE_TYPE position = graph_.start(vertex);
                 position < graph_.end(vertex); ++position) {
                subgraph_degree +=
                    states_[checkedNeighbor(position)] ==
                    GlobalVertexState::Subgraph;
            }
            boundary_[vertex] = subgraph_degree != 0;
            thresholds_[vertex] =
                subgraph_degree < k_ ? k_ - subgraph_degree : 0;
            if (boundary_[vertex] && thresholds_[vertex] == 0) {
                states_[vertex] = GlobalVertexState::Forced;
                forced_blockers_.push_back(vertex);
            }
        }
    }

    // 统计尚未 forced 的外部顶点诱导度数。
    // 输入：全局状态。输出：degrees_ 保存 residual 外部度数。
    void initializeDegrees() {
        for (NODE_TYPE vertex = 0; vertex < graph_.nodesNum; ++vertex) {
            if (states_[vertex] != GlobalVertexState::Residual) {
                continue;
            }
            for (EDGE_TYPE position = graph_.start(vertex);
                 position < graph_.end(vertex); ++position) {
                degrees_[vertex] +=
                    states_[checkedNeighbor(position)] ==
                    GlobalVertexState::Residual;
            }
        }
    }

    // 把当前低于阈值的 residual 点加入 peeling 队列。
    // 输入：vertex 为 global ID。输出：必要时更新状态并追加队列。
    void enqueueIfUnstable(const NODE_TYPE vertex) {
        if (states_[vertex] == GlobalVertexState::Residual &&
            degrees_[vertex] < thresholds_[vertex]) {
            states_[vertex] = GlobalVertexState::Unstable;
            peel_queue_.push_back(vertex);
        }
    }

    // 从 forced blocker 状态执行异质 peeling 到 fixed point。
    // 输入：当前 residual 度数。输出：Peeled 状态和 fixed-point degrees_。
    void peelForcedOnly() {
        for (NODE_TYPE vertex = 0; vertex < graph_.nodesNum; ++vertex) {
            enqueueIfUnstable(vertex);
        }
        for (std::size_t head = 0; head < peel_queue_.size(); ++head) {
            const NODE_TYPE vertex = peel_queue_[head];
            states_[vertex] = GlobalVertexState::Peeled;
            for (EDGE_TYPE position = graph_.start(vertex);
                 position < graph_.end(vertex); ++position) {
                const NODE_TYPE neighbor = checkedNeighbor(position);
                if (states_[neighbor] != GlobalVertexState::Residual) {
                    continue;
                }
                --degrees_[neighbor];
                enqueueIfUnstable(neighbor);
            }
        }
    }

    // 把一组 global vertices 压缩成独立局部 CSR。
    // 输入：vertices 为一个 target-related component。输出：完整局部问题。
    Component buildComponent(std::vector<NODE_TYPE> vertices) {
        std::sort(vertices.begin(), vertices.end());
        Component component; // 正在构造的值对象。
        component.index = static_cast<NODE_TYPE>(components_.size() + 1);
        component.vertices = std::move(vertices);

        // 1. 临时登记本分量的 global-to-local 映射。
        for (std::size_t local = 0; local < component.vertices.size(); ++local) {
            local_index_[component.vertices[local]] = local;
        }

        // 2. 只复制分量内部邻边，形成 local-ID CSR 与对齐的顶点属性。
        component.offsets.push_back(0);
        for (const NODE_TYPE vertex : component.vertices) {
            component.thresholds.push_back(thresholds_[vertex]);
            component.boundary.push_back(boundary_[vertex]);
            for (EDGE_TYPE position = graph_.start(vertex);
                 position < graph_.end(vertex); ++position) {
                const std::size_t local =
                    local_index_[checkedNeighbor(position)];
                if (local != kNoIndex) {
                    component.neighbors.push_back(
                        static_cast<NODE_TYPE>(local));
                }
            }
            component.offsets.push_back(component.neighbors.size());
            component.baseline_degrees.push_back(
                static_cast<NODE_TYPE>(component.offsets.back() -
                                       component.offsets[
                                           component.offsets.size() - 2]));
        }

        // 3. 清理临时映射，避免下一分量误把跨分量邻居识别为内部点。
        for (const NODE_TYPE vertex : component.vertices) {
            local_index_[vertex] = kNoIndex;
        }
        return component;
    }

    // 从每个 active boundary 出发提取一次 residual connected component。
    // 输入：forced-only fixed point。输出：components_ 只含目标相关分量。
    void extractComponents() {
        std::vector<unsigned char> visited(graph_.nodesNum, 0); // 全局 BFS 标记。
        std::vector<NODE_TYPE> queue; // 当前全局连通分量 BFS 队列。
        for (NODE_TYPE start = 0; start < graph_.nodesNum; ++start) {
            if (states_[start] != GlobalVertexState::Residual ||
                !boundary_[start] || visited[start]) {
                continue;
            }
            queue.clear();
            queue.push_back(start);
            visited[start] = 1;
            std::vector<NODE_TYPE> vertices; // 当前 target-related component。
            for (std::size_t head = 0; head < queue.size(); ++head) {
                const NODE_TYPE vertex = queue[head];
                vertices.push_back(vertex);
                for (EDGE_TYPE position = graph_.start(vertex);
                     position < graph_.end(vertex); ++position) {
                    const NODE_TYPE neighbor = checkedNeighbor(position);
                    if (states_[neighbor] == GlobalVertexState::Residual &&
                        !visited[neighbor]) {
                        visited[neighbor] = 1;
                        queue.push_back(neighbor);
                    }
                }
            }
            components_.push_back(buildComponent(std::move(vertices)));
        }
    }

    const Graph &graph_; // 只读原图。
    NODE_TYPE k_{0}; // 固定 core 参数。
    std::vector<GlobalVertexState> states_; // 每个 global vertex 的预处理状态。
    std::vector<unsigned char> boundary_; // 全局 boundary 位图。
    std::vector<NODE_TYPE> thresholds_; // 全局异质阈值。
    std::vector<NODE_TYPE> degrees_; // 预处理期间的 residual 度数。
    std::vector<NODE_TYPE> peel_queue_; // forced-only peeling 队列。
    std::vector<std::size_t> local_index_; // 临时 global-to-local 映射。
    std::vector<NODE_TYPE> forced_blockers_; // mandatory global blockers。
    std::vector<Component> components_; // 构造完成的局部问题。
};

}  // namespace

PreprocessingResult preprocess(const Graph &graph,
                               const Subgraph &target,
                               const NODE_TYPE k) {
    return Preprocessor(graph, target, k).run();
}

}  // namespace anytime::internal
