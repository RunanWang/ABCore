#include "ConstraintGeneration/Preprocessor.h"

#include "DataStructure/Subgraph.h"
#include "DataStructure/Graph.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace constraint_generation {
namespace {

constexpr std::size_t kNoLocalIndex =
    std::numeric_limits<std::size_t>::max(); // global 点不在当前 component。

enum class GlobalVertexState : unsigned char {
    Subgraph, // 固定属于目标子图，不进入外部问题。
    Residual,  // 当前仍可能属于外部 witness。
    Unstable,  // 已入 peeling 队列但尚未处理。
    Forced,    // threshold 为零的必要 blocker。
    Peeled,    // forced-only peeling 已删除。
};

// 检查并返回一个 CSR 邻居，避免损坏的输入越界传播到算法内部。
// 输入：图与 CSR 位置。输出：合法的 global vertex ID。
NODE_TYPE checkedNeighbor(const Graph &graph, const EDGE_TYPE position) {
    const NODE_TYPE neighbor = graph.edges[position]; // 待校验的 global ID。
    if (neighbor >= graph.nodesNum) {
        throw std::runtime_error(
            "exact preprocessor encountered an invalid CSR neighbor");
    }
    return neighbor;
}

// 把一个 global residual 分量复制成自包含的 local-ID CSR 问题。
// 输入：分量顶点及其全局属性。输出：编号确定、顶点有序的局部问题。
ComponentProblem buildComponent(
    const Graph &graph,
    std::vector<NODE_TYPE> vertices,
    const std::vector<NODE_TYPE> &thresholds,
    const std::vector<unsigned char> &boundary,
    std::vector<std::size_t> &local_index,
    const NODE_TYPE component_index) {
    std::sort(vertices.begin(), vertices.end());

    ComponentProblem component; // 正在构造的 local-ID 问题。
    component.index = component_index;
    component.vertices = std::move(vertices);
    component.offsets.reserve(component.vertices.size() + 1);
    component.thresholds.reserve(component.vertices.size());
    component.boundary.reserve(component.vertices.size());
    component.baseline_degrees.reserve(component.vertices.size());

    // 1. 临时建立 global ID 到 local ID 的映射。
    for (std::size_t local = 0; local < component.vertices.size(); ++local) {
        local_index[component.vertices[local]] = local;
    }

    // 2. 仅复制分量内部边，形成局部 CSR 和对应顶点属性。
    component.offsets.push_back(0);
    for (const NODE_TYPE vertex : component.vertices) {
        component.thresholds.push_back(thresholds[vertex]);
        component.boundary.push_back(boundary[vertex]);
        for (EDGE_TYPE position = graph.start(vertex);
             position < graph.end(vertex); ++position) {
            const std::size_t neighbor_local =
                local_index[checkedNeighbor(graph, position)]; // 邻居的 local ID。
            if (neighbor_local != kNoLocalIndex) {
                component.neighbors.push_back(
                    static_cast<NODE_TYPE>(neighbor_local));
            }
        }
        component.offsets.push_back(component.neighbors.size());
        component.baseline_degrees.push_back(
            static_cast<NODE_TYPE>(component.offsets.back() -
                                   component.offsets[component.offsets.size() -
                                                     2]));
    }

    // 3. 清理临时映射，避免影响下一个分量。
    for (const NODE_TYPE vertex : component.vertices) {
        local_index[vertex] = kNoLocalIndex;
    }
    return component;
}

}  // namespace

Preprocessor::Preprocessor(const Graph &graph,
                           const Subgraph &target_subgraph,
                           const NODE_TYPE k)
    : graph_(graph), target_subgraph_(target_subgraph), k_(k) {}

PreprocessingResult Preprocessor::run() const {
    if (k_ == 0) {
        throw std::invalid_argument("exact solver requires k to be positive");
    }
    if (target_subgraph_.empty()) {
        throw std::invalid_argument(
            "exact solver requires a non-empty target subgraph");
    }
    target_subgraph_.validate(graph_);
    if (target_subgraph_.connectedComponents(graph_).size() != 1) {
        throw std::invalid_argument(
            "exact solver requires a connected target subgraph");
    }

    std::vector<GlobalVertexState> states(
        graph_.nodesNum, GlobalVertexState::Residual); // 全局顶点生命周期。
    std::vector<unsigned char> boundary(graph_.nodesNum, 0); // 是否邻接 C。
    std::vector<NODE_TYPE> thresholds(graph_.nodesNum, 0);   // delta(v)。
    std::vector<NODE_TYPE> degrees(graph_.nodesNum, 0);      // 当前外部度。
    PreprocessingResult result; // 本次运行独占的最终输出。

    // 1. 固定子图顶点，并计算每个外部点的 boundary 与异质阈值。
    for (const NODE_TYPE vertex : target_subgraph_.vertices()) {
        states[vertex] = GlobalVertexState::Subgraph;
    }
    for (NODE_TYPE vertex = 0; vertex < graph_.nodesNum; ++vertex) {
        if (states[vertex] == GlobalVertexState::Subgraph) {
            continue;
        }
        NODE_TYPE subgraph_degree = 0; // v 在目标子图中的邻居数。
        for (EDGE_TYPE position = graph_.start(vertex);
             position < graph_.end(vertex); ++position) {
            subgraph_degree +=
                states[checkedNeighbor(graph_, position)] ==
                GlobalVertexState::Subgraph;
        }
        boundary[vertex] = subgraph_degree != 0;
        thresholds[vertex] =
            subgraph_degree < k_ ? k_ - subgraph_degree : 0;
        if (boundary[vertex] && thresholds[vertex] == 0) {
            states[vertex] = GlobalVertexState::Forced;
            result.forced_blockers.push_back(vertex);
        }
    }

    // 2. 在排除 C 与 forced blockers 后的外部诱导图上初始化度数。
    for (NODE_TYPE vertex = 0; vertex < graph_.nodesNum; ++vertex) {
        if (states[vertex] != GlobalVertexState::Residual) {
            continue;
        }
        for (EDGE_TYPE position = graph_.start(vertex);
             position < graph_.end(vertex); ++position) {
            degrees[vertex] +=
                states[checkedNeighbor(graph_, position)] ==
                GlobalVertexState::Residual;
        }
    }

    // 3. 执行 forced-only peeling，删除不可能属于任何 witness 的顶点。
    std::vector<NODE_TYPE> peel_queue; // 只入队一次的失稳顶点队列。
    // 输入：待检查 global ID。输出：失稳点被标记并加入 peel_queue。
    const auto enqueue_if_unstable = [&](const NODE_TYPE vertex) {
        if (states[vertex] == GlobalVertexState::Residual &&
            degrees[vertex] < thresholds[vertex]) {
            states[vertex] = GlobalVertexState::Unstable;
            peel_queue.push_back(vertex);
        }
    };
    for (NODE_TYPE vertex = 0; vertex < graph_.nodesNum; ++vertex) {
        enqueue_if_unstable(vertex);
    }
    for (std::size_t head = 0; head < peel_queue.size(); ++head) {
        const NODE_TYPE vertex = peel_queue[head]; // 本轮正式移除的 global 点。
        states[vertex] = GlobalVertexState::Peeled;
        for (EDGE_TYPE position = graph_.start(vertex);
             position < graph_.end(vertex); ++position) {
            const NODE_TYPE neighbor =
                checkedNeighbor(graph_, position); // 受级联影响的邻居。
            if (states[neighbor] != GlobalVertexState::Residual) {
                continue;
            }
            --degrees[neighbor];
            enqueue_if_unstable(neighbor);
        }
    }

    // 4. 只从 surviving boundary 出发提取 target-related residual 分量。
    std::vector<unsigned char> visited(graph_.nodesNum, 0); // 分量 BFS 标记。
    std::vector<NODE_TYPE> queue; // 当前分量复用的 BFS 队列。
    std::vector<std::size_t> local_index(
        graph_.nodesNum, kNoLocalIndex); // 当前分量的 global-to-local 映射。
    for (NODE_TYPE start = 0; start < graph_.nodesNum; ++start) {
        if (states[start] != GlobalVertexState::Residual || !boundary[start] ||
            visited[start]) {
            continue;
        }
        queue.clear();
        queue.push_back(start);
        visited[start] = 1;
        std::vector<NODE_TYPE> vertices; // 当前 global residual 分量。
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const NODE_TYPE vertex = queue[head]; // 当前扩展的 global 点。
            vertices.push_back(vertex);
            for (EDGE_TYPE position = graph_.start(vertex);
                 position < graph_.end(vertex); ++position) {
                const NODE_TYPE neighbor =
                    checkedNeighbor(graph_, position); // 待访问的 global 邻居。
                if (states[neighbor] == GlobalVertexState::Residual &&
                    !visited[neighbor]) {
                    visited[neighbor] = 1;
                    queue.push_back(neighbor);
                }
            }
        }
        result.components.push_back(buildComponent(
            graph_, std::move(vertices), thresholds, boundary, local_index,
            static_cast<NODE_TYPE>(result.components.size() + 1)));
    }

    return result;
}

}  // namespace constraint_generation
