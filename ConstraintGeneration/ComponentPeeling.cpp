#include "ConstraintGeneration/ComponentPeeling.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace constraint_generation {
namespace {

// 检查局部问题的 CSR、顶点属性与 baseline 度数是否一致。
// 输入：待绑定问题。输出：合法时返回；非法时抛出 invalid_argument。
void validateProblem(const ComponentProblem &problem) {
    const std::size_t size = problem.vertices.size(); // 所有维度的基准大小。
    if (size > std::numeric_limits<NODE_TYPE>::max() ||
        problem.offsets.size() != size + 1 ||
        problem.thresholds.size() != size ||
        problem.boundary.size() != size ||
        problem.baseline_degrees.size() != size ||
        problem.offsets.empty() || problem.offsets.front() != 0 ||
        problem.offsets.back() != problem.neighbors.size()) {
        throw std::invalid_argument(
            "exact component has inconsistent dimensions");
    }
    for (NODE_TYPE vertex = 0; vertex < problem.size(); ++vertex) {
        if (problem.offsets[vertex] > problem.offsets[vertex + 1] ||
            problem.baseline_degrees[vertex] !=
                problem.offsets[vertex + 1] - problem.offsets[vertex]) {
            throw std::invalid_argument(
                "exact component has inconsistent CSR offsets or degrees");
        }
    }
    for (const NODE_TYPE neighbor : problem.neighbors) {
        if (neighbor >= problem.size()) {
            throw std::invalid_argument(
                "exact component has an out-of-range CSR neighbor");
        }
    }
}

}  // namespace

ComponentPeeling::ComponentPeeling(const ComponentProblem &problem)
    : problem_(problem) {
    validateProblem(problem_);
}

Residual ComponentPeeling::run(
    const std::vector<NODE_TYPE> &blockers) const {
    for (const NODE_TYPE blocker : blockers) {
        if (blocker >= problem_.size()) {
            throw std::invalid_argument(
                "exact blocker local id is out of range");
        }
    }

    Residual result; // 本次 blocker 集对应的 fixed-point 快照。
    result.active.assign(problem_.size(), 1);
    result.degrees.assign(problem_.size(), 0);
    std::vector<unsigned char> queued(problem_.size(), 0); // 是否已进入队列。
    std::vector<NODE_TYPE> queue;                          // 失稳顶点队列。

    // 1. 同时应用全部 blocker，再准确计算剩余诱导度。
    for (const NODE_TYPE blocker : blockers) {
        result.active[blocker] = 0;
    }
    for (NODE_TYPE vertex = 0; vertex < problem_.size(); ++vertex) {
        if (!result.active[vertex]) {
            continue;
        }
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            result.degrees[vertex] +=
                result.active[problem_.neighbors[position]];
        }
        if (result.degrees[vertex] < problem_.thresholds[vertex]) {
            queued[vertex] = 1;
            queue.push_back(vertex);
        }
    }

    // 2. 传播失稳删除，直到达到唯一 fixed point。
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const NODE_TYPE vertex = queue[head]; // 本轮正式 peeling 的点。
        if (!result.active[vertex]) {
            continue;
        }
        result.active[vertex] = 0;
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem_.neighbors[position]; // 度数被减一的 active 邻居。
            if (!result.active[neighbor]) {
                continue;
            }
            --result.degrees[neighbor];
            if (!queued[neighbor] &&
                result.degrees[neighbor] < problem_.thresholds[neighbor]) {
                queued[neighbor] = 1;
                queue.push_back(neighbor);
            }
        }
    }

    // 3. 汇总 residual 规模，feasible 只由 surviving boundary 决定。
    for (NODE_TYPE vertex = 0; vertex < problem_.size(); ++vertex) {
        if (result.active[vertex]) {
            ++result.active_vertices;
            result.active_boundaries += problem_.boundary[vertex] != 0;
        }
    }
    return result;
}

std::vector<Witness> ComponentPeeling::targetComponents(
    const Residual &residual,
    const std::size_t limit) const {
    validateResidual(residual);
    std::vector<Witness> components; // 最多 limit 个 target-related 分量。
    std::vector<unsigned char> visited(problem_.size(), 0); // BFS 标记。
    std::vector<NODE_TYPE> queue; // 当前分量复用的 BFS 队列。

    // 仅从 boundary 出发；不含 boundary 的 active 分量与目标隔离无关。
    for (NODE_TYPE start = 0;
         start < problem_.size() && components.size() < limit; ++start) {
        if (!residual.active[start] || !problem_.boundary[start] ||
            visited[start]) {
            continue;
        }
        queue.clear();
        queue.push_back(start);
        visited[start] = 1;
        Witness component; // 当前 boundary 所在 active 分量。
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const NODE_TYPE vertex = queue[head]; // 当前扩展的 local 点。
            component.push_back(vertex);
            for (std::size_t position = problem_.offsets[vertex];
                 position < problem_.offsets[vertex + 1]; ++position) {
                const NODE_TYPE neighbor =
                    problem_.neighbors[position]; // 待访问的 active 邻居。
                if (residual.active[neighbor] && !visited[neighbor]) {
                    visited[neighbor] = 1;
                    queue.push_back(neighbor);
                }
            }
        }
        std::sort(component.begin(), component.end());
        components.push_back(std::move(component));
    }
    return components;
}

void ComponentPeeling::validateResidual(const Residual &residual) const {
    if (residual.active.size() != problem_.size() ||
        residual.degrees.size() != problem_.size()) {
        throw std::invalid_argument("exact residual has invalid dimensions");
    }
    std::size_t active_vertices = 0; // 重新统计的 active 点数。
    std::size_t active_boundaries = 0; // 重新统计的 active boundary 数。
    for (NODE_TYPE vertex = 0; vertex < problem_.size(); ++vertex) {
        if (!residual.active[vertex]) {
            continue;
        }
        NODE_TYPE actual_degree = 0; // 从 active 位图重算的诱导度。
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            actual_degree += residual.active[problem_.neighbors[position]];
        }
        if (actual_degree != residual.degrees[vertex] ||
            actual_degree < problem_.thresholds[vertex]) {
            throw std::invalid_argument(
                "exact residual is not a valid fixed point");
        }
        ++active_vertices;
        active_boundaries += problem_.boundary[vertex] != 0;
    }
    if (active_vertices != residual.active_vertices ||
        active_boundaries != residual.active_boundaries) {
        throw std::invalid_argument("exact residual has invalid counters");
    }
}

void ComponentPeeling::validateWitness(const Witness &witness) const {
    if (witness.empty() ||
        !std::is_sorted(witness.begin(), witness.end()) ||
        std::adjacent_find(witness.begin(), witness.end()) != witness.end()) {
        throw std::invalid_argument(
            "exact witness must be non-empty, sorted and duplicate-free");
    }

    std::vector<unsigned char> in_witness(problem_.size(), 0); // 集合位图。
    bool contains_boundary = false; // 是否已观察到 target boundary。
    for (const NODE_TYPE vertex : witness) {
        if (vertex >= problem_.size()) {
            throw std::invalid_argument(
                "exact witness contains an out-of-range vertex");
        }
        in_witness[vertex] = 1;
        contains_boundary = contains_boundary || problem_.boundary[vertex];
    }
    if (!contains_boundary) {
        throw std::invalid_argument("exact witness contains no boundary");
    }

    // 1. 检查 witness 诱导子图中的异质阈值稳定性。
    for (const NODE_TYPE vertex : witness) {
        NODE_TYPE induced_degree = 0; // 当前点在 witness 内的度数。
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            induced_degree += in_witness[problem_.neighbors[position]];
        }
        if (induced_degree < problem_.thresholds[vertex]) {
            throw std::invalid_argument(
                "exact witness violates a local threshold");
        }
    }

    // 2. 从首点 BFS，验证全部 witness 点属于同一连通分量。
    std::vector<unsigned char> reached(problem_.size(), 0); // BFS 到达标记。
    std::vector<NODE_TYPE> queue{witness.front()}; // 连通性检查队列。
    reached[witness.front()] = 1;
    std::size_t reached_count = 0; // 从首点实际到达的 witness 点数。
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const NODE_TYPE vertex = queue[head]; // 当前扩展的 witness 点。
        ++reached_count;
        for (std::size_t position = problem_.offsets[vertex];
             position < problem_.offsets[vertex + 1]; ++position) {
            const NODE_TYPE neighbor =
                problem_.neighbors[position]; // 待访问的 witness 邻居。
            if (in_witness[neighbor] && !reached[neighbor]) {
                reached[neighbor] = 1;
                queue.push_back(neighbor);
            }
        }
    }
    if (reached_count != witness.size()) {
        throw std::invalid_argument("exact witness is disconnected");
    }
}

}  // namespace constraint_generation
