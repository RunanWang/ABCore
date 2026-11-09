#include "Subgraph.h"

#include "Graph.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

NODE_TYPE parseVertex(const std::string &token) {
    if (token.empty() || token.front() == '-') {
        throw std::invalid_argument("invalid subgraph vertex: " + token);
    }
    std::size_t parsed_length = 0;
    unsigned long long value = 0;
    try {
        value = std::stoull(token, &parsed_length);
    } catch (const std::exception &) {
        throw std::invalid_argument("invalid subgraph vertex: " + token);
    }
    if (parsed_length != token.size() ||
        value > std::numeric_limits<NODE_TYPE>::max()) {
        throw std::invalid_argument("invalid subgraph vertex: " + token);
    }
    return static_cast<NODE_TYPE>(value);
}

}  // namespace

Subgraph::Subgraph(std::vector<NODE_TYPE> vertices)
    : vertices_(std::move(vertices)) {
    std::sort(vertices_.begin(), vertices_.end());
    vertices_.erase(std::unique(vertices_.begin(), vertices_.end()), vertices_.end());
}

Subgraph::Subgraph(std::vector<NODE_TYPE> vertices, SortedUniqueTag)
    : vertices_(std::move(vertices)) {}

Subgraph Subgraph::fromText(const std::string_view text) {
    std::istringstream lines{std::string(text)};
    std::vector<NODE_TYPE> vertices;
    std::string line;
    while (std::getline(lines, line)) {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream tokens(line);
        std::string token;
        while (tokens >> token) {
            vertices.push_back(parseVertex(token));
        }
    }
    if (vertices.empty()) {
        throw std::invalid_argument("subgraph vertex set is empty");
    }
    return Subgraph(std::move(vertices));
}

std::size_t Subgraph::size() const noexcept {
    return vertices_.size();
}

bool Subgraph::empty() const noexcept {
    return vertices_.empty();
}

bool Subgraph::contains(NODE_TYPE vertex) const {
    return std::binary_search(vertices_.begin(), vertices_.end(), vertex);
}

const std::vector<NODE_TYPE> &Subgraph::vertices() const noexcept {
    return vertices_;
}

void Subgraph::validate(const Graph &graph) const {
    for (const NODE_TYPE vertex : vertices_) {
        if (vertex >= graph.nodesNum) {
            throw std::invalid_argument(
                "subgraph contains vertex " + std::to_string(vertex) +
                " outside the graph");
        }
    }
}

std::vector<Subgraph> Subgraph::connectedComponents(
    const Graph &graph) const {
    validate(graph);
    std::vector<Subgraph> components;
    if (empty()) {
        return components;
    }

    std::vector<unsigned char> in_subgraph(graph.nodesNum, 0);
    std::vector<unsigned char> visited(graph.nodesNum, 0);
    for (const NODE_TYPE vertex : vertices_) {
        in_subgraph[vertex] = 1;
    }

    for (const NODE_TYPE start : vertices_) {
        if (visited[start]) {
            continue;
        }
        std::vector<NODE_TYPE> component;
        std::queue<NODE_TYPE> pending;
        visited[start] = 1;
        pending.push(start);
        while (!pending.empty()) {
            const NODE_TYPE vertex = pending.front();
            pending.pop();
            component.push_back(vertex);
            for (EDGE_TYPE position = graph.start(vertex);
                 position < graph.end(vertex); ++position) {
                const NODE_TYPE neighbor = graph.edges[position];
                if (neighbor >= graph.nodesNum) {
                    throw std::runtime_error(
                        "graph contains a neighbor outside its vertex range");
                }
                if (in_subgraph[neighbor] && !visited[neighbor]) {
                    visited[neighbor] = 1;
                    pending.push(neighbor);
                }
            }
        }
        components.emplace_back(std::move(component));
    }
    return components;
}

