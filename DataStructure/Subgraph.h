#ifndef ABCORE_SUBGRAPH_H
#define ABCORE_SUBGRAPH_H

#include "../constant.h"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

class Graph;

// A vertex set stored in canonical sorted, duplicate-free form.
class Subgraph {
public:
    Subgraph() = default;
    explicit Subgraph(std::vector<NODE_TYPE> vertices);

    // Parse one subgraph from comma/whitespace-separated vertex IDs. Lines
    // may contain comments beginning with '#'.
    static Subgraph fromText(std::string_view text);

    std::size_t size() const noexcept;
    bool empty() const noexcept;
    bool contains(NODE_TYPE vertex) const;

    const std::vector<NODE_TYPE> &vertices() const noexcept;

    // Graph-relative subgraph operations. Invalid vertex IDs are rejected.
    void validate(const Graph &graph) const;
    std::vector<Subgraph> connectedComponents(const Graph &graph) const;

private:
    struct SortedUniqueTag {};

    Subgraph(std::vector<NODE_TYPE> vertices, SortedUniqueTag);

    std::vector<NODE_TYPE> vertices_;
};

#endif  // ABCORE_SUBGRAPH_H
