//
// Created by 王润安 on 2023/11/16.
//

#include "Graph.h"
#include "algorithm"
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

Graph::~Graph() {
    delete[] index;
    delete[] edges;
}

// Using CSR to store graph.
void Graph::BuildFromEdgeLst(vector<edge> &edge_buf, NODE_TYPE num_of_vtx, EDGE_TYPE num_of_edge) {
    EDGE_TYPE i, j, pNodeFirst;
    NODE_TYPE v;

    if (num_of_edge != edge_buf.size()) {
        throw std::invalid_argument("num_of_edge does not match the edge-list size");
    }
    for (const auto &input_edge : edge_buf) {
        if (input_edge.s >= num_of_vtx || input_edge.t >= num_of_vtx) {
            throw std::invalid_argument("edge endpoint is outside the vertex range");
        }
    }

    delete[] index;
    delete[] edges;
    index = nullptr;
    edges = nullptr;
    edgesNum = 0;
    maxDegree = 0;

    nodesNum = num_of_vtx;
    index = new EDGE_TYPE[nodesNum + 1];
    edges = new NODE_TYPE[num_of_edge + nodesNum + 1];
    // edges[index[v]:index[v+1]] = [deg_of_v(index ptr to here), nbr1, nbr2, ...]

    std::sort(edge_buf.begin(), edge_buf.end());

    i = 0; // index of edge_buf
    j = 0; // index of edges in CSR
    for (v = 0; v < nodesNum; v++) {
        index[v] = j;
        if (i >= num_of_edge || edge_buf[i].s > v) {
            // 这个点没有邻居，存一个度数=0
            edges[j++] = 0;
        } else {
            pNodeFirst = j++; // index to store vtx degree
            bool has_last_neighbor = false;
            NODE_TYPE last_neighbor = 0;
            while (i < num_of_edge && edge_buf[i].s == v) {
                const NODE_TYPE neighbor = edge_buf[i].t;
                if (neighbor != v && (!has_last_neighbor || neighbor != last_neighbor)) {
                    edges[j++] = neighbor;
                    last_neighbor = neighbor;
                    has_last_neighbor = true;
                }
                i++;
            }
            edges[pNodeFirst] = j - pNodeFirst - 1;
            edgesNum += edges[pNodeFirst];
            if (edges[pNodeFirst] > maxDegree) maxDegree = edges[pNodeFirst];
        }
    }
    // 最后edges和index分别+1，方便访问时避免数组越界
    edges[j] = 0;
    index[nodesNum] = j;
}

void Graph::readUndirectedGraphFromFile(const std::string &file_path) {
    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open graph file: " + file_path);
    }

    std::vector<edge> edge_buffer;
    NODE_TYPE max_vertex = 0;
    bool has_vertex = false;
    std::string line;
    std::size_t line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }

        std::istringstream line_stream(line);
        unsigned long long source;
        unsigned long long target;
        if (!(line_stream >> source >> target)) {
            throw std::runtime_error("invalid edge at line " + std::to_string(line_number) +
                                     " in " + file_path);
        }
        std::string trailing_token;
        if ((line_stream >> trailing_token) && trailing_token.front() != '#') {
            throw std::runtime_error("unexpected data at line " + std::to_string(line_number) +
                                     " in " + file_path);
        }
        if (source > std::numeric_limits<NODE_TYPE>::max() ||
            target > std::numeric_limits<NODE_TYPE>::max()) {
            throw std::runtime_error("vertex id is out of range at line " +
                                     std::to_string(line_number) + " in " + file_path);
        }

        const auto s = static_cast<NODE_TYPE>(source);
        const auto t = static_cast<NODE_TYPE>(target);
        if (s == t) {
            continue;
        }
        edge_buffer.emplace_back(s, t);
        edge_buffer.emplace_back(t, s);
        max_vertex = std::max(max_vertex, std::max(s, t));
        has_vertex = true;
    }

    if (edge_buffer.size() > std::numeric_limits<EDGE_TYPE>::max()) {
        throw std::runtime_error("too many edges in graph file: " + file_path);
    }

    if (has_vertex && max_vertex == std::numeric_limits<NODE_TYPE>::max()) {
        throw std::runtime_error("vertex count exceeds NODE_TYPE in graph file: " + file_path);
    }
    const NODE_TYPE vertex_count = has_vertex ? max_vertex + 1 : 0;
    BuildFromEdgeLst(edge_buffer, vertex_count, static_cast<EDGE_TYPE>(edge_buffer.size()));
}

NODE_TYPE Graph::getDegreeOfVertex(NODE_TYPE v) const {
    return edges[index[v]];
}

EDGE_TYPE Graph::start(NODE_TYPE v) const {
    return index[v] + 1;
}

EDGE_TYPE Graph::end(NODE_TYPE v) const {
    return index[v + 1];
}

