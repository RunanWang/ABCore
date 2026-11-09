//
// Created by 王润安 on 2023/11/16.
//

#ifndef ABCORE_GRAPH_H
#define ABCORE_GRAPH_H

#include "../constant.h"
#include <string>
#include "vector"

using namespace std;

struct edge {
    NODE_TYPE s{0};
    NODE_TYPE t{0};

    edge() = default;

    edge(NODE_TYPE s_, NODE_TYPE t_) : s(s_), t(t_) {};

    bool operator<(const edge &e) const {
        return s < e.s || (s == e.s && t < e.t);
    }
};

class Graph {
public:
    Graph() = default;

    ~Graph();

    NODE_TYPE *edges{nullptr};
    EDGE_TYPE *index{nullptr};

    NODE_TYPE nodesNum{0};
    EDGE_TYPE edgesNum{0};
    NODE_TYPE maxDegree{0};

    void BuildFromEdgeLst(vector<edge> &edge_buf, NODE_TYPE num_of_vtx, EDGE_TYPE num_of_edge);

    // Read an undirected two-column edge list and store both directions in CSR.
    void readUndirectedGraphFromFile(const std::string &file_path);

    NODE_TYPE getDegreeOfVertex(NODE_TYPE v) const;

    EDGE_TYPE start(NODE_TYPE v) const;

    EDGE_TYPE end(NODE_TYPE v) const;

};

#endif //ABCORE_GRAPH_H
