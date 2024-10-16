#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
   private:
       unordered_map<VertexT, map<VertexT, WeightT>> adj_list;

   public:
    /// Default constructor
    graph() {

    }

    /// @brief Add the vertex `v` to the graph, must run in at most O(log |V|).
    /// @param v
    /// @return true if successfully added; false if it existed already
    bool addVertex(VertexT v) {
        if (adj_list.find(v) == adj_list.end()) {
            adj_list[v] = {};
            return true;
        }
        return false;
    }

    /// @brief Add or overwrite directed edge in the graph, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight edge weight / label
    /// @return true if successfully added or overwritten;
    ///         false if either vertices isn't in graph
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
        if (adj_list.find(from) == adj_list.end() || adj_list.find(to) == adj_list.end()) {
            return false;
        }
        adj_list[from][to] = weight;
        return true;
    }

    /// @brief Maybe get the weight associated with a given edge, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight output parameter
    /// @return true if the edge exists, and `weight` is set;
    ///         false if the edge does not exist
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
        auto iter = adj_list.find(from); //iterator
        if (iter != adj_list.end()) {
            if (iter->second.find(to) != iter->second.end()) {
                weight = iter->second.find(to)->second;
                return true;
            }
        }
        return false;
    }

    /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
    /// @param v
    /// @return vertices that v has an edge to
    set<VertexT> neighbors(VertexT v) const {
        set<VertexT> S;
        auto iter = adj_list.find(v); //iterator
        if (iter != adj_list.end()) {
            for (const auto& ver : iter->second) {
                S.insert(ver.first);
            }
        }
        return S;
    }

    /// @brief Return a vector containing all vertices in the graph
    vector<VertexT> getVertices() const {
        vector<VertexT> v; //vector of vertexes
        for (const auto& ver : adj_list) {
            v.push_back(ver.first);
        }
        return v;
    }

    /// @brief Get the number of vertices in the graph. Runs in O(1).
    size_t NumVertices() const {
        return adj_list.size();
    }

    /// @brief Get the number of directed edges in the graph. Runs in at most O(|V|).
    size_t NumEdges() const {
        size_t count = 0;
        for (const auto& ver : adj_list) {
            count += ver.second.size();
        }
        return count;
    }
};
