#pragma once

#include "ds/graph/Graph.hpp"

namespace algorithm {
namespace graph {

/**
 * @brief Returns all vertices reachable from the source veretx.
 *
 * @param graph graph
 * @param source source vertex
 * @return std::vector<ds::graph::Graph::Vertex> connected component
 */
std::vector<ds::graph::Graph::Vertex> connected_component(ds::graph::Graph const& graph, ds::graph::Graph::Vertex source);

/**
 * @brief Finds all bridges in the graph.
 *
 * @param graph graph
 * @return std::vector<ds::graph::Graph::Edge> list of bridges
 */
std::vector<ds::graph::Graph::Edge> find_bridges(ds::graph::Graph const& graph);

/**
 * @brief Finds any u-v path in the graph.
 *
 * @param graph graph
 * @param u vertex u
 * @param v vertex v
 * @return std::vector<ds::graph::Graph::Vertex> a u-v path if one exists; empty vector otherwise
 *
 * @throw std::invalid_argument there are no vertices u, v in the graph
 */
std::vector<ds::graph::Graph::Vertex> find_path(  //
    ds::graph::Graph const& graph, ds::graph::Graph::Vertex u, ds::graph::Graph::Vertex v);

/**
 * @brief Finds bridges in a u-v path in G-uv.
 *
 * @param graph graph
 * @param u vertex u
 * @param v vertex v
 * @return std::vector<ds::graph::Graph::Edge> list of bridges in a u-v path withtout using edge uv
 */
std::vector<ds::graph::Graph::Edge> find_bridges_in_between(  //
    ds::graph::Graph const& graph, ds::graph::Graph::Vertex u, ds::graph::Graph::Vertex v);
}  // namespace graph
}  // namespace algorithm
