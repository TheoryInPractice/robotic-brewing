#pragma once

#include "ds/graph/Graph.hpp"

namespace algorithm {
namespace graph {

/**
 * @brief Returns a Eulerian path (or cycle) from source using all given multiedges.
 *
 * @param multiedges list of edges in an undirected multigraph
 * @param source source vertex
 * @return std::vector<ds::graph::Graph::Vertex> Eulerian path
 */
std::vector<ds::graph::Graph::Vertex> eulerian_path(std::vector<ds::graph::Graph::Edge> const& multiedges,
                                                    ds::graph::Graph::Vertex source);

}  // namespace graph
}  // namespace algorithm
