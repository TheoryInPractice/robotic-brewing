#pragma once

#include <atomic>

#include "ds/graph/Graph.hpp"

namespace algorithm {
namespace graph {

/**
 * @brief Returns the shortest path from source to destination.
 *
 * @param graph graph
 * @param source source
 * @param destination destination
 *
 * @return std::vector<ds::graph::Graph::Vertex> shortest path
 * @throw std::invalid_argument destination is unreachable from source
 */
std::vector<ds::graph::Graph::Vertex> shortest_path(ds::graph::Graph const& graph, ds::graph::Graph::Vertex source,
                                                    ds::graph::Graph::Vertex destination);

/**
 * @brief Returns the shortest path length from source to destination.
 *
 * @param graph graph
 * @param source source
 * @param destination destination
 * @return ds::graph::Graph::Weight shortest path length
 * @throw std::invalid_argument destination is unreachable from source
 */
ds::graph::Graph::Weight shortest_path_length(ds::graph::Graph const& graph, ds::graph::Graph::Vertex source,
                                              ds::graph::Graph::Vertex destination);

/**
 * @brief Returns the all weights of the single-source shortest paths.
 *
 * @param graph graph
 * @param source source
 * @return std::unordered_map<ds::graph::Graph::Vertex, ds::graph::Graph::Weight> single-source shortest paths
 */
std::unordered_map<ds::graph::Graph::Vertex, ds::graph::Graph::Weight> single_source_shortest_paths(  //
    ds::graph::Graph const& graph, ds::graph::Graph::Vertex source);

/**
 * @brief Returns the all-pairs shortest paths using Dijkstra's algorithm.
 *
 * @param graph graph
 * @param cancel_token cancelation token
 *
 * @return std::unordered_map<ds::graph::Graph::EdgeKey, ds::graph::Graph::Weight> distance map for every reachable vertex pair
 */
std::unordered_map<ds::graph::Graph::EdgeKey, ds::graph::Graph::Weight> all_pairs_dijkstra_path_length(
    ds::graph::Graph const& graph, std::vector<ds::graph::Graph::Vertex> const& target = {}, std::atomic_bool* cancel_token = nullptr);

}  // namespace graph
}  // namespace algorithm
