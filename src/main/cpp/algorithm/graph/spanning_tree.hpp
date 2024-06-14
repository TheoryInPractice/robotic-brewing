#pragma once

#include "ds/graph/Graph.hpp"

namespace algorithm {
namespace graph {

/**
 * @brief Finds the the minimum spanning tree.
 *
 * @pre graph must be connected
 * @param graph graph
 * @return ds::graph::Graph::EdgeList edges used for a minimum spanning tree
 * @throw std::invalid_argument when graph is disconnected
 */
ds::graph::Graph::EdgeList minimum_spanning_tree(ds::graph::Graph const& graph);

/**
 * @brief Finds the cost of the minimum spanning tree.
 *
 * @pre graph must be connected
 * @param graph graph
 * @return ds::graph::Graph::Weight cost of the minimum spanning tree
 * @throw std::invalid_argument when graph is disconnected
 */
ds::graph::Graph::Weight minimum_spanning_tree_cost(ds::graph::Graph const& graph);

}  // namespace graph
}  // namespace algorithm
