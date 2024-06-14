#include "spanning_tree.hpp"
#include "ds/graph/Graph.hpp"
#include "ds/tree/UnionFind.hpp"

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
ds::graph::Graph::EdgeList minimum_spanning_tree(ds::graph::Graph const& graph) {
  typedef ds::graph::Graph::Edge Edge;
  typedef ds::graph::Graph::EdgeList EdgeList;
  typedef ds::graph::Graph::Weight Weight;
  typedef std::pair<Weight, Edge> WeightedEdge;

  EdgeList ret;
  ds::tree::UnionFind s;
  std::size_t n = graph.number_of_vertices();
  std::priority_queue<WeightedEdge, std::vector<WeightedEdge>, std::greater<WeightedEdge>> q;
  for (auto& e : graph.weighted_edges()) q.push({e.second, e.first});

  while (!q.empty()) {
    auto p = q.top();
    q.pop();
    if (s.Union(p.second.first, p.second.second)) {
      ret.push_back(p.second);
      if (ret.size() == n - 1) break;
    }
  }

  if (ret.size() != n - 1) throw std::invalid_argument("graph is disconnected");
  return ret;
}

/**
 * @brief Finds the cost of the minimum spanning tree.
 *
 * @param graph graph
 * @return ds::graph::Graph::Weight cost of the minimum spanning tree
 * @throw std::invalid_argument when graph is disconnected
 */
ds::graph::Graph::Weight minimum_spanning_tree_cost(ds::graph::Graph const& graph) {
  auto mst_edges = minimum_spanning_tree(graph);
  ds::graph::Graph::Weight total = 0;
  for (auto& e : mst_edges) total += graph.get_weight(e.first, e.second);
  return total;
}

}  // namespace graph
}  // namespace algorithm
