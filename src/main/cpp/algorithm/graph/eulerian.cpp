#include "eulerian.hpp"

namespace algorithm {
namespace graph {

/**
 * @brief Returns a Eulerian path (or cycle) from source using all given multiedges.
 *
 * @param multiedges list of edges in an undirected multigraph
 * @param source source vertex
 * @return std::vector<ds::graph::Graph::Vertex> Eulerian path
 *
 * Undefined behavior if the input is not Eulerian.
 */
std::vector<ds::graph::Graph::Vertex> eulerian_path(std::vector<ds::graph::Graph::Edge> const& multiedges,
                                                    ds::graph::Graph::Vertex source) {
  using Vertex = ds::graph::Graph::Vertex;

  // Hierholzer’s Algorithm for undirected multigraphs.
  std::vector<Vertex> ret;

  // Create adjacency sets.
  std::unordered_map<Vertex, std::multiset<Vertex>> adj;
  for (auto& e : multiedges) {
    adj[e.first].insert(e.second);
    adj[e.second].insert(e.first);
  }

  // Find an Eulerian path.
  std::vector<Vertex> cur = {source};

  while (!cur.empty()) {
    auto v = cur.back();

    if (!adj[v].empty()) {
      // Take an arbitrary neighbor.
      auto u = *adj[v].begin();
      cur.push_back(u);

      // Remove this edge from the graph.
      adj[v].extract(u);
      adj[u].extract(v);
    } else {
      ret.push_back(cur.back());
      cur.pop_back();
    }
  }
  std::reverse(ret.begin(), ret.end());
  return ret;
}
}  // namespace graph
}  // namespace algorithm
