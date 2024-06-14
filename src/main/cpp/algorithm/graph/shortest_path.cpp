#include "shortest_path.hpp"

using namespace ds::graph;

namespace algorithm {
namespace graph {

namespace impl {

void single_source_dijkstra(                                 //
    Graph const& graph,                                      //
    Graph::Vertex source,                                    //
    std::vector<Graph::Vertex> const& destinations,          //
    std::unordered_map<Graph::Vertex, Graph::Vertex>* pred,  // pointer to predecessor map
    std::unordered_map<Graph::Vertex, Graph::Weight>& dist  // reference to final distance map (reachable vertices only)
) {
  using VertexData = std::pair<Graph::Weight, Graph::Vertex>;
  std::priority_queue<VertexData, std::vector<VertexData>, std::greater<VertexData>> q;
  std::unordered_set<Graph::Vertex> remain(destinations.begin(), destinations.end());

  q.push({dist[source] = 0, source});

  while (!q.empty()) {
    auto p = q.top();
    q.pop();
    int u = p.second;

    if (dist.at(u) < p.first) continue;  // outdated entry
    if (!destinations.empty()) {         // destinations are specified
      remain.erase(u);                   // finalize dist(source, u)
      if (remain.empty()) break;         // safe to stop here
    }

    for (auto v : graph.neighbors(u)) {
      auto cost = dist.at(u) + graph.get_weight(u, v);
      if (!util::contains(dist, v) || dist.at(v) > cost) {  // found a shorter path
        if (pred) (*pred)[v] = u;
        q.push({dist[v] = cost, v});
      }
    }
  }
}
}  // namespace impl

/**
 * @brief Returns the shortest path from source to destination.
 *
 * @param graph graph
 * @param source source
 * @param destination destination
 * @return std::vector<ds::graph::Graph::Vertex> shortest path
 * @throw std::invalid_argument destination is unreachable from source
 */
std::vector<ds::graph::Graph::Vertex> shortest_path(ds::graph::Graph const& graph, ds::graph::Graph::Vertex source,
                                                    ds::graph::Graph::Vertex destination) {
  // run Dijkstra
  std::unordered_map<Graph::Vertex, Graph::Vertex> pred;  // predecessor
  std::unordered_map<Graph::Vertex, Graph::Weight> dist;  // map of final distances to reachable vertices

  impl::single_source_dijkstra(graph, source, {destination}, &pred, dist);

  // backtracking
  if (!util::contains(dist, destination)) throw std::invalid_argument("destination is unreachable");

  std::vector<Graph::Vertex> ret;
  for (auto v = destination; v != source; v = pred.at(v)) ret.push_back(v);
  ret.push_back(source);
  std::reverse(ret.begin(), ret.end());
  return ret;
}

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
                                              ds::graph::Graph::Vertex destination) {
  std::unordered_map<Graph::Vertex, Graph::Weight> dist;  // map of final distances to reachable vertices
  impl::single_source_dijkstra(graph, source, {destination}, nullptr, dist);

  if (!util::contains(dist, destination)) throw std::invalid_argument("destination is unreachable");
  return dist.at(destination);
}

/**
 * @brief Returns the all weights of the single-source shortest paths.
 *
 * @param graph graph
 * @param source source
 * @return std::unordered_map<ds::graph::Graph::Vertex, ds::graph::Graph::Weight> single-source shortest paths
 */
std::unordered_map<ds::graph::Graph::Vertex, ds::graph::Graph::Weight> single_source_shortest_paths(  //
    ds::graph::Graph const& graph, ds::graph::Graph::Vertex source) {
  std::unordered_map<Graph::Vertex, Graph::Weight> dist;  // map of final distances to reachable vertices
  impl::single_source_dijkstra(graph, source, {}, nullptr, dist);
  return dist;
}

/**
 * @brief Returns the all-pairs shortest paths using Dijkstra's algorithm.
 *
 * @param graph graph
 * @param target target vertices
 * @param cancel_token cancelation token
 *
 * @return std::unordered_map<ds::graph::Graph::EdgeKey, ds::graph::Graph::Weight> distance map for every reachable vertex pair
 */
std::unordered_map<ds::graph::Graph::EdgeKey, ds::graph::Graph::Weight> all_pairs_dijkstra_path_length(  //
    ds::graph::Graph const& graph, std::vector<ds::graph::Graph::Vertex> const& target, std::atomic_bool* cancel_token  //
) {
  std::unordered_map<ds::graph::Graph::EdgeKey, ds::graph::Graph::Weight> ret;

  for (auto v : target.empty() ? graph.vertices() : target) {
    if (cancel_token && cancel_token->load()) throw std::runtime_error("canceled: all_pairs_dijkstra_path_length()");

    // run Dijkstra
    std::unordered_map<Graph::Vertex, Graph::Weight> dist;  // map of final distances to reachable vertices

    impl::single_source_dijkstra(graph, v, target, nullptr, dist);

    // collect results
    if (target.empty()) {
      for (auto& p : dist) ret.emplace(graph.to_edgekey(v, p.first), p.second);
    } else {
      for (auto u : target) {
        if (util::contains(dist, u)) ret.emplace(graph.to_edgekey(v, u), dist.at(u));
      }
    }
  }

  return ret;
}

}  // namespace graph
}  // namespace algorithm
