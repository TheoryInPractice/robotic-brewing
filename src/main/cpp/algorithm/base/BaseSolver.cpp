#include <queue>

#include "BaseSolver.hpp"

#include "algorithm/graph/spanning_tree.hpp"
#include "util/util.hpp"

namespace algorithm {
namespace base {
namespace impl {
static std::vector<BaseSolver::Vertex> find_walk_on_tree(                                                       //
    std::vector<BaseSolver::Edge> const &tree_edges, BaseSolver::Vertex source, BaseSolver::Vertex destination  //
) {
  using Vertex = BaseSolver::Vertex;

  // construct a rooted tree
  std::unordered_map<Vertex, std::vector<Vertex>> adj;
  std::unordered_map<Vertex, Vertex> parent;
  std::unordered_map<Vertex, Vertex> last_child;

  // create adjacency list
  for (auto &e : tree_edges) {
    adj[e.first].push_back(e.second);
    adj[e.second].push_back(e.first);
  }

  // BFS from source
  std::queue<Vertex> q;
  q.push(source);
  while (!q.empty()) {
    auto v = q.front();
    q.pop();

    for (auto u : adj[v]) {
      if (v == source || u != parent[v]) {
        parent[u] = v;  // every vertex except source must have one parent
        q.push(u);
      }
    }
  }

  // find a path between source and destination
  for (auto t = destination; t != source; t = parent[t]) last_child[parent[t]] = t;

  // DFS from source
  std::vector<Vertex> walk = {source};
  std::stack<std::pair<Vertex, bool>> s;
  auto enqueue = [&](Vertex v) {
    // visit last child last
    if (util::contains(last_child, v)) s.push({last_child[v], true});  // go down

    // visit other children
    for (auto u : adj[v]) {
      if (util::contains(parent, v) && parent[v] == u) continue;
      if (util::contains(last_child, v) && last_child[v] == u) continue;

      s.push({v, false});  // go up
      s.push({u, true});   // go down
    }
  };
  enqueue(source);

  while (!s.empty()) {
    auto v = s.top().first;
    auto direction = s.top().second;  // down (true) or up (false)
    s.pop();
    walk.push_back(v);

    if (direction) enqueue(v);
  }

  return walk;
}
}  // namespace impl

/**
 * @brief Finds a solution using the edges in a minimum spanning tree.
 *
 * @param source starting vertex
 * @param destination ending vertex (can be same as source)
 *
 * @return std::vector<Vertex> walk visiting all vertices
 */
std::vector<BaseSolver::Vertex> BaseSolver::find_mst_solution(Vertex source, Vertex destination) const {
  // compute a minimum spanning tree
  auto mst_edges = algorithm::graph::minimum_spanning_tree(get_graph());

  // find a walk
  return impl::find_walk_on_tree(mst_edges, source, destination);
}

/**
 * @brief Finds a list of vertices that collect k colors nearest to either source or destination.
 *
 * @param k number of colors to collect
 * @param source starting vertex
 * @param destination ending vertex
 * @param rand pointer to util::Random instance or nullptr (no randomization)
 * @return std::vector<Vertex> list of terminals (including source and destination)
 */
std::vector<BaseSolver::Vertex> BaseSolver::find_nearest_terminals(int k, Vertex source, Vertex destination, util::Random *rand) const {
  std::vector<Vertex> ret = {source};
  auto &g = get_graph();
  auto distances = algorithm::graph::single_source_shortest_paths(g, source);
  if (source != destination) {
    ret.push_back(destination);
    for (auto &p : algorithm::graph::single_source_shortest_paths(g, destination)) {
      distances[p.first] = std::min(distances.at(p.first), p.second);
    }
  }

  std::vector<std::tuple<Weight, double, Vertex>> vs;
  for (auto &p : distances) { vs.push_back({p.second, rand ? rand->random() : 0.0, p.first}); }

  // sort vertices by weight with random tie-breaking
  std::sort(vs.begin(), vs.end());

  ColorSet seen = g.get_colors(source) | g.get_colors(destination);
  for (auto &[weight, random_value, v] : vs) {
    if (static_cast<int>(seen.size()) >= k) break;  // already collected k colors
    auto new_colors = g.get_colors(v) - seen;
    if (!new_colors.empty()) {
      ret.push_back(v);
      seen |= new_colors;
    }
  }
  return ret;
}

/**
 * @brief Finds a solution using the edges in an (approximate) Steiner tree with the nearest terminals.
 *
 * @param k number of colors to collect
 * @param source starting vertex
 * @param destination ending vertex
 * @param rand pointer to util::Random instance or nullptr (no randomization)
 */
double BaseSolver::find_steiner_tree_solution(int k, Vertex source, Vertex destination, util::Random *rand) {
  //----------------------------------------------------------------------------
  //    1. Find nearest terminals
  //----------------------------------------------------------------------------
  auto terminals = find_nearest_terminals(k, source, destination, rand);

  //----------------------------------------------------------------------------
  //    2. Compute distances between all terminal pairs
  //----------------------------------------------------------------------------
  auto &g = get_graph();
  auto dist = algorithm::graph::all_pairs_dijkstra_path_length(g, terminals);

  //----------------------------------------------------------------------------
  //    3. Create a transitive closure induced on the terminals
  //----------------------------------------------------------------------------
  ds::graph::Graph h;
  for (auto v : terminals) h.add_vertex(v, g.get_colors(v).to_vector());
  for (auto &p : dist) {
    auto e = g.from_edgekey(p.first);
    h.add_edge(e.first, e.second, p.second);
  }

  //----------------------------------------------------------------------------
  //    4. Compute the minimum spanning tree of the induced graph
  //----------------------------------------------------------------------------
  auto mst_edges = algorithm::graph::minimum_spanning_tree(h);

  //----------------------------------------------------------------------------
  //    5. Find a walk using MST edges
  //----------------------------------------------------------------------------
  auto critical_walk = impl::find_walk_on_tree(mst_edges, source, destination);

  //----------------------------------------------------------------------------
  //    6. Register the critical walk
  //----------------------------------------------------------------------------
  return set_critical_walk(critical_walk);
}

/**
 * @brief Repeatedly smoothes vertices without colors and degree at most 2.
 *
 * @param exceptions vertices to keep
 * @return number of vertices that have been removed
 */
int BaseSolver::smooth_colorless_vertices(std::vector<Vertex> const &exceptions) {
  int num_removed = 0;
  std::queue<Vertex> q;

  for (auto v : graph_.vertices()) {
    if (graph_.degree(v) <= 2 && graph_.get_colors(v).empty() && !util::contains(exceptions, v)) q.push(v);
  }

  while (!q.empty()) {
    auto v = q.front();
    q.pop();
    if (!graph_.has_vertex(v)) continue;  // already removed

    ++num_removed;

    if (graph_.degree(v) == 2) {
      // If v has 2 neighbors, create a neighborhood closure with the updated weight.
      auto nbrs = graph_.neighbors(v);
      Vertex u = nbrs[0], w = nbrs[1];
      auto weight = graph_.get_weight(v, u) + graph_.get_weight(v, w);

      if (graph_.has_edge(u, w)) {
        graph_.set_weight(u, w, std::min(graph_.get_weight(u, w), weight));
      } else {
        graph_.add_edge(u, w, weight);
      }
    }

    // possible chain effect
    for (auto x : graph_.neighbors(v)) {
      if (graph_.degree(x) <= 3 && graph_.get_colors(x).empty() && !util::contains(exceptions, x)) q.push(x);
    }

    // Remove vertex v.
    labels_.erase(v);
    graph_.remove_vertex(v);
  }

  return num_removed;
}
}  // namespace base
}  // namespace algorithm
