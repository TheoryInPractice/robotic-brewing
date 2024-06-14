#include "connectivity.hpp"
#include "util/util.hpp"

#include <optional>

using namespace ds::graph;

namespace algorithm {
namespace graph {
/**
 * @brief Returns all vertices reachable from the source veretx.
 *
 * @param graph graph
 * @param source source vertex
 * @return std::vector<ds::graph::Graph::Vertex> connected component
 */
std::vector<ds::graph::Graph::Vertex> connected_component(ds::graph::Graph const& graph, ds::graph::Graph::Vertex source) {
  // run BFS
  std::unordered_set<ds::graph::Graph::Vertex> visited;
  visited.insert(source);

  std::queue<ds::graph::Graph::Vertex> q;
  q.push(source);
  while (!q.empty()) {
    auto u = q.front();
    q.pop();
    for (auto v : graph.neighbors(u)) {
      if (!util::contains(visited, v)) {
        visited.insert(v);
        q.push(v);
      }
    }
  }
  return std::vector<ds::graph::Graph::Vertex>(visited.begin(), visited.end());
}

/**
 * @brief Finds all bridges in the graph.
 *
 * @param graph graph
 * @return std::vector<ds::graph::Graph::Edge> list of bridges
 */
std::vector<ds::graph::Graph::Edge> find_bridges(ds::graph::Graph const& graph) {
  using Vertex = ds::graph::Graph::Vertex;
  using Edge = ds::graph::Graph::Edge;

  struct Context {
    int dfsnum = -1;
    int dfslow = 0;
    std::optional<Vertex> parent = std::nullopt;
  };

  std::vector<Edge> bridges;
  std::map<Vertex, Context> ctx;
  int dfscnt = 0;

  // inner function
  auto f = [&](Vertex v, auto& self) -> void {
    // visit vertex v
    ctx[v].dfsnum = ctx[v].dfslow = dfscnt++;

    for (auto u : graph.neighbors(v)) {
      if (ctx[u].dfsnum < 0) {  // unvisited
        ctx[u].parent = v;
        self(u, self);  // recurse

        // backtrack
        if (ctx[u].dfslow > ctx[v].dfsnum) {
          // found a bridge
          bridges.push_back(std::make_pair(std::min(u, v), std::max(u, v)));
        }
        ctx[v].dfslow = std::min(ctx[v].dfslow, ctx[u].dfslow);
      } else if (u != ctx[v].parent) {
        // found a back edge
        ctx[v].dfslow = std::min(ctx[v].dfslow, ctx[u].dfsnum);
      }
    }
  };

  for (auto v : graph.vertices()) {
    if (ctx[v].dfsnum < 0) f(v, f);
  }
  return bridges;
}

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
    ds::graph::Graph const& graph, ds::graph::Graph::Vertex u, ds::graph::Graph::Vertex v) {
  if (!graph.has_vertex(u)) throw std::invalid_argument("graph does not contain vertex u");
  if (!graph.has_vertex(v)) throw std::invalid_argument("graph does not contain vertex v");

  using Vertex = ds::graph::Graph::Vertex;
  std::vector<ds::graph::Graph::Vertex> ret;

  // quick check
  if (u == v) return {u};
  if (graph.degree(u) == 0 || graph.degree(v) == 0) return ret;

  // breadth-first search from v
  std::map<Vertex, std::optional<Vertex>> parent = {{v, std::nullopt}};
  std::queue<Vertex> q;
  q.push(v);

  while (!q.empty()) {
    auto x = q.front();
    q.pop();

    for (auto y : graph.neighbors(x)) {
      if (util::contains(parent, y)) continue;  // already visited
      parent[y] = x;
      if (y == u) goto search_end;  // reached vertex u
      q.push(y);
    }
  }

search_end:
  if (!util::contains(parent, u)) return ret;  // no u-v path

  for (auto x = std::optional<Vertex>(u); x.has_value(); x = parent.at(x.value())) ret.push_back(x.value());
  return ret;
}

/**
 * @brief Finds bridges in a u-v path in G-uv.
 *
 * @param graph graph
 * @param u vertex u
 * @param v vertex v
 * @return std::vector<ds::graph::Graph::Edge> list of bridges in a u-v path withtout using edge uv
 */
std::vector<ds::graph::Graph::Edge> find_bridges_in_between(  //
    ds::graph::Graph const& graph, ds::graph::Graph::Vertex u, ds::graph::Graph::Vertex v) {
  if (!graph.has_edge(u, v)) throw std::invalid_argument("no edge uv");

  using Vertex = ds::graph::Graph::Vertex;
  using Edge = ds::graph::Graph::Edge;

  struct Context {
    int dfsnum = -1;
    int dfslow = 0;
    std::optional<Vertex> parent = std::nullopt;
    bool is_bridge = false;  // the edge between this vertex and its parent is a bridge
  };

  std::map<Vertex, Context> ctx;
  int dfscnt = 0;
  bool found_v = false;

  // inner function
  auto f = [&](Vertex x, auto& self) -> void {
    found_v |= x == v;

    // visit vertex x
    ctx[x].dfsnum = ctx[x].dfslow = dfscnt++;

    for (auto y : graph.neighbors(x)) {
      if ((x == u && y == v) || (x == v && y == u)) continue;  // ignore edge uv

      if (ctx[y].dfsnum < 0) {  // unvisited
        ctx[y].parent = x;
        self(y, self);  // recurse

        // backtrack
        if (ctx[y].dfslow > ctx[x].dfsnum) {
          // found a bridge
          ctx[y].is_bridge = true;
        }
        ctx[x].dfslow = std::min(ctx[x].dfslow, ctx[y].dfslow);
      } else if (y != ctx[x].parent) {
        // found a back edge
        ctx[x].dfslow = std::min(ctx[x].dfslow, ctx[y].dfsnum);
      }
      if (x == u && found_v) break;  // done
    }
  };

  f(u, f);

  // collect bridges in a u-v path
  std::vector<Edge> bridges;
  for (auto x = v; x != u; x = ctx[x].parent.value()) {
    if (ctx[x].is_bridge) {
      auto y = ctx[x].parent.value();
      bridges.push_back(std::make_pair(std::min(x, y), std::max(x, y)));
    }
    if (!ctx[x].parent.has_value()) break;
  }
  return bridges;
}
}  // namespace graph
}  // namespace algorithm
