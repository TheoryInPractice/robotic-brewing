#pragma once

#include "algorithm/graph/connectivity.hpp"
#include "algorithm/graph/eulerian.hpp"
#include "algorithm/merge/BaseMerger.hpp"
#include "ds/tree/UnionFind.hpp"
#include "util/logger.hpp"

namespace algorithm {
namespace merge {
class HeuristicMerger : public BaseMerger {
 public:
  /**
   * @brief Merges and simplifies all walks using heuristics.
   *
   * @param graph graph
   * @param walks input walks
   * @return std::vector<ds::graph::Graph::Vertex> merged walk
   */
  std::vector<ds::graph::Graph::Vertex> merge(ds::graph::Graph const& graph, std::vector<Walk> const& walks) const override {
    validate_walks(graph, walks);

    using Graph = ds::graph::Graph;
    using Vertex = ds::graph::Graph::Vertex;
    using Edge = ds::graph::Graph::Edge;
    using Weight = ds::graph::Graph::Weight;

    //----------------------------------------------------------------------------
    //    1. Convert input to a multigraph
    //----------------------------------------------------------------------------
    MultiGraph g = create_multigraph(graph, walks);

    //----------------------------------------------------------------------------
    //    2. Preprocess
    //----------------------------------------------------------------------------
    // If an edge multiplicity is greater than 3, repeatedly decrease it by 2.
    for (auto& p : g.multiplicity) {
      if (p.second > 3) p.second = 2 + (p.second % 2);
    }

    std::set<Edge> undeletable_edges;
    auto bridges = algorithm::graph::find_bridges(g.graph);
    // If an edge is a bridge in the underlying graph, mark as undeletable.
    for (auto& e : bridges) {
      assert(g.multiplicity[e] % 2 == 0);
      undeletable_edges.insert(e);
    }

    // If an edge has multiplicity 1 and is part of an edge cut of size 2,
    // mark as undeletable.
    for (auto& p : g.multiplicity) {
      auto& e = p.first;
      if (p.second != 1 || util::contains(undeletable_edges, e)) continue;
      auto bridges_in_between = algorithm::graph::find_bridges_in_between(g.graph, e.first, e.second);
      if (bridges_in_between.empty()) continue;  // no edge cut of size 2 including edge e

      for (auto& b : bridges_in_between) {
        if (g.multiplicity[b] == 1) {
          undeletable_edges.insert(e);
          undeletable_edges.insert(b);
        }
      }
    }

    log_trace("Undeletable edges: %s", cstr(undeletable_edges));

    //----------------------------------------------------------------------------
    //    3. Construct a spanning subgraph
    //----------------------------------------------------------------------------
    ds::tree::UnionFind uf;

    // sort remaining edges by weight
    std::vector<std::pair<Weight, Edge>> edges, remainder;

    for (auto& p : g.multiplicity) {
      auto& e = p.first;
      if (util::contains(undeletable_edges, e)) {
        uf.Union(e.first, e.second);
      } else {
        for (std::size_t i = 0; i < p.second; ++i) {
          edges.push_back(std::make_pair(graph.get_weight(e.first, e.second), e));
        }
      }
    }
    std::sort(edges.begin(), edges.end());
    for (auto& edge : edges) {
      if (!uf.Union(edge.second.first, edge.second.second)) remainder.push_back(edge);
    }

    //----------------------------------------------------------------------------
    //    4. Find a maximal cycle packing in the remainder graph
    //----------------------------------------------------------------------------
    Graph h(g.graph.colored_vertices(), {});           // graph to remove redundant cycles
    std::reverse(remainder.begin(), remainder.end());  // order by weight in non-increasing order

    std::map<Vertex, std::optional<Vertex>> parent;
    std::map<Edge, std::size_t> removed_multiplicity;

    for (auto& p : remainder) {
      auto& e = p.second;
      Vertex u = e.first, v = e.second;
      assert(u < v);

      // find a u-v path in the current graph
      auto path = algorithm::graph::find_path(h, u, v);
      if (path.empty()) {
        // add edge uv
        h.add_edge(u, v, 1);
      } else {
        // remove the u-v path and edge uv
        ++removed_multiplicity[e];
        for (std::size_t i = 1; i < path.size(); ++i) {
          Vertex x = path[i - 1], y = path[i];
          ++removed_multiplicity[std::make_pair(std::min(x, y), std::max(x, y))];
          h.remove_edge(x, y);
        }
      }
    }

    //----------------------------------------------------------------------------
    //    5. Reconstruct an Euler tour
    //----------------------------------------------------------------------------
    std::vector<Edge> final_edges;
    for (auto& p : g.multiplicity) {
      auto& e = p.first;
      assert(p.second >= removed_multiplicity[e]);

      for (std::size_t i = 0; i < p.second - removed_multiplicity[e]; ++i) final_edges.push_back(e);
    }

    Walk ret = algorithm::graph::eulerian_path(final_edges, g.source);
    return ret;
  }
};
}  // namespace merge
}  // namespace algorithm
