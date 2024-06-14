#pragma once

#include <vector>

#include "ds/graph/Graph.hpp"

namespace algorithm {
namespace merge {
class BaseMerger {
 protected:
  using Walk = std::vector<ds::graph::Graph::Vertex>;

 public:
  /**
   * @brief Constructs a new BaseMerger instance.
   */
  BaseMerger() {}

  virtual ~BaseMerger() = default;

  virtual Walk merge(ds::graph::Graph const& graph, std::vector<Walk> const& walks) const = 0;

  /**
   * @brief Represents a multigraph with edge multiplicities.
   */
  struct MultiGraph {
    /** Starting vertex. */
    ds::graph::Graph::Vertex source;

    /** Underlying simple graph. */
    ds::graph::Graph graph;

    /** Map of each edge and its multiplicity. */
    std::map<ds::graph::Graph::Edge, std::size_t> multiplicity;

    /**
     * @brief Returns a list of the edges with multiplicity 1.
     *
     * @return std::vector<ds::graph::Graph::Edge> list of one-way edges
     */
    std::vector<ds::graph::Graph::Edge> get_one_way_edges() {
      std::vector<ds::graph::Graph::Edge> ret;
      for (auto& p : multiplicity) {
        if (p.second == 1) ret.push_back(p.first);
      }
      return ret;
    }
  };

 protected:
  void validate_walks(ds::graph::Graph const& graph, std::vector<Walk> const& walks) const {
    if (walks.empty()) throw std::invalid_argument("walk list is empty");

    for (auto& walk : walks) {
      if (walk.empty()) throw std::invalid_argument("found an empty walk");
    }

    ds::graph::Graph::Vertex source = walks.front().front();
    for (auto& walk : walks) {
      if (walk.front() != source || walk.back() != source) throw std::invalid_argument("found an open walk");
      for (auto v : walk) {
        if (!graph.has_vertex(v)) throw std::invalid_argument("graph does not have a vertex in a walk");
      }
    }
  }

  /**
   * @brief Creates a multigraph representing the union of the walks.
   * All vertices except the starting vertex have unique colors.
   *
   * @param graph original edge-weighted graph
   * @param walks list of walks to merge
   * @return MultiGraph
   */
  MultiGraph create_multigraph(ds::graph::Graph const& graph, std::vector<Walk> const& walks) const {
    ds::graph::Graph::Vertex source = walks.front().front();

    ds::graph::Graph simple_graph;
    ds::graph::Graph::Color color_index = 0;

    for (auto& walk : walks) {
      for (auto v : walk) {
        if (!simple_graph.has_vertex(v)) {
          std::vector<ds::graph::Graph::Color> colors;

          // every non-source vertex has a unique color
          if (v != source) colors.push_back(color_index++);
          simple_graph.add_vertex(v, colors);
        }
      }
    }

    std::map<ds::graph::Graph::Edge, std::size_t> multiplicity;

    for (auto& walk : walks) {
      for (std::size_t i = 1; i < walk.size(); ++i) {
        ds::graph::Graph::Vertex u = walk[i - 1], v = walk[i];
        if (u == v) continue;  // filter out self-loops

        auto edge = std::make_pair(std::min(u, v), std::max(u, v));
        ++multiplicity[edge];
        if (multiplicity[edge] == 1UL) {
          // new edge
          simple_graph.add_edge(u, v, graph.get_weight(u, v));
        }
      }
    }

    return MultiGraph({source, simple_graph, multiplicity});
  }
};
}  // namespace merge
}  // namespace algorithm
