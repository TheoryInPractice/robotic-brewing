#pragma once
#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "ds/set/SortedVectorSet.hpp"
#include "util/util.hpp"

namespace ds {
namespace graph {

/**
 * @brief Models a simple undirected graph with vertex colors and edge weights.
 * One vertex may have multiple colors.
 */
class Graph {
 public:
  typedef int Vertex;                         // vertex label
  typedef int Color;                          // vertex color
  typedef ds::set::SortedVectorSet AdjList;   // adjacency list
  typedef ds::set::SortedVectorSet ColorSet;  // color set
  typedef double Weight;                      // edge weight
  typedef std::pair<Vertex, Vertex> Edge;     // edge type
  typedef uint64_t EdgeKey;                   // hashable edge identifier
  typedef std::vector<Vertex> VertexList;
  typedef std::vector<std::pair<Vertex, std::vector<Color>>> ColoredVertexList;
  typedef std::vector<Edge> EdgeList;
  typedef std::vector<std::pair<Edge, Weight>> WeightedEdgeList;

 private:
  /** Adjacency lists. */
  std::unordered_map<Vertex, AdjList> adj_;

  /** Vertex colors. */
  std::unordered_map<Vertex, ColorSet> colors_;

  /** Edge weights. */
  std::unordered_map<EdgeKey, Weight> weights_;

 public:
  /**
   * @brief Constructs a new Graph object.
   *
   * @param vertices list of vertices in the graph
   * @param edges list of edges in the graph
   */
  Graph(std::vector<std::pair<Vertex, std::vector<Color>>> const& vertices = {},
        std::vector<std::pair<Edge, Weight>> const& edges = {}) {
    for (auto v : vertices) add_vertex(v.first, v.second);
    for (auto& e : edges) add_edge(e.first.first, e.first.second, e.second);
  }

  //==================================================================================================
  //    Global Properties
  //==================================================================================================
  /**
   * @brief Returns the number of vertices in the graph.
   *
   * @return std::size_t number of vertices
   */
  std::size_t number_of_vertices() const { return adj_.size(); }

  /**
   * @brief Returns the number of edges in the graph.
   *
   * @return std::size_t number of edges
   */
  std::size_t number_of_edges() const { return weights_.size(); }

  /**
   * @brief Returns the number of total colors present in the graph.
   *
   * @return std::size_t number of colors
   *
   * Time complexity: O(n |C| log |C|)
   */
  std::size_t number_of_colors() const {
    ColorSet ret;
    for (auto& p : colors_) ret |= p.second;
    return ret.size();
  }

  //==================================================================================================
  //    Vertex Enumeration
  //==================================================================================================
  /**
   * @brief Returns a sorted list of all vertices.
   *
   * @return VertexList all vertices
   */
  VertexList vertices(bool sorted = false) const {
    VertexList ret;
    for (auto& p : adj_) ret.push_back(p.first);
    if (sorted) std::sort(ret.begin(), ret.end());
    return ret;
  }

  ColoredVertexList colored_vertices(bool sorted = false) const {
    ColoredVertexList ret;
    for (auto& p : colors_) ret.push_back(std::make_pair(p.first, p.second.to_vector()));
    if (sorted) std::sort(ret.begin(), ret.end());
    return ret;
  }

  /**
   * @brief Gets the entire color set of the graph.
   *
   * @param v vertex label
   * @return ColorSet color set
   */
  ColorSet get_colors() const {
    ColorSet ret;
    for (auto& p : colors_) ret |= p.second;
    return ret;
  }

  //==================================================================================================
  //    Edge Enumeration
  //==================================================================================================
  EdgeList edges(bool sorted = false) const {
    EdgeList ret;
    for (auto& p : weights_) ret.push_back(from_edgekey(p.first));
    if (sorted) std::sort(ret.begin(), ret.end());
    return ret;
  }

  WeightedEdgeList weighted_edges(bool sorted = false) const {
    WeightedEdgeList ret;
    for (auto& p : weights_) ret.push_back(std::make_pair(from_edgekey(p.first), p.second));
    if (sorted) std::sort(ret.begin(), ret.end());
    return ret;
  }

  //==================================================================================================
  //    Vertex Properties
  //==================================================================================================
  /**
   * @brief Checks if the graph has the given vertex.
   *
   * @param v vertex label
   * @return true vertex v exists
   * @return false vertex v does not exist
   */
  bool has_vertex(Vertex v) const { return util::contains(adj_, v); }

  /**
   * @brief Adds a new vertex to the graph.
   *
   * @param v vertex label
   * @param colors list of colors
   */
  void add_vertex(Vertex v, std::vector<Color> const& colors) {
    if (has_vertex(v)) throw std::invalid_argument("vertex already exists");
    adj_[v] = {};
    colors_[v] = colors;
  }

  /**
   * @brief Gets the color set of the given vertex.
   *
   * @param v vertex label
   * @return ColorSet const& color set
   */
  ColorSet const& get_colors(Vertex v) const {
    if (!has_vertex(v)) throw std::invalid_argument("vertex must be present");
    return colors_.at(v);
  }

  /**
   * @brief Updates colors of the given vertex.
   *
   * @param v vertex label
   * @param colors new colors
   */
  void set_colors(Vertex v, ColorSet const& colors) { colors_[v] = colors; }

  /**
   * @brief Removes a vertex from the graph.
   *
   * @param v vertex label
   */
  void remove_vertex(Vertex v) {
    if (!has_vertex(v)) throw std::invalid_argument("vertex must be present");
    for (auto u : neighbors(v)) remove_edge(u, v);
    adj_.erase(v);
    colors_.erase(v);
  }

  //==================================================================================================
  //    Edge Properties
  //==================================================================================================
  /**
   * @brief Checks if the graph has the given edge.
   *
   * @param u endpoint 1
   * @param v endpoint 2
   * @return true edge uv exists
   * @return false edge uv does not exist
   */
  bool has_edge(Vertex u, Vertex v) const { return util::contains(adj_, u) && adj_.at(u).get(v); }

  /**
   * @brief Adds a new edge to the graph.
   *
   * @param u endpoint 1
   * @param v endpoint 2
   * @param w edge weight
   */
  void add_edge(Vertex u, Vertex v, Weight w) {
    if (!has_vertex(u) || !has_vertex(v)) throw std::invalid_argument("vertex must be present");
    adj_[u].set(v);
    adj_[v].set(u);
    weights_[to_edgekey(u, v)] = w;
  }

  /**
   * @brief Removes an edge from the graph.
   *
   * @param u endpoint 1
   * @param v endpoint 2
   */
  void remove_edge(Vertex u, Vertex v) {
    if (!has_vertex(u) || !has_vertex(v)) throw std::invalid_argument("vertex must be present");
    adj_[u].reset(v);
    adj_[v].reset(u);
    weights_.erase(to_edgekey(u, v));
  }

  /**
   * @brief Gets the weight on the given edge.
   *
   * @param u endpoint 1
   * @param v endpoint 2
   * @return Weight edge weight
   */
  Weight get_weight(Vertex u, Vertex v) const {
    if (!has_edge(u, v)) throw std::invalid_argument("edge does not exist");
    return weights_.at(to_edgekey(u, v));
  }

  /**
   * @brief Updates the weight of the given edge.
   *
   * @param u endpoint 1
   * @param v endpoint 2
   * @param w non-negative weight
   */
  void set_weight(Vertex u, Vertex v, Weight w) {
    if (!has_edge(u, v)) throw std::invalid_argument("edge does not exist");
    if (w < 0) throw std::invalid_argument("weight must be non-negative");
    weights_[to_edgekey(u, v)] = w;
  }

  /**
   * @brief Converts a vertex pair (not necessarily adjacent) to the edge key.
   *
   * @param u vertex 1
   * @param v vertex 2
   * @return EdgeKey edge key
   */
  EdgeKey to_edgekey(Vertex u, Vertex v) const { return (static_cast<EdgeKey>(std::min(u, v)) << 32) + std::max(u, v); }

  /**
   * @brief Converts an edge key to the vertex pair.
   *
   * @param key edge key
   * @return std::pair<Vertex, Vertex> pair of endpoints
   */
  std::pair<Vertex, Vertex> from_edgekey(EdgeKey key) const {
    return std::make_pair(static_cast<Vertex>(key >> 32), static_cast<Vertex>(key & ((1ULL << 32) - 1)));
  }

 public:
  //==================================================================================================
  //    Neighbors
  //==================================================================================================
  /**
   * @brief Returns the degree at the given vertex.
   *
   * @param v vertex label
   * @return std::size_t degree at vertex v
   */
  std::size_t degree(Vertex v) const {
    if (!has_vertex(v)) throw std::invalid_argument("vertex does not exist");
    return adj_.at(v).size();
  }

  /**
   * @brief Returns a list of the neighbors of the given vertex.
   *
   * @param v vertex label
   * @return std::vector<int> list of neighbors
   */
  std::vector<int> neighbors(Vertex v) const {
    if (!has_vertex(v)) throw std::invalid_argument("vertex does not exist");
    return adj_.at(v).to_vector();
  }

  //==================================================================================================
  //    I/O
  //==================================================================================================
  /**
   * @brief String representation of the instance.
   *
   * @param os output stream
   * @param graph Graph instance
   * @return std::ostream& output stream
   */
  friend std::ostream& operator<<(std::ostream& os, Graph const& graph) {
    return os << "Graph(n=" << graph.number_of_vertices() << ",m=" << graph.number_of_edges() << ")";
  }
};
}  // namespace graph
}  // namespace ds
