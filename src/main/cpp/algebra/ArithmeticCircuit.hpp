#pragma once
#include <algorithm>
#include <cassert>
#include <queue>
#include <unordered_map>
#include <vector>

#include "algebra/ring/TropicalSemiring.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

// Set to 1 for debugging.
#define ARITHMETIC_CIRCUIT_TRACE_CONSTRUCTION 0

namespace algebra {

/**
 * @brief Models an arithmetic circuit consisting of source nodes, addition gates,
 * and multiplication gates with no scalar nodes.
 */
template <typename Set>
class ArithmeticCircuit {
 public:
  typedef int Node;
  typedef std::pair<Node, Node> Edge;

  /** Type of a node in the circuit. */
  enum NodeType { Variable, Addition, Multiplication, Removed };

  ArithmeticCircuit(std::size_t number_of_variables)
      : num_variables_(number_of_variables),
        num_addition_gates_(0),
        num_multiplication_gates_(0),
        num_output_nodes_(0),
        num_edges_(0),
        in_neighbors_(number_of_variables),
        out_neighbors_(number_of_variables),
        node_types_(number_of_variables, NodeType::Variable),
        is_output_(number_of_variables, false) {
    assert(number_of_variables >= 1);
  }

  ArithmeticCircuit& operator=(ArithmeticCircuit other) {
    num_variables_ = other.num_variables_;
    num_addition_gates_ = other.num_addition_gates_;
    num_multiplication_gates_ = other.num_multiplication_gates_;
    num_output_nodes_ = other.num_output_nodes_;
    num_edges_ = other.num_edges_;
    in_neighbors_ = other.in_neighbors_;
    out_neighbors_ = other.out_neighbors_;
    node_types_ = other.node_types_;
    is_output_ = other.is_output_;
    removed_ = other.removed_;
    return *this;
  }

  /**
   * @brief Returns the number of variable (i.e. source) nodes in the circuit.
   *
   * @return std::size_t number of variables
   */
  constexpr std::size_t number_of_variables() const { return num_variables_; }

  /**
   * @brief Returns the number of nodes in the circuit.
   *
   * @return std::size_t number of nodes
   */
  std::size_t number_of_nodes() const { return node_types_.size() - removed_.size(); }

  /**
   * @brief Returns the number of addition gates in the circuit.
   *
   * @return std::size_t number of addition gates
   */
  std::size_t number_of_addition_gates() const { return num_addition_gates_; }

  /**
   * @brief Returns the number of multiplication gates in the circuit.
   *
   * @return std::size_t number of multiplication gates
   */
  std::size_t number_of_multiplication_gates() const { return num_multiplication_gates_; }

  /**
   * @brief Returns the number of output nodes in the circuit.
   *
   * @return std::size_t number of output nodes
   */
  std::size_t number_of_output_nodes() const { return num_output_nodes_; }

  /**
   * @brief Returns the number of edges in the circuit.
   *
   * @return std::size_t number of edges
   */
  std::size_t number_of_edges() const { return num_edges_; }

  Node add_addition_gate(std::vector<Node> const& in_neighbors, bool set_as_output = true) {
    ++num_addition_gates_;
    Node ret = add_gate(NodeType::Addition, in_neighbors, set_as_output);
#if ARITHMETIC_CIRCUIT_TRACE_CONSTRUCTION
    log_trace("[+:%d] in-nbrs:%s", ret, cstr(in_neighbors));
#endif
    return ret;
  }

  Node add_multiplication_gate(std::vector<Node> const& in_neighbors, bool set_as_output = true) {
    ++num_multiplication_gates_;
    Node ret = add_gate(NodeType::Multiplication, in_neighbors, set_as_output);
#if ARITHMETIC_CIRCUIT_TRACE_CONSTRUCTION
    log_trace("[*:%d] in-nbrs:%s", ret, cstr(in_neighbors));
#endif
    return ret;
  }

  void set_output_node(Node output_node) {
    if (!is_output_[output_node]) {
      ++num_output_nodes_;
      is_output_[output_node] = true;
    }
  }

  void reset_output_node(Node output_node) {
    if (is_output_[output_node]) {
      --num_output_nodes_;
      is_output_[output_node] = false;
    }
  }

  std::vector<Node> get_output_nodes() const {
    std::vector<Node> ret;
    for (Node i = num_variables_; i < static_cast<Node>(is_output_.size()); ++i) {
      if (is_output_[i]) ret.push_back(i);
    }
    assert(ret.size() == num_output_nodes_);
    return ret;
  }

  void add_edge(Node u, Node v) {
    assert(node_types_[u] != NodeType::Removed);
    assert(node_types_[v] != NodeType::Removed);

    add_edge_(u, v);

#if ARITHMETIC_CIRCUIT_TRACE_CONSTRUCTION
    log_trace("[e] %d->%d", u, v);
#endif
  }

  bool has_node(Node v) const {
    return 0 <= v && v < static_cast<Node>(node_types_.size()) && node_types_[v] != NodeType::Removed;
  }

  bool has_edge(Node u, Node v) const { return has_node(u) && has_node(v) && out_neighbors_[u].get(v); }

  void remove_node(Node v) {
    reset_output_node(v);

    switch (node_types_[v]) {
      case NodeType::Variable: {
        throw std::invalid_argument("Variable nodes cannot be removed.");
        break;
      }
      case NodeType::Addition: {
        --num_addition_gates_;
        break;
      }
      case NodeType::Multiplication: {
        --num_multiplication_gates_;
        break;
      }
      case NodeType::Removed: {
        throw std::invalid_argument("Already removed node.");
        break;
      }
      default: {
        throw std::invalid_argument("Unknown node type.");
      }
    }

    num_edges_ -= in_neighbors_[v].size() + out_neighbors_[v].size();
    for (auto u : in_neighbors_[v].to_vector()) out_neighbors_[u].reset(v);
    for (auto u : out_neighbors_[v].to_vector()) in_neighbors_[u].reset(v);
    in_neighbors_[v].clear();
    out_neighbors_[v].clear();
    node_types_[v] = NodeType::Removed;
    removed_.push_back(v);
  }

  void remove_edge(Node u, Node v) {
    assert(node_types_[u] != NodeType::Removed);
    assert(node_types_[v] != NodeType::Removed);

    in_neighbors_[v].reset(u);
    out_neighbors_[u].reset(v);
    --num_edges_;
  }

  /**
   * @brief Removes the given node and other dangling nodes if such ones exist.
   *
   * @param v node to remove first
   */
  void remove_and_clean_node(Node v) {
    std::queue<std::pair<Node, int>> q;  // (node, direction)
    q.push({v, 3});
    while (!q.empty()) {
      auto p = q.front();
      q.pop();
      auto u = p.first;
      if (p.second & 1) {  // backward
        for (auto w : in_neighbors_[u].to_vector()) {
          if (out_degree(w) == 1 && !is_variable_node(w) && !is_output_node(w)) q.push({w, 1});
        }
      }
      if (p.second & 2) {  // forward
        for (auto w : out_neighbors_[u].to_vector()) {
          if (in_degree(w) == 1 && !is_variable_node(w) && !is_output_node(w)) q.push({w, 2});
        }
      }
      remove_node(u);
    }
  }

  //----------------------------------------------------------------------------
  //    Property accessors
  //----------------------------------------------------------------------------

  bool is_variable_node(Node v) const { return has_node(v) && node_types_[v] == NodeType::Variable; }
  bool is_addition_node(Node v) const { return has_node(v) && node_types_[v] == NodeType::Addition; }
  bool is_multiplication_node(Node v) const { return has_node(v) && node_types_[v] == NodeType::Multiplication; }
  bool is_output_node(Node v) const { return has_node(v) && is_output_[v]; }

  int in_degree(Node v) const {
    assert(has_node(v));
    return in_neighbors_[v].size();
  }

  int out_degree(Node v) const {
    assert(has_node(v));
    return out_neighbors_[v].size();
  }

  std::vector<Node> get_in_neighbors(Node v) const {
    assert(has_node(v));
    return in_neighbors_.at(v).to_vector();
  }

  std::vector<Node> get_out_neighbors(Node v) const {
    assert(has_node(v));
    return out_neighbors_.at(v).to_vector();
  }

  std::vector<Node> topological_ordering(bool reverse) const {
    std::vector<Node> ret;

    auto n = node_types_.size();
    std::vector<int> deg(n);  // degrees to keep track
    auto& in = reverse ? out_neighbors_ : in_neighbors_;
    auto& out = reverse ? in_neighbors_ : out_neighbors_;

    for (std::size_t i = 0; i < n; ++i) deg[i] = in[i].size();
    for (auto v : removed_) deg[v] = -1;  // invalidate removed nodes

    // repeatedly choose a vertex with no parents.
    std::queue<Node> q;
    if (reverse) {
      for (auto output_node : get_output_nodes()) q.push(output_node);  // traverse from output nodes
    } else {
      for (std::size_t i = 0; i < num_variables_; ++i) q.push(i);  // traverse from source nodes
    }

    while (!q.empty()) {
      auto p = q.front();
      q.pop();
      ret.push_back(p);
      // log_warning("visiting: %d", p);

      for (auto u : out[p].to_vector()) {
        if (--deg[u] == 0) q.push(u);
      }
    }
    return ret;
  }

  /**
   * @brief Performs breadth first search (BFS) from source.
   *
   * @param source source node
   * @param reverse if true, reverse all edges while the traversal
   * @return std::vector<Node> bfs ordering of all nodes reachable from source
   */
  std::vector<Node> bfs(Node source, bool reverse) const {
    std::vector<Node> ret;
    auto& out = reverse ? in_neighbors_ : out_neighbors_;

    std::queue<Node> q;
    q.push(source);
    std::unordered_set<Node> visited;

    while (!q.empty()) {
      auto p = q.front();
      q.pop();
      ret.push_back(p);

      for (auto u : out[p].to_vector()) {
        if (!util::contains(visited, u)) {
          visited.insert(u);
          q.push(u);
        }
      }
    }
    return ret;
  }

  /**
   * @brief Removes all internal nodes in the circuit that cannot reach the output node.
   *
   * Note: Variable nodes will not be removed.
   */
  void remove_unreachable(std::unordered_set<Node> const& exceptions = {}) {
    std::vector<bool> reachable(node_types_.size() - num_variables_);

    for (auto output_node : get_output_nodes()) {
      if (reachable[output_node - num_variables_]) continue;

      for (auto v : bfs(output_node, true)) {
        if (!is_variable_node(v)) reachable[v - num_variables_] = true;
      }
    }

    for (std::size_t i = num_variables_; i < node_types_.size(); ++i) {
      if (has_node(i) && !reachable[i - num_variables_] && !util::contains(exceptions, i)) {
        // log_trace("Removing node: %lu", i);
        remove_node(i);
      }
    }
  }

  /**
   * @brief Checks if all the output nodes are reachable from any source.
   *
   * @return true all output nodes are reachable
   * @return false there exists an unreachable output node
   */
  bool is_output_reachable() const {
    std::queue<Node> q;
    std::set<Node> reachable;
    std::size_t output_count = 0;

    // Run BFS.
    for (Node i = 0; i < num_variables_; ++i) q.push(i);
    while (!q.empty()) {
      auto p = q.front();
      q.pop();
      for (auto v : out_neighbors_[p]) {
        if (!util::contains(reachable, v)) {
          if (is_output_node(v)) ++output_count;
          reachable.insert(v);
          q.push(v);
        }
      }
    }

    return output_count == num_output_nodes_;
  }

  std::vector<Edge> fingerprint_edges() const {
    std::vector<Edge> ret;
    for (std::size_t i = num_variables_; i < node_types_.size(); ++i) {
      if (node_types_[i] == NodeType::Addition) {
        for (auto j : in_neighbors_[i].to_vector()) ret.push_back(std::make_pair(j, i));
      }
    }
    return ret;
  }

  template <typename T>
  std::vector<T> evaluate(std::vector<T> const& x, std::map<Edge, T> const& fingerprint = {}) const {
    assert(num_variables_ == x.size());

    std::unordered_map<Node, T> values;
    std::size_t num_visited_output_nodes = 0;

    auto vs = topological_ordering(false);
    for (auto i : vs) {
      assert(has_node(i));

      switch (node_types_[i]) {
        case NodeType::Variable: {
          values[i] = x[i];
          // log_trace("Evaluation at var %d: value=%s", i, cstr(x[i]));
          break;
        }
        case NodeType::Addition: {
          assert(!in_neighbors_[i].empty());
          auto in = in_neighbors_[i].to_vector();

          auto v = values[in[0]];
          auto fp0 = fingerprint.find(std::make_pair(in[0], i));
          if (fp0 != fingerprint.end()) v *= fp0->second;

          for (std::size_t j = 1; j < in.size(); ++j) {
            auto fp = fingerprint.find(std::make_pair(in[j], i));
            if (fp == fingerprint.end()) {
              v += values[in[j]];
            } else {
              v += values[in[j]] * fp->second;
            }
          }

          values[i] = v;
          // log_trace("Evaluation at +-node %d: value=%s", i, cstr(v));
          break;
        }
        case NodeType::Multiplication: {
          assert(!in_neighbors_[i].empty());
          auto in = in_neighbors_[i].to_vector();

          auto v = values[in[0]];
          for (std::size_t j = 1; j < in.size(); ++j) v *= values[in[j]];
          values[i] = v;
          // log_trace("Evaluation at *-node %d: value=%s", i, cstr(v));
          break;
        }
        default: {
          // do nothing
        }
      }
      if (is_output_node(i)) {
        if (number_of_output_nodes() == ++num_visited_output_nodes) break;
      }
    }

    std::vector<T> ret;
    for (auto output_node : get_output_nodes()) ret.push_back(values[output_node]);
    return ret;
  }

  /**
   * @brief Returns the maximum degree of output polynomials.
   *
   * @return int the maximum degree of output polynomials
   */
  int degree() const {
    int ret = 0;
    std::vector<ring::TropicalSemiring> x(number_of_variables(), 1);
    for (auto y : evaluate(x)) ret = std::max(ret, y.value);
    return ret;
  }

  /**
   * @brief Returns a list of nodes.
   *
   * @return std::vector<std::pair<Node, NodeType>> node list
   */
  std::vector<std::pair<Node, NodeType>> nodes() const {
    std::vector<std::pair<Node, NodeType>> ret;
    for (std::size_t i = 0; i < node_types_.size(); ++i) {
      if (node_types_[i] != NodeType::Removed) ret.push_back(std::make_pair(i, node_types_[i]));
    }
    return ret;
  }

  /**
   * @brief Returns a list of edges.
   *
   * @return std::vector<Edge> edge list
   */
  std::vector<Edge> edges() const {
    std::vector<Edge> ret;
    for (std::size_t i = 0; i < node_types_.size(); ++i) {
      if (node_types_[i] != NodeType::Removed) {
        for (auto j : out_neighbors_[i].to_vector()) ret.push_back(std::make_pair(i, j));
      }
    }
    return ret;
  }

 private:
  std::size_t num_variables_;             // number of variables; immutable
  std::size_t num_addition_gates_;        // number of addition gates
  std::size_t num_multiplication_gates_;  // number of multiplication gates
  std::size_t num_output_nodes_;          // number of output nodes
  std::size_t num_edges_;                 // number of edges
  std::vector<Set> in_neighbors_;
  std::vector<Set> out_neighbors_;
  std::vector<NodeType> node_types_;
  std::vector<bool> is_output_;  // true if output node
  std::vector<Node> removed_;    // removed nodes

  Node add_node(NodeType type, bool set_as_output) {
    Node ret = in_neighbors_.size();
    if (removed_.empty()) {  // all nodes are in use
      in_neighbors_.push_back(Set());
      out_neighbors_.push_back(Set());
      node_types_.push_back(type);
      is_output_.push_back(set_as_output);
    } else {
      ret = removed_.back();
      removed_.pop_back();  // reuse one of the removed nodes
      node_types_[ret] = type;
      is_output_[ret] = set_as_output;
    }

    if (set_as_output) ++num_output_nodes_;
    return ret;
  }

  Node add_gate(NodeType type, std::vector<Node> const& in_neighbors, bool set_as_output = true) {
    auto v = add_node(type, set_as_output);
    for (auto u : in_neighbors) add_edge_(u, v);
    return v;
  }

  void add_edge_(Node u, Node v) {
    in_neighbors_[v].set(u);
    out_neighbors_[u].set(v);
    ++num_edges_;
  }
};
}  // namespace algebra
