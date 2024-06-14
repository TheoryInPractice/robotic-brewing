#pragma once

#include <atomic>
#include <cmath>

#include "algorithm/graph/connectivity.hpp"
#include "algorithm/graph/shortest_path.hpp"
#include "ds/map/Bimap.hpp"
#include "ds/set/ArrayBitset.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

namespace algorithm {
namespace base {
class BaseSolver {
 public:
  using Vertex = ds::graph::Graph::Vertex;
  using Edge = ds::graph::Graph::Edge;
  using Weight = ds::graph::Graph::Weight;
  using Color = ds::graph::Graph::Color;
  using ColorSet = ds::graph::Graph::ColorSet;
  using CompactColorSet = ds::set::ArrayBitset;

 private:
  char const* solver_name_;
  ds::graph::Graph original_graph_;
  bool print_solution_;
  bool print_certificate_;
  ds::graph::Graph graph_;
  std::unordered_map<Vertex, Vertex> labels_;

 private:
  /** Stores the last solution. */
  std::vector<Vertex> solution_;
  ds::graph::Graph::Weight solution_weight_;
  ds::graph::Graph::ColorSet solution_colors_;

  /** Stores the best walk length for each coverage. */
  std::map<int, Weight> best_weights_;

  bool is_feasible_;
  util::Timer timer_;

  /** Compact color representation. */
  ds::map::Bimap<std::size_t, Color> compact_color_mapping_;
  std::unordered_map<Vertex, CompactColorSet> compact_colors_;

 public:
  /**
   * @brief Constructs a new BaseSolver instance.
   *
   * @param solver_name solver description
   * @param graph original graph
   * @param print_solution if true, print solutions every time a solver finds a feasible solution
   * @param print_certificate if true, print trails every time a solver finds a feasible solution
   */
  BaseSolver(char const* solver_name, ds::graph::Graph const& graph, bool print_solution = false, bool print_certificate = false)
      : solver_name_(solver_name),
        original_graph_(graph),
        print_solution_(print_solution),
        print_certificate_(print_certificate),
        graph_(graph),
        solution_weight_(INFINITY),
        is_feasible_(true),
        timer_() {
    clean();
  }

  virtual ~BaseSolver() = default;

  /**
   * @brief Cleans and resets the environment.
   */
  void clean() {
    graph_ = original_graph_;

    // populate labels
    labels_.clear();
    for (auto v : graph_.vertices()) labels_[v] = v;

    solution_.clear();
    solution_colors_.clear();
    solution_weight_ = INFINITY;
    best_weights_.clear();
    is_feasible_ = true;
  }

  /**
   * @brief Solves the Inspection Planning problem.
   * Result will be stored as the fields `solution_`, `solution_weight_`, `solution_colors_`,
   * and `is_feasible_`.
   *
   * @param k number of colors to collect
   * @param source starting vertex
   * @param destination ending vertex (can be same as source)
   * @param time_limit time limit in seconds (0: no time limit)
   *
   * @return true finished solving the problem (can be either feasible or infeasible)
   * @return false solver timed out
   */
  virtual bool solve(int k, Vertex source, Vertex destination, int time_limit) = 0;

  //==================================================================================================
  //    Accessors
  //==================================================================================================
  /**
   * @brief Gets the solver name
   *
   * @return char const* solver name
   */
  char const* get_solver_name() const { return solver_name_; }

  /**
   * @brief Gets the current graph.
   *
   * @return ds::graph::Graph current graph
   */
  ds::graph::Graph const& get_graph() const { return graph_; }
  ds::graph::Graph& get_graph() { return graph_; }

  /**
   * @brief Gets the original graph.
   *
   * @return ds::graph::Graph original graph
   */
  ds::graph::Graph const& get_original_graph() const { return original_graph_; }

  /**
   * @brief Checks if the solver has a solution.
   *
   * @return true solver has a solution
   * @return false solver does not have s solution
   */
  bool has_solution() const { return !solution_.empty(); }

  /**
   * @brief Clears current solutions.
   */
  void clear_solution() {
    solution_.clear();
    solution_colors_.clear();
    solution_weight_ = INFINITY;
    best_weights_.clear();
    is_feasible_ = true;
  }

  /**
   * @brief Gets the solution walk.
   *
   * @return std::vector<Vertex> solution walk
   */
  std::vector<Vertex> get_solution() const { return solution_; }

  /**
   * @brief Gets the solution weight.
   *
   * @return ds::graph::Graph::Weight solution weight (objective)
   */
  ds::graph::Graph::Weight get_solution_weight() const { return solution_weight_; }

  /**
   * @brief Gets the solution colors
   *
   * @return ds::graph::Graph::ColorSet solution color set
   */
  ds::graph::Graph::ColorSet get_solution_colors() const { return solution_colors_; }

  /**
   * @brief Returns if there is a solution.
   *
   * @return true problem instance is feasible
   * @return false problem instance is infeasible
   */
  bool is_feasible() const { return is_feasible_; }

  //==================================================================================================
  //    Utilities
  //==================================================================================================

  /**
   * @brief Removes the colors collected at the given terminals from the entire graph.
   *
   * @param terminals list of vertices
   * @return std::size_t number of colors collected at terminals
   */
  std::size_t remove_terminal_colors(std::vector<ds::graph::Graph::Vertex> const& terminals) {
    ds::graph::Graph::ColorSet colors;

    // collect terminal colors
    for (auto v : terminals) colors |= graph_.get_colors(v);

    // remove collected colors from the graph
    if (!colors.empty()) {
      for (auto v : graph_.vertices()) graph_.set_colors(v, graph_.get_colors(v) - colors);
    }
    return colors.size();
  }

  /**
   * @brief Splits every multi-colored vertex into distinct vertices with a single color.
   *
   * @param duplicate_edges if true, duplciate all edges indicent to the original vertex
   */
  void split_colors(bool duplicate_edges = false) {
    Vertex top_label = 0;
    for (auto v : graph_.vertices()) top_label = std::max(top_label, v);

    // not necessary to sort, but this will lead to a determinitstic construction
    for (auto v : graph_.vertices(true)) {
      auto colors = graph_.get_colors(v).to_vector();
      if (colors.size() > 1) {
        graph_.set_colors(v, ds::graph::Graph::ColorSet({colors[0]}));

        for (std::size_t i = 1; i < colors.size(); ++i) {
          auto u = ++top_label;
          labels_[u] = v;  // original vertex
          graph_.add_vertex(u, {colors[i]});

          if (duplicate_edges) {
            for (auto& w : graph_.neighbors(v)) { graph_.add_edge(u, w, graph_.get_weight(v, w)); }
          }

          graph_.add_edge(u, v, 0);
        }
      }
    }

    log_info("%s: Split multi-colored vertices to single-colored ones.", solver_name_);
  }

  /**
   * @brief Removes all vertices unreachable from the source vertex.
   *
   * @param source source vertex
   * @return number of vertices that have been removed
   */
  int remove_unreachable_vertices(Vertex source) {
    auto cc = algorithm::graph::connected_component(graph_, source);
    std::unordered_set<Vertex> to_keep(cc.begin(), cc.end());
    int ret = graph_.number_of_vertices() - cc.size();

    for (auto v : graph_.vertices()) {
      if (!util::contains(to_keep, v)) {
        labels_.erase(v);
        graph_.remove_vertex(v);
      }
    }
    return ret;
  }

  /**
   * @brief Repeatedly smoothes vertices without colors and degree at most 2.
   *
   * @param exceptions vertices to keep
   * @return number of vertices that have been removed
   */
  int smooth_colorless_vertices(std::vector<Vertex> const& exceptions = {});

  /**
   * @brief Remove vertices without colors.
   *
   * @param exceptions vertices that must be kept
   */
  void remove_colorless_vertices(std::vector<Vertex> const& exceptions) {
    std::vector<Vertex> to_remove;
    for (auto v : graph_.vertices()) {
      if (util::contains(exceptions, v)) continue;
      if (graph_.get_colors(v).empty()) to_remove.push_back(v);
    }
    for (auto v : to_remove) {
      labels_.erase(v);
      graph_.remove_vertex(v);
    }

    if (!to_remove.empty()) {
      log_info("%s: Removed vertices with no colors: num_removed=%lu, n=%lu, m=%lu", solver_name_, to_remove.size(),
               graph_.number_of_vertices(), graph_.number_of_edges());
    }
  }

  /**
   * @brief Creates the transitive closure of the current graph.
   *
   * @param create_metric if true, updates edge weights to the shortest distance between
   *                      the endpoints, thus making a metric cluster graph.
   * @param cancel_token cancelation token
   */
  void create_transitive_closure(bool create_metric = false, std::atomic_bool* cancel_token = nullptr) {
    auto n = graph_.number_of_vertices();
    auto m = graph_.number_of_edges();

    if (!create_metric && m * 2 == n * (n - 1)) return;  // already a complete graph

    auto length = algorithm::graph::all_pairs_dijkstra_path_length(graph_, {}, cancel_token);
    auto vs = graph_.vertices();
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        auto u = vs[i];
        auto v = vs[j];
        auto key = graph_.to_edgekey(u, v);
        if (!util::contains(length, key)) continue;  // u and v are unreachable

        if (!graph_.has_edge(u, v)) {
          graph_.add_edge(u, v, length.at(key));
        } else if (create_metric) {
          graph_.set_weight(u, v, length.at(key));
        }
      }
    }

    log_info("%s: Created transitive closure (with%s updated weights).", solver_name_, create_metric ? "" : "out");
  }

  /**
   * @brief Rounds the current graph's weights to integers.
   */
  void round_weights() {
    int lo = -1, hi = 0;
    for (auto& e : graph_.edges()) {
      int w = std::round(graph_.get_weight(e.first, e.second));
      graph_.set_weight(e.first, e.second, w);

      lo = lo < 0 ? w : std::min(lo, w);
      hi = std::max(hi, w);
    }
    log_info("%s: Rounded weights to nearest integers: min=%d, max=%d", solver_name_, lo, hi);
  }

  /**
   * @brief Compresses the color set present in the graph and
   *        creates a vertex-color map using a compact data structure.
   */
  void compact_colors() {
    compact_colors_.clear();
    auto all_colors = graph_.get_colors().to_vector();
    auto num_colors = all_colors.size();
    compact_color_mapping_ = ds::map::Bimap<std::size_t, Color>(all_colors);

    for (auto v : graph_.vertices()) {
      std::vector<std::size_t> cs;
      for (auto c : graph_.get_colors(v)) cs.push_back(compact_color_mapping_.g(c));
      compact_colors_.emplace(v, CompactColorSet(num_colors, cs));
    }
    log_info("%s: Compacted the current color set: |C|=%lu", solver_name_, num_colors);
  }

  /**
   * @brief Obtains the compact representation of the color set at the given vertex.
   *
   * @param v vertex
   * @return CompactColorSet const& compact color set
   */
  CompactColorSet const& get_compact_colors(Vertex v) const { return compact_colors_.at(v); }

  CompactColorSet get_compact_colors() const { return ~CompactColorSet(compact_color_mapping_.size()); }

  //==================================================================================================
  //    Common preprocessing
  //==================================================================================================

  /**
   * @brief Finds a solution using the edges in a minimum spanning tree.
   *
   * @param source starting vertex
   * @param destination ending vertex (can be same as source)
   *
   * @return std::vector<Vertex> walk visiting all vertices
   */
  std::vector<Vertex> find_mst_solution(Vertex source, Vertex destination) const;

  /**
   * @brief Finds a list of vertices that collect k colors nearest to either source or destination.
   *
   * @param k number of colors to collect
   * @param source starting vertex
   * @param destination ending vertex
   * @param rand pointer to util::Random instance or nullptr (no randomization)
   * @return std::vector<Vertex> list of terminals (including source and destination)
   */
  std::vector<Vertex> find_nearest_terminals(int k, Vertex source, Vertex destination, util::Random* rand = nullptr) const;

  /**
   * @brief Finds a solution using the edges in an (approximate) Steiner tree with the nearest terminals.
   *
   * @param k number of colors to collect
   * @param source starting vertex
   * @param destination ending vertex
   * @param rand pointer to util::Random instance or nullptr (no randomization)
   * @param rand util::Random instance
   * @return solution weight
   */
  double find_steiner_tree_solution(int k, Vertex source, Vertex destination, util::Random* rand = nullptr);

  /**
   * @brief Performs common preprocessing steps.
   *
   * @param k number of colors to collect
   * @param source starting vertex
   * @param destination ending vertex (can be same as source)
   *
   * @return true reduced to a trivial instance
   * @return false did not reduce to a trivial instance
   */
  bool preprocess(int& k, Vertex source, Vertex destination) {
    clean();
    if (!get_graph().has_vertex(source)) throw std::invalid_argument("source vertex does not exist");

    int num_removed = remove_unreachable_vertices(source);
    if (num_removed > 0) {
      log_info("%s: Removed the vertices unreachable from source: num_removed=%d", solver_name_, num_removed);
    }

    std::vector<Vertex> terminals = {source, destination};
    int num_terminal_colors = remove_terminal_colors(terminals);
    if (num_terminal_colors >= k) {
      log_info("%s: Reducible instance: #terminal_colors=%d >= k", solver_name_, num_terminal_colors);
      if (source == destination) {
        set_solution({source});
      } else {
        set_critical_walk({source, destination});
      }
      return true;
    }

    if (num_terminal_colors > 0) {
      k -= num_terminal_colors;  // update k
      log_info("%s: Removed terminal colors: #terminal_colors=%d, new_k=%d", solver_name_, num_terminal_colors, k);
    }

    int c = get_graph().number_of_colors();  // number of total colors
    if (c < k) {
      log_info("%s: Infeasible instance: #collectable_colors=%d < k", solver_name_, c);
      is_feasible_ = false;
      return true;
    }

    // Smooth colorless vertices.
    num_removed = smooth_colorless_vertices(terminals);
    if (num_removed > 0) {
      log_info("%s: Smoothed vertices with no colors: num_removed=%d", solver_name_, num_removed);
    }

    // Find heuristic solutions.
    // util::Timer mst_timer;
    // set_solution(find_mst_solution(source, destination));
    // log_info("%s: Found an MST-based solution: objective=%.10f, #colors=%lu, #hops=%lu, s=%d, t=%d, elapsed=%.3fs",
    //          solver_name_, get_solution_weight(), get_solution_colors().size(), get_solution().size(), source,
    //          destination, mst_timer.stop());

    util::Timer steiner_timer;
    find_steiner_tree_solution(k, source, destination);
    log_info("%s: Found an ST-based solution: objective=%.10f, #colors=%lu, #hops=%lu, s=%d, t=%d, elapsed=%.3fs",
             solver_name_, get_solution_weight(), get_solution_colors().size(), get_solution().size(), source,
             destination, steiner_timer.stop());

    log_info("%s: Finished common preprocessing: n=%lu, m=%lu, |C|=%d", solver_name_, get_graph().number_of_vertices(),
             get_graph().number_of_edges(), c);
    return false;
  }

  //==================================================================================================
  //    Updating solution
  //==================================================================================================
  /**
   * @brief Converts a walk on the augmented graph into a walk on the original graph.
   * Set the walk on the original graph as a solution.
   *
   * @param walk walk on the augmented graph
   * @return solution weight
   */
  double set_critical_walk(std::vector<Vertex> walk) {
    if (walk.empty()) throw std::invalid_argument("walk cannot be empty");

    auto num_hops = walk.size();

    std::vector<Vertex> xs;
    for (std::size_t i = 0; i < num_hops - 1; ++i) {
      auto u = labels_.at(walk[i]);
      auto v = labels_.at(walk[i + 1]);

      for (auto x : algorithm::graph::shortest_path(original_graph_, u, v)) {
        if (xs.empty() || xs.back() != x) xs.push_back(x);
      }
    }

    return set_solution(xs);
  }

 private:
  /**
   * @brief Sets the given walk as a solution.
   * Updates the set of collected colors and total weight.
   *
   * @param walk walk from source to destination
   * @return solution weight
   */
  double set_solution(std::vector<Vertex> const& walk) {
    if (walk.empty()) throw std::invalid_argument("walk cannot be empty");

    // collect information
    int num_hops = walk.size();
    ColorSet collected_colors;
    for (auto v : walk) collected_colors |= original_graph_.get_colors(v);
    int num_collected_colors = collected_colors.size();

    double weight = 0.0;
    for (int i = 0; i < num_hops - 1; ++i) weight += original_graph_.get_weight(walk[i], walk[i + 1]);

    // check if the weight improves for the coverage
    auto it = best_weights_.lower_bound(num_collected_colors);
    if (it != best_weights_.end() && it->second <= weight) return weight;

    // update solution
    solution_ = walk;
    solution_colors_ = collected_colors;
    solution_weight_ = weight;
    best_weights_[num_collected_colors] = weight;

    // print solution
    if (print_solution_) {
      printf("%.3f %d %.10f %d", timer_.stop(), num_collected_colors, solution_weight_, num_hops);
      if (print_certificate_) {
        for (auto v : walk) printf(" %d", v);
      }
      printf("\n");
      fflush(stdout);  // do not buffer
    }
    return weight;
  }
};
}  // namespace base
}  // namespace algorithm
