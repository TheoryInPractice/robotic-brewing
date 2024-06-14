#pragma once

#include <atomic>
#include <future>
#include <optional>

#include "algorithm/base/BaseSolver.hpp"

// TODO: Implement multithreading

namespace algorithm {
namespace exact {
class NaiveDPSolver : public base::BaseSolver {
 private:
  int num_threads_;
  bool transitive_closure_;

 public:
  NaiveDPSolver(ds::graph::Graph const &graph, int num_threads, bool print_solution = false,
                bool print_certificate = false, bool transitive_closure = false)
      : base::BaseSolver("NaiveDPSolver", graph, print_solution, print_certificate),
        num_threads_(num_threads),
        transitive_closure_(transitive_closure) {
    log_info("%s: Initialized: num_threads=%d, transitive_closure=%s", get_solver_name(), num_threads_,
             transitive_closure_ ? "true" : "false");
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
  bool solve(int k, Vertex source, Vertex destination, int time_limit = 0) override;

 private:
  void solve_main(int k, Vertex source, Vertex destination, std::atomic_bool *cancel_token);

  // void solve_main_recuirsive(int k, int num_colors, Vertex source, Vertex destination, std::atomic_bool *cancel_token);

  /**
   * @brief Recursive implementation.
   *
   * @param source
   * @param v
   * @param colors
   * @return std::pair<Weight, std::vector<Vertex>>
   */
  // Weight solve_recursive(Vertex source, Vertex destination, Vertex v, uint32_t colors, std::atomic_bool *cancel_token);
};
}  // namespace exact
}  // namespace algorithm
