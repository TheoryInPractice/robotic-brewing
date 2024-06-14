#pragma once

#include <atomic>
#include <future>
#include <optional>

#include "algorithm/base/BaseSolver.hpp"

namespace algorithm {
namespace exact {

/**
 * @brief NaiveDP solver specialized for transitive closures and aimed for more parallelism.
 */
class NaiveDPSolver2 : public base::BaseSolver {
 private:
  int num_threads_;

 public:
  NaiveDPSolver2(ds::graph::Graph const &graph, int num_threads, bool print_solution = false, bool print_certificate = false)
      : base::BaseSolver("NaiveDPSolver2", graph, print_solution, print_certificate), num_threads_(num_threads) {
    log_info("%s: Initialized: num_threads=%d", get_solver_name(), num_threads_);
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
};
}  // namespace exact
}  // namespace algorithm
