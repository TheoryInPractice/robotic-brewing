#pragma once

#include <algorithm>
#include <atomic>
#include <future>

#include "algebra/MultilinearDetector.hpp"
#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/graph/spanning_tree.hpp"
#include "algorithm/ilp/ILPRunner.hpp"

namespace algorithm {
namespace exact {

namespace algebraic {
/** Search strategy. */
enum SearchStrategy { Basic, Probabilistic, Sequential, MultiOutput };

enum CompactStrategy { Naive, Standard, SemiCompact, Compact };

enum RecoveryStrategy { MC, LV, TP, None };
}  // namespace algebraic

class AlgebraicSolver : public base::BaseSolver {
 private:
  util::Random &rand_;
  int num_threads_;
  int num_confident_failures_;
  bool recover_all_;
  algebraic::SearchStrategy search_strategy_;

  /**
   * @brief Circuit compact level.
   *
   * 0: Original idea: split all colors to distinct vertices.
   * 1: Standard
   * 2: Semi-compact: larger but more effective circuit than fully compact; does not split colors.
   * 3: Fully compact: asymptotically the smallest circuit; may be slow when each vertex has few colors.
   */
  algebraic::CompactStrategy compact_level_;

  /**
   * @brief Solution Recovery Strategy.
   *
   * MC: Monte Carlo
   * LV: Las Vegas
   */
  algebraic::RecoveryStrategy recovery_strategy_;

 public:
  using Circuit = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>;

  AlgebraicSolver(ds::graph::Graph const &graph, util::Random &rand, int num_threads, int num_confident_failures = 30,
                  bool recover_all = false, algebraic::SearchStrategy search_strategy = algebraic::SearchStrategy::Probabilistic,
                  algebraic::CompactStrategy compact_strategy = algebraic::CompactStrategy::SemiCompact,
                  algebraic::RecoveryStrategy recovery_strategy = algebraic::RecoveryStrategy::LV,
                  bool print_solution = false, bool print_certificate = false)
      : base::BaseSolver("AlgebraicSolver", graph, print_solution, print_certificate),
        rand_(rand),
        num_threads_(num_threads),
        num_confident_failures_(num_confident_failures),
        recover_all_(recover_all),
        search_strategy_(search_strategy),
        compact_level_(compact_strategy),
        recovery_strategy_(recovery_strategy) {
    if (recovery_strategy == algebraic::RecoveryStrategy::TP) {
      if (compact_strategy == algebraic::CompactStrategy::Naive || compact_strategy == algebraic::CompactStrategy::Standard) {
        std::string msg = "Two phase recovery does not work with the following compact strategies: Naive, Standard";
        throw std::invalid_argument(msg);
      }
    }

    log_info("%s: Initialized: strategy=%d, #confident=%d, compact=%d", get_solver_name(), search_strategy_,
             num_confident_failures, compact_strategy);
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

  /**
   * @brief Standard binary search.
   *
   * @param k
   * @param source
   * @param destination
   * @param c
   * @param lo
   * @param hi
   * @param cancel_token
   * @return int
   */
  int run_standard_binary_search(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token);

  /**
   * @brief Probabilistic binary search (deprecated).
   *
   * @param k
   * @param source
   * @param destination
   * @param c
   * @param lo
   * @param hi
   * @param cancel_token
   * @return int
   */
  int run_probabilistic_binary_search_naive(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token);

  /**
   * @brief Probabilistic binary search.
   *
   * @param k
   * @param source
   * @param destination
   * @param c
   * @param lo
   * @param hi
   * @param cancel_token
   * @return int
   */
  int run_probabilistic_binary_search(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token);

  int run_sequential_search(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token);

  int run_multi_output_search(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token);

  bool test_walk_length(int k, int s, int t, int c, int lb, int ub, int num_trials, std::atomic_bool *cancel_token);

  void find_certificate(int k, int s, int t, int c, int walk_length, int num_trials, std::atomic_bool *cancel_token);
};
}  // namespace exact
}  // namespace algorithm
