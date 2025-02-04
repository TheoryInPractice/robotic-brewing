#pragma once

#include <atomic>
#include <cmath>
#include <thread>

#include <omp.h>  // parallel processing

#include "algebra/ArithmeticCircuit.hpp"
#include "algebra/FinitePoly.hpp"
#include "algebra/MultilinearDetectorRunner.hpp"
#include "ds/set/SortedVectorSet.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

namespace algebra {
class MultilinearDetector {
 public:
  static int const MAX_K_VALUE = 62;

 private:
  typedef ArithmeticCircuit<ds::set::SortedVectorSet> Circuit;
  typedef Circuit::Node Node;
  typedef Circuit::Edge Edge;
  typedef uint64_t T;
  typedef algebra::FinitePoly<T> P;

  Circuit circuit_;        // arithmetic circuit
  int const n_;            // number of variables
  int const k_;            // degree upper bound
  int const num_threads_;  // number of threads
  int const ell_;          // parameter for fingerprint size
  int num_total_;          // number of total tries
  int num_successes_;      // number of successes

 private:
  int default_ell(Circuit const& circuit) {
    return 3 + static_cast<int>(std::ceil(std::log2(std::max(1, circuit.degree()))));
  }

 public:
  MultilinearDetector(Circuit const& circuit, int k, int num_threads = 1, int ell = -1)
      : circuit_(circuit),
        n_(circuit_.number_of_variables()),
        k_(std::min(n_, k)),  // degree is at most the number of variables
        num_threads_(num_threads),
        ell_(ell < 0 ? default_ell(circuit) : ell),
        num_total_(0),
        num_successes_(0) {
    if (k < 1 || MAX_K_VALUE < k) {
      throw std::invalid_argument(util::format("k must be between 1 and %d, inclusive", MAX_K_VALUE));
    }
    if (num_threads_ < 1) throw std::invalid_argument("num_threads must be at least 1");

    // remove redundant nodes in the circuit
    circuit_.remove_unreachable();
    log_debug("MultilinearDetector: Initialized: k=%d, #nodes=%lu (+:%lu, *:%lu), #edges=%lu", k_, circuit_.number_of_nodes(),
              circuit_.number_of_addition_gates(), circuit_.number_of_multiplication_gates(), circuit_.number_of_edges());
  }

  bool run(util::Random& rand, int num_iterations = 30, bool early_return = true, std::atomic_bool* cancel_token = nullptr) {
    log_debug("MultilinearDetector: Started: n=%d, #nodes=%lu, #edges=%lu, k=%d, ell=%d, #threads=%d, #iterations=%d",
              n_, circuit_.number_of_nodes(), circuit_.number_of_edges(), k_, ell_, num_threads_, num_iterations);

    if (circuit_.number_of_output_nodes() != 1) throw std::invalid_argument("circuit must contain one output node");

    bool success = false;
    util::Timer timer;
    MultilinearDetectorRunner det(circuit_, k_, num_threads_, ell_);
    int num_total = 0, num_successes = 0;

    for (int t = 0; t < num_iterations; ++t) {
      auto ret = det.run(rand, cancel_token);
      ++num_total;
      if (ret.front()) {  // check only the first output node
        ++num_successes;
        success = true;
        if (early_return) break;
      }
    }

    auto elapsed = timer.stop();
    num_total_ += num_total;
    num_successes_ += num_successes;

    log_debug("MultilinearDetector: Finished: total=%d, success=%d, elapsed=%.3fs, per_run=%.3fs", num_total,
              num_successes, elapsed, elapsed / num_total_);
    return success;
  }

  std::vector<bool> run_single(util::Random& rand, std::atomic_bool* cancel_token = nullptr) {
    util::Timer timer;
    log_debug("MultilinearDetector: Started single run: #nodes=%lu, #edges=%lu, #vars=%d, k=%d, ell=%d, #threads=%d",
              circuit_.number_of_nodes(), circuit_.number_of_edges(), n_, k_, ell_, num_threads_);

    MultilinearDetectorRunner det(circuit_, k_, num_threads_, ell_);
    auto ret = det.run(rand, cancel_token);

    log_debug("MultilinearDetector: Finished single run: elapsed=%.3fs", timer.stop());
    return ret;
  }

  /**
   * @brief Finds a tree certificate for the circuit.
   *
   * @param rand util::Random instance
   * @param exceptions do not choose an in-neighbor on these addition gates
   * @param cancel_token cancelation token
   * @param use_las_vegas to use Las Vegas solution recovery strategy
   * @param monte_carlo_num_iterations number of iterations to run Multilinear Detector
   *
   * @return Circuit tree certificate
   */
  Circuit find_certificate(util::Random& rand, std::unordered_set<int> const& exceptions = {},
                           std::atomic_bool* cancel_token = nullptr, bool use_las_vegas = true,
                           int monte_carlo_num_iterations = 30) {
    // Copy the given circuit and remove nodes unreachable to the output node.
    Circuit C = circuit_;

    if (C.number_of_output_nodes() != 1) throw std::invalid_argument("circuit must contain one output node");
    auto output_node = C.get_output_nodes().front();

    // Traversal from the output node.
    auto vs = C.topological_ordering(true);
    for (auto v : vs) {
      if (C.is_variable_node(v)) continue;

      if (v != output_node && C.out_degree(v) == 0) {
        // Remove unlinked nodes from the circuit.
        C.remove_node(v);
        continue;
      }

      if (util::contains(exceptions, v)) continue;

      assert(v == output_node || C.out_degree(v) == 1);
      if (!C.is_addition_node(v)) continue;

      // Select one in-neighbor at an addition gate.
      auto us = C.get_in_neighbors(v);
      std::size_t left = 0, right = us.size();

      // Perform binary search.
      while (right - left > 1) {
        auto mid = (left + right) / 2;

        if (use_las_vegas) {
          auto a = left;
          auto b = mid;

          //--------------------------------------------------------------------
          //    Las Vegas Solution Recovery
          //--------------------------------------------------------------------
          while (true) {
            // Remove half of the in-edges.
            for (auto i = a; i < b; ++i) C.remove_edge(us[i], v);

            // Run one iteration of multilinear detection.
            MultilinearDetectorRunner det(C, k_, num_threads_, ell_);
            auto ret = det.run(rand, cancel_token);

            if (ret.front()) {
              // Success; safe to remove.
              if (a == left) {
                left = b;
              } else {
                right = a;
              }
              break;
            }

            // Failure; restore edges and swap intervals.
            for (auto i = a; i < b; ++i) C.add_edge(us[i], v);

            if (a == left) {
              a = mid;
              b = right;
            } else {
              a = left;
              b = mid;
            }
          }
        } else {
          //--------------------------------------------------------------------
          //    Monte Carlo Solution Recovery
          //--------------------------------------------------------------------
          // Remove half of the in-edges.
          for (auto i = left; i < mid; ++i) C.remove_edge(us[i], v);

          MultilinearDetector det(C, k_, num_threads_, ell_);
          auto ret = det.run(rand, monte_carlo_num_iterations, true, cancel_token);

          if (ret) {
            // Success; safe to remove.
            left = mid;
          } else {
            // Failure; remove the other part.
            for (auto i = left; i < mid; ++i) C.add_edge(us[i], v);
            for (auto i = mid; i < right; ++i) C.remove_edge(us[i], v);
            right = mid;
          }
        }
      }
    }

    return C;
  }
};
}  // namespace algebra
