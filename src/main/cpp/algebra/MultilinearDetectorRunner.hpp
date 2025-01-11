#pragma once

#include <atomic>
#include <cmath>
#include <thread>

#include <omp.h>  // parallel processing

#include "algebra/ArithmeticCircuit.hpp"
#include "algebra/FinitePoly.hpp"
#include "ds/set/SortedVectorSet.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

namespace algebra {
class MultilinearDetectorRunner {
 public:
  static int const MAX_K_VALUE = 62;

 private:
  typedef ArithmeticCircuit<ds::set::SortedVectorSet> Circuit;
  typedef Circuit::Node Node;
  typedef Circuit::Edge Edge;
  typedef uint64_t T;
  typedef algebra::FinitePoly<T> P;

  Circuit const& circuit_;  // arithmetic circuit
  int const n_;             // number of variables
  int const k_;             // degree upper bound
  int const num_threads_;   // number of threads
  int const ell_;           // parameter for fingerprint size

 private:
  int default_ell(Circuit const& circuit) {
    return 3 + static_cast<int>(std::ceil(std::log2(std::max(1, circuit.degree()))));
  }

 public:
  MultilinearDetectorRunner(Circuit const& circuit, int k, int num_threads = 1, int ell = -1)
      : circuit_(circuit),
        n_(circuit_.number_of_variables()),
        k_(std::min(n_, k)),  // degree is at most the number of variables
        num_threads_(num_threads),
        ell_(ell < 0 ? default_ell(circuit) : ell) {
    if (k < 1 || MAX_K_VALUE < k) {
      throw std::invalid_argument(util::format("k must be between 1 and %d, inclusive", MAX_K_VALUE));
    }
    if (num_threads_ < 1) throw std::invalid_argument("num_threads must be at least 1");
  }

  std::vector<bool> run(util::Random& rand, std::atomic_bool* cancel_token = nullptr) {
    util::Timer timer;
    log_debug("MLD: Started single run: #nodes=%lu, #edges=%lu, #vars=%d, k=%d, ell=%d, #threads=%d",
              circuit_.number_of_nodes(), circuit_.number_of_edges(), n_, k_, ell_, num_threads_);

    auto ret = run_iteration(rand, cancel_token);

    log_debug("MLD: Finished single run: result=%s, elapsed=%.3fs", cstr(ret), timer.stop());
    return ret;
  }

 private:
  /**
   * @brief Creates a random number in [1, 2^k).
   *
   * @param rand util::Random instance
   * @return P random number
   */
  T create_random_constant(util::Random& rand) const { return rand.randint<T>(1ULL, (1ULL << k_) - 1); }

  /**
   * @brief Creates a random polynomial of degree (ell-1) whose coefficients are 0 or 1.
   *
   * @param rand util::Random instance
   * @return P random polynomial
   */
  P create_random_polynomial(util::Random& rand) const {
    T rand_val = rand.randint<T>(1ULL, (1ULL << ell_) - 1);
    std::vector<T> coef;
    for (int i = 0; i < ell_; ++i) coef.push_back((rand_val >> i) & 1);
    return P(ell_, coef);
  }

  std::vector<bool> run_iteration(util::Random& rand, std::atomic_bool* cancel_token = nullptr) const {
    // Create random values for variables.
    std::vector<T> x;
    for (int i = 0; i < n_; ++i) x.push_back(create_random_constant(rand));

    // Create random fingerprints for all in-edges to addition gates.
    auto fp_edges = circuit_.fingerprint_edges();
    std::map<Edge, P> fingerprint;
    for (auto& e : fp_edges) fingerprint[e] = create_random_polynomial(rand);

    // Evaluate the circuit.
    auto ret = run_evaluation(x, fingerprint, cancel_token);
    return ret;
  }

  /**
   * @brief Returns true if there exists a nonzero coefficient when taking mod 2^(k+1).
   *
   * @param poly polynomial
   * @return true there exists a nonzero coefficient mod 2^(k+1)
   * @return false no nonzero coefficients mod 2^(k+1)
   */
  bool is_nonzero_poly(P const& poly) const {
    uint64_t mask = (1ULL << (k_ + 1)) - 1;
    for (auto c : poly.coefficients()) {
      if (c & mask) return true;
    }
    return false;
  }

  std::vector<bool> run_evaluation(std::vector<T> const& x, std::map<Edge, P> const& fingerprint,
                                   std::atomic_bool* cancel_token = nullptr) const {
    int w = circuit_.number_of_output_nodes();
    std::vector<P> result(w, ell_);
    log_trace("MLD: evaluating: k=%d, x=%s, fingerprint=%s", k_, cstr(x), cstr(fingerprint));

#pragma omp parallel num_threads(num_threads_)
    {
      std::vector<P> local_result(w, ell_);

#pragma omp for schedule(static)
      for (uint64_t j = 0; j < (1ULL << k_); ++j) {
        if (!cancel_token || !cancel_token->load()) {
          // Construct z(x, j)
          std::vector<P> z;
          for (int i = 0; i < n_; ++i) {
            int oddity = __builtin_popcountll(x[i] & j) % 2;
            z.push_back(P(ell_, oddity ? 0 : 2));
          }

          // Evaluate the circuit.
          auto evaluated = circuit_.evaluate(z, fingerprint);

          // Add to the overall result.
          for (int i = 0; i < w; ++i) {
            if (evaluated[i].is_valid()) local_result[i] += evaluated[i];
          }
          // log_trace("evaluated: %s", cstr(evaluated));
          // log_trace("local result updated: %s", cstr(local_result));
        }
      }

#pragma omp critical
      for (int i = 0; i < w; ++i) result[i] += local_result[i];
    }

    if (cancel_token && cancel_token->load()) {
      throw std::runtime_error("canceled: MultilinearDetectorRunner::run_evaluation()");
    }

    log_trace("MLD: final result: %s", cstr(result));

    // Determine if there exists a nonzero coefficient when taking mod 2^(k+1).
    std::vector<bool> ret;
    for (int i = 0; i < w; ++i) ret.push_back(is_nonzero_poly(result[i]));
    return ret;
  }
};
}  // namespace algebra
