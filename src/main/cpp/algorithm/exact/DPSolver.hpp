#pragma once

#include <atomic>
#include <future>
#include <omp.h>  // parallel processing

#include "algorithm/base/BaseSolver.hpp"
#include "util/Profiler.hpp"

#define USE_INT_COLOR_SET true

namespace algorithm {
namespace exact {
class DPSolver : public base::BaseSolver {
 private:
  int num_threads_;
  double epsilon_;
  double rho_;
  double scaling_factor_;
  bool transitive_closure_;

  /**
   * The DP table has form T[i][v][S] = TE, where
   * i is an int between 0 and n - 2
   *     - In practice this is 0 or 1, as we never need previous layers
   * v is a Vertex
   * S is a ColorSet
   * TE is a TableEntry representing a walk of length i ending at v
   *
   * S (the key) is the ColorSet that this TableEntry ''claims'' to collect
   * Each table entry is a tuple
   * The first entry is the ''claimed'' weight of this walk.
   * The second entry is the ColorSet that this walk actually collects (a subset of S).
   * The third entry is the actual weight of this walk.
   * The fourth entry is a certificate (the walk).
   *
   * We guarantee that every entry is (epsilon, rho) bounded, meaning that the actual weight is at most
   * a (1 + epsilon) factor larger than the claimed weight, and the actual color set has size at least
   * a rho-fraction of the size of S.
   */

#if USE_INT_COLOR_SET
  using ColorSetType = uint32_t;
  std::vector<ColorSetType> vertex_colors_;
#else
  using ColorSetType = CompactColorSet;
#endif
  using TableEntry = std::tuple<Weight, ColorSetType, Weight, std::vector<Vertex>>;
  using TableMap = std::map<ColorSetType, TableEntry>;

  std::vector<TableMap> table_[2];

 public:
  DPSolver(ds::graph::Graph const& graph, int num_threads, bool print_solution = false, bool print_certificate = false,
           double epsilon = 0, double rho = 1, double scaling_factor = 0, bool transitive_closure = false)
      : base::BaseSolver("DPSolver", graph, print_solution, print_certificate),
        num_threads_(num_threads),
        epsilon_(epsilon),
        rho_(rho),
        scaling_factor_(scaling_factor),
        transitive_closure_(transitive_closure) {
    log_info("%s: Initialized: num_threads=%d", get_solver_name(), num_threads);
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
  bool solve(int k, Vertex source, Vertex destination, int time_limit = 0) override {
#if USE_INT_COLOR_SET
    if (k > 32) { throw std::invalid_argument(util::format("k must be between 1 and %d, inclusive", 32)); }
#endif
    if (time_limit > 0) {
      std::atomic_bool cancel_token(false);
      std::future<void> future = std::async(&DPSolver::solve_main, this, k, source, destination, &cancel_token);
      if (future.wait_for(std::chrono::seconds(time_limit)) == std::future_status::timeout) {
        cancel_token.store(true);  // cancel the task

        char const* failure_message = "";
        try {
          future.get();
        } catch (std::exception const& e) {  //
          failure_message = e.what();
        }

        log_warning("%s: timed out after %d seconds: %s", get_solver_name(), time_limit, failure_message);
        return false;
      }
      future.get();  // may throw exceptions
    } else {
      solve_main(k, source, destination, nullptr);
    }
    return true;
  }

  void solve_main(int k, Vertex source, Vertex destination, std::atomic_bool* cancel_token) {
    //--------------------------------------------------------------------------
    // 1. Preprocessing
    //--------------------------------------------------------------------------
    if (preprocess(k, source, destination)) return;

    // re-index colors
    compact_colors();

#if USE_INT_COLOR_SET
    // initialize color table
    auto vs = get_graph().vertices();
    vertex_colors_.clear();
    vertex_colors_.resize(*std::max_element(vs.begin(), vs.end()) + 1);
    for (auto v : vs) {
      ColorSetType colors = 0;
      for (auto c : get_compact_colors(v)) colors |= 1U << c;
      vertex_colors_[v] = colors;
    }
#endif

    log_info("%s: Finished preprocessing: n=%lu, m=%lu, |C|=%lu", get_solver_name(), get_graph().number_of_vertices(),
             get_graph().number_of_edges(), get_graph().number_of_colors());

    // if flagged, create transitive closure and remove colorless vertices
    if (transitive_closure_) {
      log_info("%s: Creating transitive closure", get_solver_name());
      create_transitive_closure(true);
      remove_colorless_vertices({source, destination});
    }

    //--------------------------------------------------------------------------
    // 2. Main Computation
    //--------------------------------------------------------------------------
    if (!cancel_token && (epsilon_ > 0 || rho_ < 1) && scaling_factor_ != 0) {
      throw std::runtime_error(
          "Iterative parameter tightening with no timeout will result in unbounded execution time (DPSolver)");
    }

    while (true) {
      solve_approximate_dominance(k, source, destination, cancel_token);

      // tighten the parameters and recurse
      if (scaling_factor_ == 0 || (epsilon_ == 0 && rho_ == 1)) { break; }
      log_debug("Tightening parameters: scaling_factor = %f", scaling_factor_);
      update_epsilon();
      update_rho();
    }
  }

  /**
   * @brief Sets epsilon_
   * @param epsilon
   * @return double
   */
  double set_epsilon(double epsilon) {
    epsilon_ = epsilon;
    return epsilon_;
  }

  /**
   * @brief Sets rho_
   * @param rho
   * @return double
   */
  double set_rho(double rho) {
    rho_ = rho;
    return rho_;
  }

  /**
   * @brief Updates epsilon_
   * @return void
   */
  void update_epsilon() { epsilon_ = epsilon_ + (0 - epsilon_) * scaling_factor_; }

  /**
   * @brief Updates rho_
   * @return void
   */
  void update_rho() { rho_ = rho_ + (1 - rho_) * scaling_factor_; }

 private:
#if USE_INT_COLOR_SET
  inline ColorSetType get_color_set(Vertex v) const { return vertex_colors_[v]; }
#else
  inline ColorSetType get_color_set(Vertex v) const { return get_compact_colors(v); }
#endif

#if USE_INT_COLOR_SET
  inline std::size_t get_color_set_size(ColorSetType cs) const { return __builtin_popcount(cs); }
#else
  inline std::size_t get_color_set_size(ColorSetType const& cs) const { return cs.size(); }
#endif

#if USE_INT_COLOR_SET
  inline bool is_subset_of(ColorSetType lhs, ColorSetType rhs) const { return (lhs & rhs) == lhs; }
#else
  inline bool is_subset_of(ColorSetType const& lhs, ColorSetType const& rhs) const { return lhs.is_subset_of(rhs); }
#endif

  /**
   * @brief DP implementation with approximate dominance.
   * @param k
   * @param source
   * @param dest
   * @return void
   */
  void solve_approximate_dominance(int k, Vertex source, Vertex dest, std::atomic_bool* cancel_token = nullptr) {
    log_info("Started dynamic program: epsilon=%.6f, rho=%.6f", epsilon_, rho_);
    util::Timer timer;

    // initialize table
    int last_layer = 0;
    int this_layer = 1;
    Vertex largest_vertex_label = get_graph().vertices(true).back();
    table_[this_layer] = std::vector<TableMap>(largest_vertex_label + 1, TableMap());
    table_[last_layer] = std::vector<TableMap>(largest_vertex_label + 1, TableMap());

    auto source_colors = get_color_set(source);
    table_[last_layer][source][source_colors] = std::make_tuple(0, source_colors, 0, (std::vector<Vertex>){source});

    uint64_t iteration_cap = (transitive_closure_) ? k + 1 : 2ULL * get_graph().number_of_vertices() - 2;
    for (uint64_t i = 1; i <= iteration_cap; i++) {  // main loop

#pragma omp parallel num_threads(num_threads_)
      {
#pragma omp for schedule(static)
        for (Vertex v : get_graph().vertices()) {      // vertex loop
          for (Vertex u : get_graph().neighbors(v)) {  // neighbor loop
            if (!cancel_token || !cancel_token->load()) {
              util::prof.start("neighbor loop");

              for (const auto& [existing_colors_pot, existing_table_entry] : table_[last_layer][u]) {  // table entry loop
                util::prof.start("table entry loop");

                // it is not possible to improve upon a walk ending in dest and collecting all colors by extending it.
                if (u == dest && (int)get_color_set_size(existing_colors_pot) >= k) {
                  util::prof.stop("table entry loop");
                  util::prof.count("table entry loop break", 1);
                  continue;
                }

                // unpack
                const Weight existing_weight_pot = std::get<0>(existing_table_entry);
                const ColorSetType existing_S_ach = std::get<1>(existing_table_entry);
                const Weight existing_weight_ach = std::get<2>(existing_table_entry);
                auto existing_cert = std::get<3>(existing_table_entry);

                // create candidate table entry to be inserted into table_[v][i]
                auto S_pot = existing_colors_pot | get_color_set(v);
                const auto S_ach = existing_S_ach | get_color_set(v);
                Weight weight_pot = existing_weight_pot + get_graph().get_weight(u, v);
                const Weight weight_ach = existing_weight_ach + get_graph().get_weight(u, v);

                // check if candidate entry is absolutely dominated by another entry in table_[v][i]
                bool valid = true;
                for (const auto& [S2_pot, TE] : table_[this_layer][v]) {
                  const Weight weight2_pot = std::get<0>(TE);
                  if (weight2_pot <= weight_pot && is_subset_of(S_pot, S2_pot)) {
                    valid = false;
                    break;
                  }
                }
                if (!valid) {
                  util::prof.stop("table entry loop");
                  util::prof.count("table entry loop break", 2);
                  continue;
                }

                if (epsilon_ > 0 || rho_ < 1) {
                  // check if any current entry can subsume candidate entry while remaining (epsilon_, rho_)-bounded
                  for (const auto& [S2_pot, TE] : table_[this_layer][v]) {
                    const auto [weight2_pot, S2_ach, weight2_ach, cert2] = TE;

                    // perform subsume operation
                    const ColorSetType new_Sp = S_pot | S2_pot;
                    const Weight new_rp = std::min(weight_pot, weight2_pot);

                    // check boundedness. If the new entry is bounded, insert it and delete the old one.
                    // if existing entry has become a solution after subsuming candidate entry, check it against best known.
                    if (weight2_ach <= (1 + epsilon_) * new_rp && get_color_set_size(S2_ach) >= rho_ * get_color_set_size(new_Sp)) {
                      valid = false;
                      table_[this_layer][v].erase(S2_pot);
                      table_[this_layer][v][new_Sp] = std::make_tuple(new_rp, S2_ach, weight2_ach, cert2);
                      if (v == dest && (int)get_color_set_size(new_Sp) >= k) compare_to_best_known(weight2_ach, cert2);
                      break;
                    }
                  }
                  if (!valid) {
                    util::prof.stop("table entry loop");
                    util::prof.count("table entry loop break", 3);
                    continue;
                  }

                  // check if candidate entry can subsume any current entry (or entries) while remaining (epsilon_, rho_)-bounded
                  std::vector<ColorSetType> marked;
                  for (const auto& [S2_pot, TE] : table_[this_layer][v]) {
                    const Weight weight2_pot = std::get<0>(TE);

                    // performe subsume operation
                    const ColorSetType new_Sp = S_pot | S2_pot;
                    const Weight new_rp = std::min(weight_pot, weight2_pot);

                    // check boundedness. Mark current entry for deletion if it can be subsumed.
                    if (weight_ach <= (1 + epsilon_) * new_rp && get_color_set_size(S_ach) >= rho_ * get_color_set_size(new_Sp)) {
                      marked.push_back(S2_pot);
                      S_pot = new_Sp;
                      weight_pot = new_rp;
                    }
                  }
// delete dominated table entries
#if USE_INT_COLOR_SET
                  for (ColorSetType Smarked : marked) { table_[this_layer][v].erase(Smarked); }
#else
                  for (ColorSetType const& Smarked : marked) { table_[this_layer][v].erase(Smarked); }
#endif
                }

                existing_cert.push_back(v);
                table_[this_layer][v][S_pot] = std::make_tuple(weight_pot, S_ach, weight_ach, existing_cert);

                // if candidate entry is a solution, check it against best known
                if (v == dest && (int)get_color_set_size(S_pot) >= k) compare_to_best_known(weight_ach, existing_cert);
              }
              util::prof.stop("table entry loop");
            }  // end table entry loop
            util::prof.stop("neighbor loop");
          }  // end neighbor loop
        }    // end vertex loop
      }      // end parallel region

      if (cancel_token && cancel_token->load()) {
        throw std::runtime_error("canceled: DPSolver::solve_approximate_dominance()");
      }

      for (auto& vertex_map : table_[last_layer]) { vertex_map.clear(); }
      std::swap(last_layer, this_layer);

    }  // end main loop
    log_info("Finished dynamic program: epsilon=%.6f, rho=%.6f, elapsed=%.3fs", epsilon_, rho_, timer.stop());
  }

  /**
   * @brief Compares a given solution to the best known. Updates if better.
   * @param weight
   * @param cert
   * @return void
   */
  void compare_to_best_known(Weight weight, std::vector<Vertex> cert) {
    util::prof.start("compare_to_best_known()");

    if (epsilon_ > 0 || rho_ < 1) { std::tie(cert, weight) = remove_repetitions(cert, weight); }
    if (!has_solution() || weight < get_solution_weight()) {
      log_info("%s: Found critical walk: #hops=%lu", get_solver_name(), cert.size());
      set_critical_walk(cert);
    }

    util::prof.stop("compare_to_best_known()");
  }

  /**
   * @brief Removes repetitions from a walk certificate. These can occur when the DP table is approximate.
   * @param cert
   * @param weight
   * @return std::pair<std::vector<Vertex>, Weight>
   */
  std::pair<std::vector<Vertex>, Weight> remove_repetitions(std::vector<Vertex> cert, Weight weight) const {
    util::prof.start("remove_repetitions()");

    int i = 0;
    while ((size_t)i < cert.size()) {  // a repetition has form ab...zab...z. i is the index of the first a
      bool deleted_something = false;
      for (int j = i + ((cert.size() - i) / 2); j > i + 1;
           j--) {  // j is the index of the second a (note consecutive indices never contain the same vertex)
        if (std::equal(std::next(cert.begin(), i), std::next(cert.begin(), j), std::next(cert.begin(), j))) {
          // the items in range [i, j) can safely be removed
          for (int k = i; k < j; k++) { weight -= get_graph().get_weight(cert[k], cert[k + 1]); }
          cert.erase(std::next(cert.begin(), i), std::next(cert.begin(), j));
          deleted_something = true;
          break;
        }
      }
      if (!deleted_something) i++;
    }

    util::prof.stop("remove_repetitions()");
    return std::make_pair(cert, weight);
  }
};
}  // namespace exact
}  // namespace algorithm
