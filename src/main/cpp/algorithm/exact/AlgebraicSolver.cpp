#include "AlgebraicSolver.hpp"

namespace algorithm {
namespace exact {

using Circuit = AlgebraicSolver::Circuit;
using Vertex = AlgebraicSolver::Vertex;
using CompactColorSet = AlgebraicSolver::CompactColorSet;

//==============================================================================
//    Circuit Construction
//==============================================================================
namespace impl {

static double const SINGLE_RUN_SUCCESS_PROBABILITY = 0.2;

static std::set<std::tuple<int, int, int, int>> create_circuit_internal(  //
    base::BaseSolver const *solver,                                       //
    algebraic::CompactStrategy compact_level,                             //
    int k,                                                                //
    int s,                                                                //
    int t,                                                                //
    int lb,                                                               //
    int ub,                                                               //
    Circuit &circ,                                                        //
    std::unordered_map<int, int> *node_ids,                               //
    std::unordered_set<int> *color_nodes                                  //
) {
  auto &g = solver->get_graph();

  // create vertex-color nodes ((vertex, multiplicity) -> node-id)
  std::map<std::tuple<int, int>, int> color_node_map;

  // create vertex (* multiplicity) nodes
  if (compact_level == algebraic::CompactStrategy::SemiCompact) {
    for (auto v : g.vertices()) {
      if (v == s || v == t) continue;

      std::vector<int> colors;
      for (auto c : solver->get_compact_colors(v)) colors.push_back(c);  // convert to int vector
      assert(!colors.empty());

      for (std::size_t i = 0; i < colors.size(); ++i) {
        auto color_node = circ.add_addition_gate(colors, false);  // create an aggretated node
        if (color_nodes) color_nodes->insert(color_node);
        color_node_map[std::make_pair(v, i)] = color_node;
      }
    }
  }

  // keep track of active configurations (node_id, vertex, distance, multiplicity)
  std::set<std::tuple<int, int, int, int>> active[2];
  int current = 0;

  // add internal nodes
  for (int layer = 1; layer <= k; ++layer) {
    for (auto v : g.vertices()) {
      if (v == s || v == t) continue;

      // create or get vertex node
      int color_node;
      switch (compact_level) {
        case algebraic::CompactStrategy::Standard: {
          color_node = solver->get_compact_colors(v).front();  // get a variable node
          break;
        }
        case algebraic::CompactStrategy::SemiCompact: {
          color_node = color_node_map.at(std::make_pair(v, 0));
          break;
        }
        case algebraic::CompactStrategy::Compact: {
          std::vector<int> colors;
          for (auto c : solver->get_compact_colors(v)) colors.push_back(c);  // convert to int vector
          color_node = circ.add_addition_gate(colors, false);                // create an aggretated node
          if (color_nodes) color_nodes->insert(color_node);
          break;
        }
        case algebraic::CompactStrategy::Naive: {
          color_node = solver->get_compact_colors(v).front();  // get a variable node
          break;
        }
        default: {
          throw std::invalid_argument("unknown compact level");
        }
      }

      if (layer == 1) {
        // first layer
        int d = g.get_weight(s, v);
        if (d > ub) continue;  // this edge cannot be in the solution walk

        auto mul_id = circ.add_multiplication_gate({color_node}, false);
        active[current].insert(std::make_tuple(mul_id, v, d, compact_level == algebraic::CompactStrategy::SemiCompact ? 1 : 0));

        // update vertex lookup table
        if (node_ids) (*node_ids)[mul_id] = v;
      } else if (compact_level == algebraic::CompactStrategy::Naive) {
        // internal layers (naive implementation)

        std::unordered_map<int, int> v_ids;  // weight -> addition node id

        for (auto &[prev_node_id, prev_v, prev_d, mult] : active[current ^ 1]) {
          if (prev_v == v) continue;  // do not revisit the same vertex

          // if `prev_v` can collect all colors of `v`, no need to move to `v`
          if (prev_v != v && solver->get_compact_colors(v).is_subset_of(solver->get_compact_colors(prev_v))) continue;

          // move to a different vertex
          int d = (prev_v == v ? 0 : g.get_weight(prev_v, v)) + prev_d;
          if (d > ub) continue;  // over-budget

          // create a new multiplication gate
          auto mul_id = circ.add_multiplication_gate({prev_node_id, color_node}, false);

          if (util::contains(v_ids, d)) {
            // already visited
            circ.add_edge(mul_id, v_ids.at(d));
          } else {
            // create a new addition gate
            auto add_id = circ.add_addition_gate({mul_id}, false);

            // register and enqueue new nodes
            v_ids[d] = add_id;
            if (node_ids) (*node_ids)[add_id] = v;
            active[current].insert(std::make_tuple(add_id, v, d, 0));
          }
        }
      } else {
        // internal layers
        std::unordered_map<int, int> v_ids;  // weight -> addition node id

        for (auto &[prev_node_id, prev_v, prev_d, mult] : active[current ^ 1]) {
          if (compact_level == algebraic::CompactStrategy::Standard && prev_v == v)
            continue;  // do not revisit the same vertex

          if (compact_level == algebraic::CompactStrategy::SemiCompact && prev_v == v) {
            // repeating vertex: collect more colors
            if (mult >= static_cast<int>(solver->get_compact_colors(v).size()))
              continue;  // already collected all colors at v

            // create new multiplication gate
            auto mul_id = circ.add_multiplication_gate({prev_node_id, color_node_map.at(std::make_pair(v, mult))}, false);

            // register and enqueue new node
            if (node_ids) (*node_ids)[mul_id] = v;
            active[current].insert(std::make_tuple(
                mul_id, v, prev_d, compact_level == algebraic::CompactStrategy::SemiCompact ? mult + 1 : 0));
          } else {
            // if `prev_v` can collect all colors of `v`, no need to move to `v`
            if (prev_v != v && solver->get_compact_colors(v).is_subset_of(solver->get_compact_colors(prev_v))) continue;

            // move to a different vertex
            int d = (prev_v == v ? 0 : g.get_weight(prev_v, v)) + prev_d;
            if (d > ub) continue;  // over-budget

            if (util::contains(v_ids, d)) {
              // already visited
              circ.add_edge(prev_node_id, v_ids.at(d));
            } else {
              // create new addition and multiplication gates
              auto add_id = circ.add_addition_gate({prev_node_id}, false);
              auto mul_id = circ.add_multiplication_gate({add_id, color_node}, false);

              // register and enqueue new nodes
              v_ids[d] = add_id;
              if (node_ids) (*node_ids)[mul_id] = v;
              active[current].insert(std::make_tuple(mul_id, v, d, compact_level == algebraic::CompactStrategy::SemiCompact ? 1 : 0));
            }
          }
        }
      }
    }
    // keep only previous layer's information
    current ^= 1;
    active[current].clear();
  }

  return active[current ^ 1];
}

static void create_circuit(                    //
    base::BaseSolver const *solver,            //
    bool multiple_output,                      //
    algebraic::CompactStrategy compact_level,  //
    int k,                                     //
    int s,                                     //
    int t,                                     //
    int lb,                                    //
    int ub,                                    //
    Circuit &circ,                             //
    std::unordered_map<int, int> *node_ids,    //
    std::unordered_set<int> *color_nodes       //
) {
  util::Timer circuit_create_time;
  log_debug("%s: Started: Creating Circuit using compact_level=%d", solver->get_solver_name(), compact_level);

  auto last_layer = create_circuit_internal(solver, compact_level, k, s, t, lb, ub, circ, node_ids, color_nodes);

  // connection to root
  if (multiple_output) {
    std::vector<int> roots;
    for (int i = lb; i <= ub; ++i) roots.push_back(circ.add_addition_gate({}, true));

    for (auto &[prev_node_id, prev_v, prev_d, mult] : last_layer) {
      int w = prev_d + solver->get_graph().get_weight(prev_v, t);
      if (lb <= w && w <= ub) circ.add_edge(prev_node_id, roots[w - lb]);
    }
  } else {
    int root = circ.add_addition_gate({}, true);
    for (auto &[prev_node_id, prev_v, prev_d, mult] : last_layer) {
      int w = prev_d + solver->get_graph().get_weight(prev_v, t);
      if (lb <= w && w <= ub) circ.add_edge(prev_node_id, root);
    }
    assert(circ.number_of_output_nodes() == 1);
  }

  log_debug("%s: Created circuit: k=%d, s=%d, t=%d, lb=%d, ub=%d, #nodes=%lu (+:%lu, *:%lu), #edges=%lu, elapsed=%.3f",
            solver->get_solver_name(), k, s, t, lb, ub, circ.number_of_nodes(), circ.number_of_addition_gates(),
            circ.number_of_multiplication_gates(), circ.number_of_edges(), circuit_create_time.stop());
}

/**
 * @brief Adjusts the number of trials for deciding infeasible values so that the success probability remains.
 *
 * @param original_num_trials original number of trials
 * @param success_probability single-run success probability
 * @param num_runs number of runs of detections
 * @return int adjusted number of trials
 */
static int adjust_num_trials(int original_num_trials, double success_probability, int num_runs) {
  auto q = 1 - success_probability;
  int num_trials = original_num_trials;
  double target_p = std::pow(q, original_num_trials);  // target success probability

  while (true) {
    auto qtheta = std::pow(q, num_trials);
    if (std::pow(1 - qtheta, num_runs) >= 1 - target_p) break;
    ++num_trials;
  }
  return num_trials;
}
}  // namespace impl

//==============================================================================
//    Solver Management
//==============================================================================

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
bool AlgebraicSolver::solve(int k, Vertex source, Vertex destination, int time_limit) {
  if (time_limit > 0) {
    std::atomic_bool cancel_token(false);
    std::future<void> future = std::async(&AlgebraicSolver::solve_main, this, k, source, destination, &cancel_token);
    if (future.wait_for(std::chrono::seconds(time_limit)) == std::future_status::timeout) {
      cancel_token.store(true);  // cancel the task

      char const *failure_message = "";
      try {
        future.get();
      } catch (std::exception const &e) {  //
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

void AlgebraicSolver::solve_main(int k, Vertex source, Vertex destination, std::atomic_bool *cancel_token) {
  //--------------------------------------------------------------------------
  // 1. Preprocessing
  //--------------------------------------------------------------------------
  if (preprocess(k, source, destination)) return;
  if (k > algebra::MultilinearDetector::MAX_K_VALUE) {
    throw std::invalid_argument(util::format("k must be between 1 and %d, inclusive", algebra::MultilinearDetector::MAX_K_VALUE));
  }

  create_transitive_closure(true, cancel_token);
  remove_colorless_vertices({source, destination});  // keep source and destination
  round_weights();                                   // round weights after creating the transitive closure
  create_transitive_closure(true, cancel_token);     // again, we need to make the graph metric

  if (compact_level_ == algebraic::CompactStrategy::Standard || compact_level_ == algebraic::CompactStrategy::Naive) {
    split_colors(true);
  }
  compact_colors();  // re-index colors

  log_info("%s: Finished preprocessing: n=%lu, m=%lu, |C|=%lu", get_solver_name(), get_graph().number_of_vertices(),
           get_graph().number_of_edges(), get_graph().number_of_colors());

  //--------------------------------------------------------------------------
  // 2. Computing Upper and Lower Bounds
  //--------------------------------------------------------------------------

  auto lb = source == destination ? 0 : int(get_graph().get_weight(source, destination));

#if GUROBI_ON
  // find lower bound by LP Relaxation
  int lp_seed = rand_.randint(0, 2000000000);
  auto lp_runner = ilp::ILPRunner(get_graph(), k, source, destination, true, num_threads_, lp_seed, 0, false);
  lp_runner.run();
  auto lp_lb = lp_runner.get_objective();
  lb = std::max(lb, static_cast<int>(std::ceil(lp_lb)));
#endif

  // compute integral upper bound from the current heuristic solution
  int ub = 0;
  if (has_solution()) {
    auto &g = get_graph();
    auto ub_sol = get_solution();
    auto prev = ub_sol[0];
    for (std::size_t i = 1; i < ub_sol.size() - 1; ++i) {
      if (g.has_vertex(ub_sol[i])) {
        ub += static_cast<int>(get_graph().get_weight(prev, ub_sol[i]));
        prev = ub_sol[i];
      }
    }
    ub += static_cast<int>(get_graph().get_weight(prev, ub_sol.back()));
  }

  //--------------------------------------------------------------------------
  // 3. Binary Search for Finding Minimum Distance
  //--------------------------------------------------------------------------
  util::Timer search_timer;
  int c = get_graph().number_of_colors();
  int hi = ub;

  switch (search_strategy_) {
    case algebraic::SearchStrategy::Basic: {
      log_info("%s: Started standard binary search: lo=%d, hi=%d", get_solver_name(), lb, ub);
      hi = run_standard_binary_search(k, source, destination, c, lb, ub, cancel_token);
      break;
    }
    case algebraic::SearchStrategy::Probabilistic: {
      log_info("%s: Started probabilistic binary search: lo=%d, hi=%d", get_solver_name(), lb, ub);
      hi = run_probabilistic_binary_search(k, source, destination, c, lb, ub, cancel_token);
      break;
    }
    case algebraic::SearchStrategy::Sequential: {
      log_info("%s: Started sequential search: lo=%d, hi=%d", get_solver_name(), lb, ub);
      hi = run_sequential_search(k, source, destination, c, lb, ub, cancel_token);
      break;
    }
    case algebraic::SearchStrategy::MultiOutput: {
      log_info("%s: Started multi-output search: lo=%d, hi=%d", get_solver_name(), lb, ub);
      hi = run_multi_output_search(k, source, destination, c, lb, ub, cancel_token);
      break;
    }
    default: {
      throw std::invalid_argument("unknown search strategy");
    }
  }
  if (!recover_all_) {
    // solution recovery
    find_certificate(k, source, destination, c, hi, num_confident_failures_, cancel_token);
  }
  log_info("%s: Found optimal walk length: %d, elapsed=%.3fs", get_solver_name(), hi, search_timer.stop());
}

//==============================================================================
//    Search Algorithms
//==============================================================================

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
int AlgebraicSolver::run_standard_binary_search(int k, int source, int destination, int c, int lo, int hi,
                                                std::atomic_bool *cancel_token) {
  // adjust num_trials
  int num_trials = impl::adjust_num_trials(num_confident_failures_, impl::SINGLE_RUN_SUCCESS_PROBABILITY,
                                           std::floor(std::log2(hi - lo + 1)));
  log_debug("%s: Adjusted num_trials for search: from=%d, to=%d", get_solver_name(), num_confident_failures_, num_trials);

  // binary search
  while (lo < hi) {
    int m = (lo + hi) / 2;
    log_debug("%s: Testing objective: %d <- [%d, %d]", get_solver_name(), m, lo, hi);
    if (test_walk_length(k, source, destination, c, lo, m, num_confident_failures_, cancel_token)) {
      hi = m;  // minimize feasible k
      if (recover_all_) {
        find_certificate(k, source, destination, c, hi, num_confident_failures_, cancel_token);  // solution recovery
      }

      // obtained weight can be less than the target
      hi = std::min(hi, static_cast<int>(get_solution_weight()));
    } else {
      lo = m + 1;  // maximize infeasible k
    }
  }
  return hi;
}

// /**
//  * @brief Probabilistic binary search (deprecated).
//  *
//  * @param k
//  * @param source
//  * @param destination
//  * @param c
//  * @param lb
//  * @param ub
//  * @param cancel_token
//  * @return int
//  */
/*
int AlgebraicSolver::run_probabilistic_binary_search_naive(int k, int source, int destination, int c, int lo, int hi,
                                                           std::atomic_bool *cancel_token) {
  std::vector<int> failure_count(hi);

  while (lo < hi) {
    int m = (lo + hi) / 2;

    while (m < hi) {
      log_debug("%s: Testing objective: %d <- [%d, %d]", get_solver_name(), m, lo, hi);
      if (test_walk_length(k, source, destination, c, lo, m, 1, cancel_token)) {
        hi = m;                                                                           // minimize feasible k
        if (recover_all_) find_certificate(k, source, destination, c, hi, cancel_token);  // solution recovery

        // obtained weight can be less than the target
        hi = std::min(hi, static_cast<int>(get_solution_weight()));
        break;
      } else {
        ++failure_count[m];  // increment counter

        int cnt = 0;
        for (int i = m; i < hi; ++i) cnt += failure_count[i];

        if (cnt >= num_confident_failures_) {
          lo = m + 1;  // confident enough to reject m
          break;
        }

        m = (m + hi + 1) / 2;  // update m
      }
    }
  }
  return hi;
}
*/

// /**
//  * Deprecated: The "correct" version of this algorithm is quite slow.
//  *
//  * @brief Probabilistic binary search.
//  *
//  * @param k
//  * @param source
//  * @param destination
//  * @param c
//  * @param lb
//  * @param ub
//  * @param cancel_token
//  * @return int
//  */
/*
int AlgebraicSolver::run_probabilistic_binary_search(int k, int source, int destination, int c, int lo, int hi,
                                                     std::atomic_bool *cancel_token) {
  double const p = impl::SINGLE_RUN_SUCCESS_PROBABILITY;  // estimated success probability
  std::vector<int> success_count(hi);
  std::vector<int> failure_count(hi);
  std::vector<double> weights(hi);
  bool scale_weights = false;
  int orig_hi = hi;
  int orig_range = hi - lo + 1;
  double target_p = std::pow(1 - p, num_confident_failures_);

  // precompute binomial coefficients
  int max_a = hi - lo;  // there should be at most (hi-lo) successes
  int max_b = (max_a + 1) * num_confident_failures_ - max_a;
  std::vector<std::vector<double>> binom(max_a + max_b + 1, std::vector<double>(max_a + 1));
  for (int i = 0; i <= max_a + max_b; ++i) {
    binom[i][0] = 1.0;
    for (int j = 1; j <= max_a && j <= i; ++j) { binom[i][j] = binom[i - 1][j] + binom[i - 1][j - 1]; }
  }

  // binary search with weights
  while (lo < hi) {
    double max_weight = *std::max_element(weights.begin() + lo, weights.begin() + hi);
    int m = 0;

    if (max_weight <= 0.0) {
      m = (lo + hi) / 2;
      weights[m] = 1.0;  // reset to the mid-point
    } else {
      // pick a value with the maximum weight
      std::vector<int> candidates;
      for (int i = lo; i < hi; ++i) {
        if (weights[i] == max_weight) candidates.push_back(i);
      }

      // scale weights
      if (scale_weights) {
        double sum = std::accumulate(weights.begin() + lo, weights.begin() + hi, 0.0);
        for (int i = lo; i < hi; ++i) { weights[i] /= sum; }
        scale_weights = false;
      }

      auto r = rand_.randint(0, static_cast<int>(candidates.size()) - 1);
      m = candidates[r];
    }

    log_debug("%s: Testing objective: %d <- [%d, %d]", get_solver_name(), m, lo, hi);

    if (test_walk_length(k, source, destination, c, lo, m, 1, cancel_token)) {
      // printf("m=%d, T:F%d\n", m, failure_count[m]);
      ++success_count[m];  // increment counter

      hi = m;  // minimize feasible k
      if (recover_all_) {
        find_certificate(k, source, destination, c, hi, num_confident_failures_, cancel_token);  // solution recovery
      }

      // obtained weight can be less than the target
      hi = std::min(hi, static_cast<int>(get_solution_weight()));
      scale_weights = true;
    } else {
      ++failure_count[m];  // increment counter
      // printf("m=%d, F%d\n", m, failure_count[m]);

      // compute stats
      for (int i = lo; i <= m; ++i) {
        int a = 0, b = 0;
        for (int j = i; j < orig_hi; ++j) {
          a += success_count[j];
          b += failure_count[j];
        }
        assert(a <= max_a);
        assert(b <= max_b);
        if (std::pow(1 - binom[a + b][a] * std::pow(p, a) * std::pow(1 - p, b), orig_range + 1) >= 1 - target_p) {
        // if (binom[a + b][a] * std::pow(p, a) * std::pow(1 - p, b) < target_p / orig_range) {
          lo = i + 1;  // confident enough to reject m
          scale_weights = true;
        }
      }

      // distribute weights to higher half
      if (m >= lo) {
        int nxt = (m + 1 + hi) / 2;
        if (nxt < hi) {
          auto diff = weights[m] * p;
          weights[m] -= diff;
          weights[nxt] += diff;
        }
      }
    }
  }
  return hi;
}
*/

/**
 * @brief Probabilistic binary search.
 *
 * @param k
 * @param source
 * @param destination
 * @param c
 * @param lb
 * @param ub
 * @param cancel_token
 * @return int
 */
int AlgebraicSolver::run_probabilistic_binary_search(int k, int source, int destination, int c, int lo, int hi,
                                                     std::atomic_bool *cancel_token) {
  double const p = impl::SINGLE_RUN_SUCCESS_PROBABILITY;  // estimated success probability
  // adjust num_trials
  int num_trials = impl::adjust_num_trials(num_confident_failures_, p, hi - lo + 1);
  log_debug("%s: Adjusted num_trials for search: from=%d, to=%d", get_solver_name(), num_confident_failures_, num_trials);

  std::vector<std::pair<int, int>> mid_points;  // use as a stack of (ell, number of failures)

  // binary search
  while (lo < hi) {
    if (mid_points.empty()) {
      mid_points.push_back({(lo + hi) / 2, 0});  // create a new midpoint
    }
    int const m = mid_points.back().first;
    log_debug("%s: Testing objective: %d <- [%d, %d]", get_solver_name(), m, lo, hi);

    if (test_walk_length(k, source, destination, c, lo, m, 1, cancel_token)) {
      hi = m;  // update upper bound
      if (recover_all_) {
        find_certificate(k, source, destination, c, hi, num_confident_failures_, cancel_token);  // solution recovery
      }

      // obtained weight can be less than the target
      hi = std::min(hi, static_cast<int>(get_solution_weight()));
      mid_points.pop_back();  // pop from stack
    } else {
      // increment failure counter and check with threshold
      if (mid_points.back().second++ >= num_confident_failures_) {
        lo = m + 1;          // confident enough to reject m
        mid_points.clear();  // clear the stack
      } else {
        // go higher with some probability
        if (m < hi - 1 && rand_.random() < 1 - p / 2) mid_points.push_back({(m + 1 + hi) / 2, 0});
      }
    }
  }
  return hi;
}

int AlgebraicSolver::run_sequential_search(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token) {
  while (lo < hi) {
    int m = hi - 1;
    log_debug("%s: Testing objective: %d <- [%d, %d]", get_solver_name(), m, lo, hi);
    if (test_walk_length(k, source, destination, c, lo, m, num_confident_failures_, cancel_token)) {
      hi = m;  // minimize feasible k
      if (recover_all_)
        find_certificate(k, source, destination, c, hi, num_confident_failures_, cancel_token);  // solution recovery

      // obtained weight can be less than the target
      hi = std::min(hi, static_cast<int>(get_solution_weight()));
    } else {
      break;
    }
  }
  return hi;
}

int AlgebraicSolver::run_multi_output_search(int k, int source, int destination, int c, int lo, int hi, std::atomic_bool *cancel_token) {
  for (int iteration = 0; lo < hi && iteration < num_confident_failures_; ++iteration) {
    // create a circuit
    Circuit circ(c);
    impl::create_circuit(this, true, compact_level_, k, source, destination, lo, hi - 1, circ, nullptr, nullptr);

    // run detector
    algebra::MultilinearDetector detector(circ, k, num_threads_);
    auto ret = detector.run_single(rand_, cancel_token);

    for (int i = lo; i < hi; ++i) {
      if (ret[i - lo]) {
        hi = i;
        log_info("%s: Found upper bound: it=%d, lb=%d, ub=%d", get_solver_name(), iteration, lo, hi);
        break;
      }
    }
  }
  return hi;
}

//==============================================================================
//    Multilinear Detection
//==============================================================================

bool AlgebraicSolver::test_walk_length(int k, int s, int t, int c, int lb, int ub, int num_trials, std::atomic_bool *cancel_token) {
  // create a circuit
  Circuit circ(c);
  impl::create_circuit(this, false, compact_level_, k, s, t, lb, ub, circ, nullptr, nullptr);
  if (circ.number_of_output_nodes() != 1) throw std::invalid_argument("circuit must have one output node");

  // output node must be reachable
  if (circ.in_degree(circ.get_output_nodes().front()) == 0) {
    log_debug("%s: infeasible circuit", get_solver_name());
    return false;
  }

  // run detector
  algebra::MultilinearDetector detector(circ, k, num_threads_);
  auto ret = detector.run(rand_, num_trials, true, cancel_token);
  return ret;
}

//==============================================================================
//    Solution Recovery
//==============================================================================

void AlgebraicSolver::find_certificate(int k, int s, int t, int c, int walk_length, int num_trials, std::atomic_bool *cancel_token) {
  log_info("%s: Finding certificate: objective=%d", get_solver_name(), walk_length);
  util::Timer timer;

  // create a circuit
  std::unordered_map<int, int> node_ids;
  std::unordered_set<int> color_nodes;
  Circuit circ(c);
  impl::create_circuit(this, false, compact_level_, k, s, t, walk_length, walk_length, circ, &node_ids, &color_nodes);
  log_info("%s: Created circuit: #vars=%lu, #nodes=%lu (+:%lu, *:%lu), #edges=%lu, k=%d, #threads=%d",
           get_solver_name(), circ.number_of_variables(), circ.number_of_nodes(), circ.number_of_addition_gates(),
           circ.number_of_multiplication_gates(), circ.number_of_edges(), k, num_threads_);

  // find a certificate circuit
  bool use_las_vegas = recovery_strategy_ == algebraic::RecoveryStrategy::LV;
  if (!use_las_vegas) {
    int orig_num_trials = num_trials;
    num_trials = impl::adjust_num_trials(num_trials, impl::SINGLE_RUN_SUCCESS_PROBABILITY,
                                         k * std::log2(get_graph().number_of_vertices()));
    log_debug("%s: Adjusted num_trials: from=%d, to=%d", get_solver_name(), orig_num_trials, num_trials);
  }
  algebra::MultilinearDetector detector(circ, k, num_threads_);
  auto cert = detector.find_certificate(rand_, color_nodes, cancel_token, use_las_vegas, num_trials);

  // backtrack nodes in the certificate circuit
  std::vector<int> vs = {t};
  for (auto node : cert.topological_ordering(true)) {
    if (util::contains(node_ids, node)) vs.push_back(node_ids.at(node));
  }
  vs.push_back(s);
  std::reverse(vs.begin(), vs.end());

  log_info("%s: Found certificate walk: objective=%d, elapsed=%.3fs", get_solver_name(), walk_length, timer.stop());
  set_critical_walk(vs);
}

}  // namespace exact
}  // namespace algorithm