#include <omp.h>

#include "algorithm/exact/NaiveDPSolver.hpp"

namespace algorithm {
namespace exact {

using Vertex = NaiveDPSolver::Vertex;
using Weight = NaiveDPSolver::Weight;
using ColorSetType = uint32_t;  // supports up to 32 colors

namespace impl {

/** Memoization table. */
using Key = uint64_t;                                 // lower 32-bit: colors, higher 32-bit: vertex
using Value = std::pair<Weight, std::optional<Key>>;  // best weight and previous node
using Table = std::vector<std::unordered_map<ColorSetType, Value>>;

static std::vector<ColorSetType> vertex_colors;
static std::vector<std::vector<Vertex>> in_neighbors;  // (works only with transitive closure) keeps only "color-collecting" edges
static Table dp_table;
static Table dp_active[2];

static inline Key key(Vertex v, ColorSetType colors) { return (static_cast<uint64_t>(v) << 32) | colors; }

/**
 * @brief Returns the best weight in the current DP table.
 *
 * @param v vertex
 * @param colors color set
 * @return Weight best weight
 */
static Weight get_weight(Vertex v, ColorSetType colors) {
  auto it = dp_table[v].find(colors);
  return it == dp_table[v].end() ? INFINITY : it->second.first;
}

static bool is_solution(int k, int num_colors, ColorSetType mask) {
  if (k == num_colors) {
    return mask == (1U << k) - 1;
  } else {
    return __builtin_popcount(mask) >= k;
  }
}

static std::vector<Vertex> construct_solution(Vertex destination, ColorSetType colors) {
  std::vector<Vertex> walk;
  for (std::optional<Key> cur = key(destination, colors); cur.has_value();) {
    Vertex prev_vertex = cur.value() >> 32;
    ColorSetType prev_colors = cur.value();  // drop higher 32-bit
    walk.push_back(prev_vertex);
    cur = dp_table[prev_vertex].at(prev_colors).second;
  }
  std::reverse(walk.begin(), walk.end());
  return walk;
}

// static std::map<Key, Value> memo;
}  // namespace impl

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
bool NaiveDPSolver::solve(int k, Vertex source, Vertex destination, int time_limit) {
  if (time_limit > 0) {
    std::atomic_bool cancel_token(false);
    std::future<void> future = std::async(&NaiveDPSolver::solve_main, this, k, source, destination, &cancel_token);
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

void NaiveDPSolver::solve_main(int k, Vertex source, Vertex destination, std::atomic_bool *cancel_token) {
  //--------------------------------------------------------------------------
  // 1. Preprocessing
  //--------------------------------------------------------------------------
  if (preprocess(k, source, destination)) return;

  // if flagged, create transitive closure and remove colorless vertices
  if (transitive_closure_) {
    create_transitive_closure(true, cancel_token);
    remove_colorless_vertices({source, destination});
  }

  auto const &g = get_graph();
  int num_colors = g.number_of_colors();
  if (num_colors > 32) throw std::runtime_error("Not implemented: |C| > 32");

  // re-index colors
  compact_colors();

  // store vertex colors as integers
  auto vs = g.vertices();
  int table_size = *std::max_element(vs.begin(), vs.end()) + 1;
  impl::vertex_colors.clear();
  impl::vertex_colors.resize(table_size);

  for (auto v : vs) {
    ColorSetType colors = 0;
    for (auto c : get_compact_colors(v)) colors |= 1U << c;
    impl::vertex_colors[v] = colors;
  }

  if (transitive_closure_) {
    impl::in_neighbors.clear();
    impl::in_neighbors.resize(table_size);

    // only consider "color-collecting" (directed) edges
    for (auto v : vs) {
      for (auto u : g.neighbors(v)) {
        if (v == destination || ((impl::vertex_colors[u] & impl::vertex_colors[v]) != impl::vertex_colors[v])) {
          // v collects a new color or v is destination
          impl::in_neighbors[v].push_back(u);
        }
      }
    }
  }

  //--------------------------------------------------------------------------
  // 2. Main Computation
  //--------------------------------------------------------------------------
  Weight best_weight = get_solution_weight();

  // set up tables
  impl::dp_table.clear();
  impl::dp_table.resize(table_size);
  for (int i = 0; i < 2; ++i) {
    impl::dp_active[i].clear();
    impl::dp_active[i].resize(table_size);
  }

  // add initial entry (assuming source has no colors)
  int previous_layer = 1, current_layer = 0;
  impl::dp_table[source][0] = impl::dp_active[previous_layer][source][0] = std::make_pair(0.0, std::nullopt);

  // main loop
  int num_iterations = 0;  // for debugging
  bool updated = true;
  while (updated) {
    updated = false;

    if (util::logging::LogLevel::DEBUG >= util::logging::log_level) {
      // log stats
      std::size_t dp_table_sz = 0, prev_table_sz = 0;
      for (auto &t : impl::dp_table) dp_table_sz += t.size();
      for (auto &t : impl::dp_active[previous_layer]) prev_table_sz += t.size();

      log_debug("%s: iteration=%d, dp_table_size=%lu, active_table_size=%lu", get_solver_name(), num_iterations,
                dp_table_sz, prev_table_sz);
    }

    // parallelize over vertices
#pragma omp parallel num_threads(num_threads_)
    {
#pragma omp for schedule(dynamic)
      for (Vertex v : vs) {  // vertex loop
        // look for in-neighbor's new information
        for (Vertex u : (transitive_closure_ ? impl::in_neighbors[v] : g.neighbors(v))) {
          if (!cancel_token || !cancel_token->load()) {  // check for timeout
            for (auto &p : impl::dp_active[previous_layer][u]) {
              auto current_colors = p.first;
              auto new_colors = current_colors | impl::vertex_colors[v];
              if (transitive_closure_) {
                if (v == destination) {
                  // go to destination only in the end
                  if (!impl::is_solution(k, num_colors, new_colors)) continue;
                } else {
                  // not a color-collecting walk; reject
                  if (current_colors == new_colors) continue;
                }
              }

              auto new_weight = p.second.first + g.get_weight(u, v);
              auto current_weight = impl::get_weight(v, new_colors);

              if (new_weight < current_weight) {
                // update information
                updated = true;
                auto new_value = std::make_pair(new_weight, impl::key(u, current_colors));
                impl::dp_table[v][new_colors] = impl::dp_active[current_layer][v][new_colors] = new_value;

                // check for the stop condition
                if (v == destination && impl::is_solution(k, num_colors, new_colors)) {
                  if (new_weight < best_weight) {
                    // backtrack soluition walk
                    best_weight = new_weight;
                    set_critical_walk(impl::construct_solution(destination, new_colors));
                  }
                }
              }
            }
          }
        }
      }  // end vertex loop
    }    // end parallel region

    // handle timeout
    if (cancel_token && cancel_token->load()) { throw std::runtime_error("canceled: NaiveDPSolver::solve_main()"); }

    // switch active layers
    for (auto &entry : impl::dp_active[previous_layer]) entry.clear();
    std::swap(current_layer, previous_layer);
    ++num_iterations;
  }
}

// void NaiveDPSolver::solve_main_recuirsive(int k, int num_colors, Vertex source, Vertex destination, std::atomic_bool *cancel_token) {
//   //--------------------------------------------------------------------------
//   // 2. Main Computation
//   //--------------------------------------------------------------------------
//   impl::memo.clear();
//   double result = INFINITY;
//   ColorSetType target = (1U << num_colors) - 1;  // all |C| colors

//   if (k == num_colors) {
//     result = solve_recursive(source, destination, destination, target, cancel_token);
//   } else {
//     for (ColorSetType mask = 1; mask != (1U << num_colors); ++mask) {
//       // count the number of 1-bits
//       if (__builtin_popcount(mask) < k) continue;  // we need k colors

//       auto ret = solve_recursive(source, destination, destination, mask, cancel_token);
//       if (ret < result) {
//         result = ret;
//         target = mask;
//       }
//     }
//   }

//   //--------------------------------------------------------------------------
//   // 3. Solution Recovery
//   //--------------------------------------------------------------------------
//   bool found = !std::isinf(result);
//   if (found) {
//     std::vector<Vertex> walk;
//     for (auto cur = std::optional(destination); cur.has_value();) {
//       auto v = cur.value();
//       walk.push_back(v);
//       cur = impl::memo.at(impl::key(v, target)).second;
//       target &= ~impl::vertex_colors[v];
//     }
//     std::reverse(walk.begin(), walk.end());
//     set_critical_walk(walk);
//   }
// }

/**
 * @brief Recursive implementation.
 *
 * @param source
 * @param v
 * @param colors
 * @return std::pair<Weight, std::vector<Vertex>>
 */
// Weight NaiveDPSolver::solve_recursive(Vertex source, Vertex destination, Vertex v, uint32_t colors, std::atomic_bool *cancel_token) {
//   if (cancel_token && cancel_token->load()) { throw std::runtime_error("canceled: NaiveDPSolver::solve_recursive()"); }

//   auto key = impl::key(v, colors);
//   if (!util::contains(impl::memo, key)) {
//     Weight best = INFINITY;                     // defined in <cmath>
//     std::optional<Vertex> prev = std::nullopt;  // for backtracking
//     auto vcol = impl::vertex_colors[v];

//     if (v == source && colors == 0) {
//       best = 0.0;  // base case
//     } else if (v != destination && (vcol & colors) == 0) {
//       // set infinity
//     } else {
//       for (auto u : get_graph().neighbors(v)) {
//         auto w = solve_recursive(source, destination, u, colors & (~vcol), cancel_token);
//         w += get_graph().get_weight(u, v);
//         if (w < best) {
//           best = w;
//           prev = u;
//         }
//       }
//     }
//     impl::memo[key] = std::make_pair(best, prev);
//   }
//   return impl::memo.at(key).first;
// }
}  // namespace exact
}  // namespace algorithm
