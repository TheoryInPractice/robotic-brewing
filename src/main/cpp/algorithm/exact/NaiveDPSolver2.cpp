#include <omp.h>

#include "algorithm/exact/NaiveDPSolver2.hpp"

namespace algorithm {
namespace exact {

using Vertex = NaiveDPSolver2::Vertex;
using Weight = NaiveDPSolver2::Weight;
using ColorSetType = uint32_t;  // supports up to 32 colors

namespace impl {

using Key = uint64_t;                                 // lower 32-bit: colors, higher 32-bit: vertex
using Value = std::pair<Weight, std::optional<Key>>;  // best weight and previous node
using MainTable = std::vector<std::vector<Value>>;
using ActiveTable = std::vector<std::unordered_set<ColorSetType>>;

// graph properties
static ds::map::Bimap<int, Vertex> vertex_labels;  // map from compacted label (left) to original label (right)
static std::vector<ColorSetType> vertex_colors;
static std::vector<std::vector<Weight>> weight_matrix;  // adjacency matrix of compacted vertices
static std::vector<std::vector<Vertex>> in_neighbors;  // (works only with transitive closure) keeps only "color-collecting" edges
static int idx_source = 0;
static int idx_destination = 0;

// DP tables
static MainTable dp_table;
static ActiveTable dp_active[2];

static void initialize_graph(algorithm::base::BaseSolver const *solver, Vertex source, Vertex destination) {
  auto const &graph = solver->get_graph();
  int num_vertices = graph.number_of_vertices();

  // re-index vertices;
  vertex_labels = ds::map::Bimap<int, int>(graph.vertices());
  idx_source = vertex_labels.g(source);
  idx_destination = vertex_labels.g(destination);

  // store vertex colors as integers
  vertex_colors.clear();
  vertex_colors.resize(num_vertices);

  for (int i = 0; i < num_vertices; ++i) {
    ColorSetType colors = 0;
    for (auto c : solver->get_compact_colors(vertex_labels.f(i))) colors |= 1U << c;
    vertex_colors[i] = colors;
  }

  // initialize "color-collecting" neighbors
  in_neighbors.clear();
  in_neighbors.resize(num_vertices);

  for (int i = 0; i < num_vertices; ++i) {
    auto v = vertex_labels.f(i);
    for (auto u : graph.neighbors(v)) {
      auto j = vertex_labels.g(u);
      if (i == idx_destination || ((vertex_colors[j] & vertex_colors[i]) != vertex_colors[i])) {
        // i collects a new color or i is destination
        in_neighbors[i].push_back(j);
      }
    }
  }

  // initialize weights
  weight_matrix.clear();
  weight_matrix.resize(num_vertices, std::vector<Weight>(num_vertices, 0.0));
  for (int i = 0; i < num_vertices; ++i) {
    for (int j = i + 1; j < num_vertices; ++j) {
      weight_matrix[i][j] = weight_matrix[j][i] = graph.get_weight(vertex_labels.f(i), vertex_labels.f(j));
    }
  }
}

static inline Key key(Vertex v, ColorSetType colors) { return (static_cast<uint64_t>(v) << 32) | colors; }

static bool is_solution(int k, int num_colors, ColorSetType mask) {
  if (k == num_colors) {
    return mask == (1U << k) - 1;
  } else {
    return __builtin_popcount(mask) >= k;
  }
}

static std::vector<Vertex> construct_solution(int idx_destination, ColorSetType colors) {
  std::vector<Vertex> walk;
  for (std::optional<Key> cur = key(idx_destination, colors); cur.has_value();) {
    Vertex prev_vertex = cur.value() >> 32;
    ColorSetType prev_colors = cur.value();  // drop higher 32-bit
    walk.push_back(vertex_labels.f(prev_vertex));
    cur = dp_table[prev_vertex][prev_colors].second;
  }
  std::reverse(walk.begin(), walk.end());
  return walk;
}
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
bool NaiveDPSolver2::solve(int k, Vertex source, Vertex destination, int time_limit) {
  if (time_limit > 0) {
    std::atomic_bool cancel_token(false);
    std::future<void> future = std::async(&NaiveDPSolver2::solve_main, this, k, source, destination, &cancel_token);
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

void NaiveDPSolver2::solve_main(int k, Vertex source, Vertex destination, std::atomic_bool *cancel_token) {
  //--------------------------------------------------------------------------
  // 1. Preprocessing
  //--------------------------------------------------------------------------
  if (preprocess(k, source, destination)) return;

  create_transitive_closure(true, cancel_token);
  remove_colorless_vertices({source, destination});

  auto const &g = get_graph();
  int num_vertices = g.number_of_vertices();
  int num_colors = g.number_of_colors();
  if (g.number_of_colors() > 32) throw std::runtime_error("Not implemented: |C| > 32");

  // re-index colors
  compact_colors();

  // initialize graph properties
  impl::initialize_graph(this, source, destination);

  //--------------------------------------------------------------------------
  // 2. Main Computation
  //--------------------------------------------------------------------------
  Weight best_weight = get_solution_weight();

  // set up tables
  impl::dp_table.clear();
  impl::dp_table.resize(num_vertices, std::vector<impl::Value>(1UL << num_colors, std::make_pair(INFINITY, std::nullopt)));
  for (int i = 0; i < 2; ++i) {
    impl::dp_active[i].clear();
    impl::dp_active[i].resize(num_vertices);
  }

  // add initial entry (assuming source has no colors)
  int previous_layer = 1, current_layer = 0;
  impl::dp_table[impl::idx_source][0] = std::make_pair(0.0, std::nullopt);
  impl::dp_active[previous_layer][impl::idx_source].insert(0);

  // main loop
  int num_iterations = 0;  // for debugging
  bool updated = true;
  while (updated) {
    updated = false;

    if (util::logging::LogLevel::DEBUG >= util::logging::log_level) {
      // log stats
      std::size_t dp_table_sz = impl::dp_table.size() * impl::dp_table[0].size(), prev_table_sz = 0;
      for (auto &t : impl::dp_active[previous_layer]) prev_table_sz += t.size();

      log_debug("%s: iteration=%d, dp_table_size=%lu, active_table_size=%lu", get_solver_name(), num_iterations,
                dp_table_sz, prev_table_sz);
    }

    // parallelize over vertices
#pragma omp parallel num_threads(num_threads_)
    {
#pragma omp for schedule(dynamic, 1)            // chunk-size=1
      for (int i = 0; i < num_vertices; ++i) {  // vertex loop
        // look for in-neighbor's new information
        for (auto j : impl::in_neighbors[i]) {
          if (!cancel_token || !cancel_token->load()) {  // check for timeout
            for (auto j_colors : impl::dp_active[previous_layer][j]) {
              auto new_colors = j_colors | impl::vertex_colors[i];
              if (i == impl::idx_destination) {
                // go to destination only in the end
                if (!impl::is_solution(k, num_colors, new_colors)) continue;
              } else {
                // not a color-collecting walk; reject
                if (j_colors == new_colors) continue;
              }

              auto new_weight = impl::dp_table[j][j_colors].first + impl::weight_matrix[j][i];
              auto current_weight = impl::dp_table[i][new_colors].first;

              if (new_weight < current_weight) {
                // update information
                updated = true;
                auto new_value = std::make_pair(new_weight, impl::key(j, j_colors));
                impl::dp_table[i][new_colors] = new_value;
                impl::dp_active[current_layer][i].insert(new_colors);

                // check for the stop condition
                if (i == impl::idx_destination && impl::is_solution(k, num_colors, new_colors)) {
                  if (new_weight < best_weight) {
                    // backtrack soluition walk
                    best_weight = new_weight;
                    set_critical_walk(impl::construct_solution(impl::idx_destination, new_colors));
                  }
                }
              }
            }
          }
        }
      }  // end vertex loop
    }    // end parallel region

    // handle timeout
    if (cancel_token && cancel_token->load()) { throw std::runtime_error("canceled: NaiveDPSolver2::solve_main()"); }

    // switch active layers
    for (auto &entry : impl::dp_active[previous_layer]) entry.clear();
    std::swap(current_layer, previous_layer);
    ++num_iterations;
  }
}
}  // namespace exact
}  // namespace algorithm
