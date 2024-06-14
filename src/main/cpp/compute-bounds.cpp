#include <memory>

#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/ilp/ILPRunner.hpp"
#include "app/bound/BoundConfiguration.hpp"
#include "readwrite/iris.hpp"
#include "readwrite/pace.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

using namespace std;
using namespace app::bound;

// dummy class
struct Solver : public algorithm::base::BaseSolver {
  Solver(ds::graph::Graph const& g) : algorithm::base::BaseSolver("Solver", g) {}

  bool solve(int k, ds::graph::Graph::Vertex s, ds::graph::Graph::Vertex t, int time_limit = 0) override {
    return true;
  }
};

/**
 * @brief Entry point of the program.
 *
 * @param argc argument count
 * @param argv argument strings
 * @return int status code
 */
int main(int argc, char* argv[]) {
  util::Timer timer;

  // parse args
  app::bound::BoundConfiguration conf(argc, argv);
  if (conf.exit) return conf.status_code;

  // configure logging
  util::set_log_level(conf.log_level);  // log level
  util::set_colored_log(!conf.disable_colored_log);

  // pseudorandom number generator
  util::Random rand(conf.seed);
  log_info("Configuration: seed=%d, #threads=%d", conf.seed, conf.num_threads);

  // load input file(s)
  auto suffix = CLI::detail::to_lower(std::filesystem::path(conf.input_path).extension());  // requires C++17
  auto reader = conf.reader == Reader::Auto ? suffix == ".gr" ? Reader::PACE : Reader::IRIS : conf.reader;
  ds::ProblemInstance instance;
  try {
    switch (reader) {
      case Reader::IRIS: {
        instance = readwrite::load_iris(conf.input_path.c_str());
        break;
      }
      case Reader::PACE: {
        instance = readwrite::load_pace(conf.input_path.c_str());
        break;
      }
      default: {
        throw std::invalid_argument("unsupported reader type");
      }
    }
  } catch (std::invalid_argument const& e) {
    log_error("File format error: %s", e.what());
    return 2;
  }

  int num_colors = instance.graph.number_of_colors();
  log_info("Loaded graph (n=%ld, m=%ld, |C|=%d): %s", instance.graph.number_of_vertices(),
           instance.graph.number_of_edges(), num_colors, conf.input_path.c_str());

  // override parameters
  if (conf.s_value >= 0) instance.s = conf.s_value;
  if (conf.t_value >= 0) instance.t = conf.t_value;

  if (instance.s < 0) { log_warning_("Source vertex is not set; will default to 0.") instance.s = 0; }

  if (instance.t < 0) {
    log_warning_("Destination vertex is not set; will find a closed walk.");
    instance.t = instance.s;
  }

  int ilp_version = 2;

#ifdef NDEBUG
  try {
#endif
    // preprocessing
    auto solver = Solver(instance.graph);  // dummy solver
    solver.remove_unreachable_vertices(instance.s);
    num_colors = solver.get_graph().number_of_colors();
    int num_terminal_colors = solver.remove_terminal_colors({instance.s, instance.t});

    // adjust target range
    int k_start = conf.k_start;
    while (k_start < num_terminal_colors + 1) k_start += conf.k_step;
    int k_stop = conf.k_stop < 0 ? num_colors + 1 : std::min(conf.k_stop, num_colors + 1);

    if (ilp_version == 1) {
      solver.create_transitive_closure(true);
      solver.remove_colorless_vertices({instance.s, instance.t});
    }

    // vectors for stroing results
    vector<double> upper_bounds(num_colors + 1);
    vector<double> lower_bounds(num_colors + 1);

    for (int k = k_start; k < k_stop; k += conf.k_step) {
      int k_effective = k - num_terminal_colors;

      // compute upper bounds
      auto weight = solver.find_steiner_tree_solution(k_effective, instance.s, instance.t);
      upper_bounds[k] = weight;
      log_debug("Found upper bound: k=%d, ub=%.10f", k, upper_bounds[k]);

      if (conf.upper_bound_only) continue;

#if GUROBI_ON
      // We may not run multiple ILPRunner at the same time.
      // find lower bound by LP Relaxation
      int lp_seed = rand.randint(0, 2000000000);
      auto lp_runner = algorithm::ilp::ILPRunner(solver.get_graph(), k_effective, instance.s, instance.t, true,
                                                 conf.num_threads, lp_seed, 0, false, ilp_version);
      lp_runner.run();
      lower_bounds[k] = lp_runner.get_objective();
      log_debug("Found lower bound: k=%d, lb=%.10f", k, lower_bounds[k]);
#endif
    }

    // output result
    for (int k = conf.k_start; k < k_stop; k += conf.k_step) {
      // format: k lower-bound upper-bound
      printf("%d %.10f %.10f\n", k, lower_bounds[k], upper_bounds[k]);
    }

    log_success("Found bounds: %d <= k <= %d, step=%d, elapsed=%.3fs", conf.k_start, k_stop - 1, conf.k_step, timer.stop());

#ifdef NDEBUG
  } catch (std::invalid_argument const& e) {
    log_error("error: invalid_argument: %s", e.what());
    return 2;
  } catch (std::exception const& e) {
    log_critical("error: %s", e.what());
    return 2;
  }
#endif
  return 0;
}
