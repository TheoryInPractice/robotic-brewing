#include <memory>

#include "algorithm/exact/AlgebraicSolver.hpp"
#include "algorithm/exact/DPSolver.hpp"
#include "algorithm/exact/ILPSolver.hpp"
#include "algorithm/exact/NaiveDPSolver.hpp"
#include "algorithm/exact/NaiveDPSolver2.hpp"
#include "app/solver/SolverConfiguration.hpp"
#include "readwrite/iris.hpp"
#include "readwrite/pace.hpp"
#include "util/Profiler.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

using namespace std;
using namespace app::solver;

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
  app::solver::SolverConfiguration conf(argc, argv);
  if (conf.exit) return conf.status_code;

  // configure logging
  util::set_log_level(conf.log_level);  // log level
  util::set_colored_log(!conf.disable_colored_log);
  log_info("Output format: Time(s) Coverage WalkLength NumberOfHops%s", conf.no_cert ? "" : " Walk");

  // pseudorandom number generator
  util::Random rand(conf.seed);
  log_info("Configuration: seed=%u, #threads=%d, time-limit=%s", conf.seed, conf.num_threads,
           conf.time_limit > 0 ? util::format("%ds", conf.time_limit).c_str() : "N/A");

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

  auto num_colors = instance.graph.number_of_colors();
  log_info("Loaded graph (n=%ld, m=%ld, |C|=%ld): %s", instance.graph.number_of_vertices(),
           instance.graph.number_of_edges(), num_colors, conf.input_path.c_str());

  // override parameters
  if (conf.k_value >= 0) instance.k = conf.k_value;
  if (conf.s_value >= 0) instance.s = conf.s_value;
  if (conf.t_value >= 0) instance.t = conf.t_value;

  if (instance.k < 0) {
    log_warning_("k-value is not set; will collect all colors");
    instance.k = num_colors;
  }
  if (instance.s < 0) {
    log_error_("Source vertex is not set. Use -s option.");
    return 1;
  }
  if (instance.t < 0) {
    log_warning_("Destination vertex is not set; will find a closed walk");
    instance.t = instance.s;
  }

  log_info("Parameters: k=%d, s=%d, t=%d", instance.k, instance.s, instance.t);

  // scale weights
  if (conf.scale != 1.0) {
    log_info("Scaling edge weights by: %.10f", conf.scale);
    for (auto& e : instance.graph.edges()) {
      auto w = instance.graph.get_weight(e.first, e.second) * conf.scale;
      instance.graph.set_weight(e.first, e.second, w);
    }
  }

  // create the solver
  std::unique_ptr<algorithm::base::BaseSolver> solver = nullptr;

  switch (conf.alg) {
    case app::solver::Algorithm::DPSolver: {
      log_info_("Using algorithm: DPSolver");
      solver = std::make_unique<algorithm::exact::DPSolver>(instance.graph, conf.num_threads, true, !conf.no_cert, conf.dp_epsilon,
                                                            conf.dp_rho, conf.dp_scaling, conf.dp_transitive_closure);
      break;
    }
    case app::solver::Algorithm::ILPSolver: {
      log_info_("Using algorithm: ILPSolver");
      solver = std::make_unique<algorithm::exact::ILPSolver>(instance.graph, conf.seed, conf.num_threads,
                                                             conf.ilp_output_gurobi_log, true, !conf.no_cert, 1);
      break;
    }
    case app::solver::Algorithm::ILPSolverV2: {
      log_info_("Using algorithm: ILPSolverV2");
      solver = std::make_unique<algorithm::exact::ILPSolver>(instance.graph, conf.seed, conf.num_threads,
                                                             conf.ilp_output_gurobi_log, true, !conf.no_cert, 2);
      break;
    }
    case app::solver::Algorithm::AlgebraicSolver: {
      log_info_("Using algorithm: AlgebraicSolver");
      solver = std::make_unique<algorithm::exact::AlgebraicSolver>(  //
          instance.graph, rand, conf.num_threads, conf.algebraic_num_failures, conf.algebraic_recover_all,
          conf.algebraic_search_strategy, conf.algebraic_compact_level, true, !conf.no_cert);
      break;
    }
    case app::solver::Algorithm::NaiveDPSolver: {
      if (conf.dp_transitive_closure) {
        log_info_("Using algorithm: NaiveDPSolver2");
        solver = std::make_unique<algorithm::exact::NaiveDPSolver2>(  //
            instance.graph, conf.num_threads, true, !conf.no_cert);
      } else {
        log_info_("Using algorithm: NaiveDPSolver");
        solver = std::make_unique<algorithm::exact::NaiveDPSolver>(  //
            instance.graph, conf.num_threads, true, !conf.no_cert, conf.dp_transitive_closure);
      }
      break;
    }
    default: {
      throw std::invalid_argument("never happens");
    }
  }

#ifdef NDEBUG
  try {
#endif
    // run the solver
    bool resolved = solver->solve(instance.k, instance.s, instance.t, conf.time_limit);

    // report results
    auto elapsed = timer.stop();
    if (resolved) {
      if (solver->is_feasible()) {
        log_success(
            "Found an optimal solution: objective=%.10f, #colors=%lu, #hops=%lu, k=%d, s=%d, t=%d, elapsed=%.3fs",
            solver->get_solution_weight(), solver->get_solution_colors().size(), solver->get_solution().size(),
            instance.k, instance.s, instance.t, elapsed);
      }
    } else {
      log_debug_("Exiting due to time limit");
    }
#ifdef NDEBUG
  } catch (std::invalid_argument const& e) {
    log_error("Solver error: invalid_argument: %s", e.what());
    return 2;
  } catch (std::exception const& e) {  //
    log_critical("Solver error: %s", e.what());
    return 2;
  }
#endif

  // profiler
  util::prof.print();
  return 0;
}
