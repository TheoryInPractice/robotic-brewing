#pragma once

#include <CLI11/CLI11.hpp>

#include "util/logger.hpp"
#include "util/util.hpp"

namespace app {
namespace solver {
char const* PROGRAM_NAME = "ip-solver";
char const* PROGRAM_VERSION = "0.0.1-SNAPSHOT";

/** Choices of algorithms. */
enum Algorithm {
  DPSolver,
  ILPSolver,
  ILPSolverV2,
  AlgebraicSolver,
  NaiveDPSolver,
};

/** Choice of compact level for algebraic solver. */
enum CompactStrategy {
  Naive,
  Standard,
  SemiCompact,
  Compact,
};

enum SearchStrategy { Basic, Probabilistic, Sequential, MultiOutput };

/** Choice of recovery strategy for algebraic solver. */
enum RecoveryStrategy {
  MC,    // Monte Carlo
  LV,    // Las Vegas
  TP,    // Two phase
  None,  // Skip solution recovery
};

/** Choice of readers*/
enum Reader {
  Auto,
  PACE,
  IRIS,
};

/**
 * @brief Handles command-line arguments and configurations.
 */
class SolverConfiguration {
 public:
  bool exit = false;  // Immediately exit after parsing.
  int status_code = 0;

  // parameters and flags
  util::logging::LogLevel log_level = util::logging::LogLevel::INFO;
  bool disable_colored_log = false;

  std::string input_path;
  uint32_t seed = 12345;
  int num_threads = 1;
  int k_value = -1;
  int s_value = -1;
  int t_value = -1;
  double scale = 1.0;
  int time_limit = 0;
  bool no_cert = false;

  Algorithm alg = DPSolver;
  Reader reader = Reader::Auto;

  // algorithm-specific options
  bool ilp_output_gurobi_log = false;
  int algebraic_num_failures = 30;
  bool algebraic_recover_all = false;  // true: recover a solution any time we find a new upper bound
  algorithm::exact::algebraic::SearchStrategy algebraic_search_strategy = algorithm::exact::algebraic::SearchStrategy::MultiOutput;
  algorithm::exact::algebraic::CompactStrategy algebraic_compact_level = algorithm::exact::algebraic::CompactStrategy::SemiCompact;
  algorithm::exact::algebraic::RecoveryStrategy algebraic_recovery_strategy = algorithm::exact::algebraic::RecoveryStrategy::LV;

  double dp_epsilon = 0;
  double dp_rho = 1;
  double dp_scaling = 0;
  bool dp_transitive_closure = false;

  SolverConfiguration(int argc, char* argv[]) {
    // parse arguments
    CLI::App app{"Solver for Inspection Planning"};

#ifdef NDEBUG
    std::string version = util::format("%s %s", PROGRAM_NAME, PROGRAM_VERSION);
#else
    std::string version = util::format("%s %s-DEBUG", PROGRAM_NAME, PROGRAM_VERSION);
#endif

    app.set_version_flag("-v,--version", version, "Print program's version number and exit");
    app.add_option("input_path", input_path, "Input file path or prefix")->required();
    app.add_option("-r,--reader", reader, util::format("Reader type; one of [%s] (default:auto)", reader_options))
        ->option_text("READER")
        ->transform(CLI::CheckedTransformer(reader_map_, CLI::ignore_case));
    app.add_option("-l,--log-level", log_level, util::format("Set log level to one of [%s]", log_level_options))
        ->option_text("LEVEL")
        ->transform(CLI::CheckedTransformer(util::logging::log_level_map, CLI::ignore_case));
    app.add_flag("--no-color", disable_colored_log, "Disable colored logging");
    app.add_option("-a", alg, util::format("Algorithm to use; one of [%s] (default:dp)", algorithm_options))
        ->option_text("ALGORITHM")
        ->transform(CLI::CheckedTransformer(alg_map_, CLI::ignore_case));
    app.add_option("--seed", seed, "Random seed")->option_text("SEED");
    app.add_option("-j", num_threads, "Number of threads");
    app.add_option("-k", k_value, "Value of k: number of colors to collect");
    app.add_option("-s", s_value, "Value of s: source vertex");
    app.add_option("-t", t_value, "Value of t: destination vertex");
    app.add_option("--scale", scale, "Scale edge weight by SCALE")->option_text("SCALE");
    app.add_option("--time-limit", time_limit, "Time limit in seconds")->option_text("TIME_LIMIT_SEC");
    app.add_flag("--no-cert", no_cert, "Do not print solution walks");

    // For AlgebraicSolver
    app.add_option("--algebraic-num-failures", algebraic_num_failures,
                   "Number of failures to determine infeasibility for AlgebraicSolver")
        ->option_text("ALG_NUM_FAIL");
    app.add_flag("--algebraic-recover-all", algebraic_recover_all,
                 "Recover a solution for every new upper bound in AlgebraicSolver");
    app.add_option("--algebraic-search", algebraic_search_strategy,
                   util::format("Search strategy for AlgebraicSolver [%s] (default:multioutput)", search_options))
        ->option_text("ALG_SEARCH")
        ->transform(CLI::CheckedTransformer(search_map_, CLI::ignore_case));
    app.add_option("--algebraic-compact", algebraic_compact_level,
                   util::format("Circuit compact level in AlgebraicSolver [%s] (default:semicompact)", compact_options))
        ->option_text("COMPACT")
        ->transform(CLI::CheckedTransformer(compact_map_, CLI::ignore_case));

    app.add_option("--algebraic-recovery", algebraic_recovery_strategy,
                   util::format("Recovery Strategy in AlgebraicSolver [%s] (default:lv)", recovery_options))
        ->option_text("RECOVERY")
        ->transform(CLI::CheckedTransformer(recovery_map_, CLI::ignore_case));
    // For ILP
    app.add_flag("--ilp-output-gurobi-log", ilp_output_gurobi_log, "Print Gurobi Optimizer's output");

    // For DP
    app.add_option("--dp-epsilon", dp_epsilon, "Epsilon for DPSolver")->option_text("DP_EPSILON");
    app.add_option("--dp-rho", dp_rho, "Rho for DPSolver")->option_text("DP_RHO");
    app.add_option("--dp-scaling", dp_scaling, "Scaling factor for approximate DP")->option_text("DP_SCALING");
    app.add_flag("--dp-transitive-closure", dp_transitive_closure, "Remove colorless vertices (NaiveDP/DP)");

    try {
      app.parse(argc, argv);

      if (scale < 0) {  // scale must be non-negative
        log_critical("Scaling factor cannot be negative: %.10f", scale);
        exit = true;
        status_code = 2;
      }
    } catch (CLI::ParseError const& e) {
      exit = true;
      status_code = app.exit(e);
    }
  }

 private:
  // Update here when a new algorithm is introduced.
  std::map<std::string, Algorithm> alg_map_{
      {"dp", DPSolver},                //
      {"ilp", ILPSolver},              //
      {"ilp2", ILPSolverV2},           //
      {"algebraic", AlgebraicSolver},  //
      {"naivedp", NaiveDPSolver}       //
  };
  char const* algorithm_options = "dp, ilp, algebraic, naivedp";

  std::map<std::string, CompactStrategy> compact_map_{
      {"naive", Naive},              //
      {"standard", Standard},        //
      {"semicompact", SemiCompact},  //
      {"compact", Compact},          //
  };
  char const* compact_options = "naive, standard, semicompact, compact";

  std::map<std::string, SearchStrategy> search_map_{
      {"basic", Basic},                  //
      {"probabilistic", Probabilistic},  //
      {"sequential", Sequential},        //
      {"multioutput", MultiOutput}       //
  };
  char const* search_options = "basic, probabilistic, sequential, multioutput";

  std::map<std::string, RecoveryStrategy> recovery_map_{
      {"mc", MC},                        //
      {"lv", LV},                        //
      {"tp", TP},                        //
      {"none", RecoveryStrategy::None},  //
  };
  char const* recovery_options = "mc, lv, tp, none";

  // Update here when a new reader is introduced.
  std::map<std::string, Reader> reader_map_{
      {"auto", Auto},
      {"pace", PACE},
      {"iris", IRIS},
  };
  char const* reader_options = "auto, pace, iris";

#ifdef NDEBUG
  char const* log_level_options = "none, success, critical, error, warning, info, debug";
#else
  char const* log_level_options = "none, success, critical, error, warning, info, debug, trace";
#endif
};
}  // namespace solver
}  // namespace app
