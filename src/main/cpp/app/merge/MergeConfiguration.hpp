#pragma once

#include <CLI11/CLI11.hpp>

#include "ds/map/Bimap.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace app {
namespace merge {
char const* PROGRAM_NAME = "merge-walks";
char const* PROGRAM_VERSION = "0.0.1-SNAPSHOT";

/** Choices of algorithms. */
enum Algorithm { Concatenation, Heuristic, Exact };

/** Choice of readers*/
enum Reader { Auto, PACE, IRIS };

/**
 * @brief Handles command-line arguments and configurations.
 */
class MergeConfiguration {
 public:
  bool exit = false;  // Immediately exit after parsing.
  int status_code = 0;

  // parameters and flags
  util::logging::LogLevel log_level = util::logging::LogLevel::INFO;
  bool disable_colored_log = false;

  std::string graph_path;
  std::string walk_path;
  uint32_t seed = 12345;
  int num_threads = 1;

  Algorithm alg = Heuristic;
  Reader reader = Reader::Auto;

  MergeConfiguration(int argc, char* argv[]) {
    // parse arguments
    CLI::App app{"Walk Merger for Inspection Planning"};

#ifdef NDEBUG
    std::string version = util::format("%s %s", PROGRAM_NAME, PROGRAM_VERSION);
#else
    std::string version = util::format("%s %s-DEBUG", PROGRAM_NAME, PROGRAM_VERSION);
#endif

    app.set_version_flag("-v,--version", version, "Print program's version number and exit");
    app.add_option("graph_path", graph_path, "Input graph file prefix")->required();
    app.add_option("walk_path", walk_path, "Input walk file path")->required();
    app.add_option("-r,--reader", reader, util::format("Reader type; one of [%s] (default:auto)", reader_options))
        ->option_text("READER")
        ->transform(CLI::CheckedTransformer(reader_map_, CLI::ignore_case));
    app.add_option("-l,--log-level", log_level, util::format("Set log level to one of [%s]", log_level_options))
        ->option_text("LEVEL")
        ->transform(CLI::CheckedTransformer(util::logging::log_level_map, CLI::ignore_case));
    app.add_flag("--no-color", disable_colored_log, "Disable colored logging");
    app.add_option("-a", alg, util::format("Algorithm to use; one of [%s] (default:heuristic)", algorithm_options))
        ->option_text("ALGORITHM")
        ->transform(CLI::CheckedTransformer(alg_map_.right(), CLI::ignore_case));
    app.add_option("--seed", seed, "Random seed")->option_text("SEED");
    app.add_option("-j", num_threads, "Number of threads");

    try {
      app.parse(argc, argv);
    } catch (CLI::ParseError const& e) {
      exit = true;
      status_code = app.exit(e);
    }
  }

  std::string algorithm() const { return alg_map_.g(alg); }

 private:
  // Update here when a new algorithm is introduced.
  ds::map::Bimap<std::string, Algorithm> alg_map_ =
      std::vector<std::pair<std::string, Algorithm>>{{"concat", Concatenation}, {"heuristic", Heuristic}, {"exact", Exact}};
  char const* algorithm_options = "concat, heuristic, exact";

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
}  // namespace merge
}  // namespace app
