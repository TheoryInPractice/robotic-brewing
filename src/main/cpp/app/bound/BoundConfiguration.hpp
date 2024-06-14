#pragma once

#include <CLI11/CLI11.hpp>

#include "util/logger.hpp"
#include "util/util.hpp"

namespace app {
namespace bound {
char const* PROGRAM_NAME = "compute-bounds";
char const* PROGRAM_VERSION = "0.0.1-SNAPSHOT";

/** Choice of readers*/
enum Reader {
  Auto,
  PACE,
  IRIS,
};

/**
 * @brief Handles command-line arguments and configurations.
 */
class BoundConfiguration {
 public:
  bool exit = false;  // Immediately exit after parsing.
  int status_code = 0;

  // parameters and flags
  util::logging::LogLevel log_level = util::logging::LogLevel::INFO;
  bool disable_colored_log = false;

  std::string input_path;
  Reader reader = Reader::Auto;

  uint32_t seed = 12345;
  int num_threads = 1;
  int s_value = -1;
  int t_value = -1;

  int k_start = 0;
  int k_stop = -1;
  int k_step = 1;

  bool upper_bound_only = false;

  BoundConfiguration(int argc, char* argv[]) {
    // parse arguments
    CLI::App app{"Finds lower and upper bounds for Inspection Planning for every possible k."};

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
    app.add_option("--seed", seed, "Random seed")->option_text("SEED");
    app.add_option("-j", num_threads, "Number of threads");
    app.add_option("-s", s_value, "Value of s: source vertex (default:0)");
    app.add_option("-t", t_value, "Value of t: destination vertex");
    app.add_option("--k-start", k_start, "start of the range of k");
    app.add_option("--k-stop", k_stop, "stop of the range of k");
    app.add_option("--k-step", k_step, "step of the range of k");
    app.add_flag("--upper-bound-only", upper_bound_only, "Compute only upper bounds.");

    try {
      app.parse(argc, argv);
    } catch (CLI::ParseError const& e) {
      exit = true;
      status_code = app.exit(e);
    }
  }

 private:
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
}  // namespace bound
}  // namespace app
