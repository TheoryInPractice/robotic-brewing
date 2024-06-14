#pragma once

#include <CLI11/CLI11.hpp>

#include "util/logger.hpp"
#include "util/util.hpp"

namespace app {
namespace filter {
char const* PROGRAM_NAME = "filter-poi";
char const* PROGRAM_VERSION = "0.0.1-SNAPSHOT";

/**
 * @brief Handles command-line arguments and configurations.
 */
class FilterConfiguration {
 public:
  bool exit = false;  // Immediately exit after parsing.
  int status_code = 0;

  // parameters and flags
  util::logging::LogLevel log_level = util::logging::LogLevel::INFO;
  bool disable_colored_log = false;

  std::string vertex_path;
  std::string poi_path;

  FilterConfiguration(int argc, char* argv[]) {
    // parse arguments
    CLI::App app{"POI Filter for Inspection Planning"};

#ifdef NDEBUG
    std::string version = util::format("%s %s", PROGRAM_NAME, PROGRAM_VERSION);
#else
    std::string version = util::format("%s %s-DEBUG", PROGRAM_NAME, PROGRAM_VERSION);
#endif

    app.set_version_flag("-v,--version", version, "Print program's version number and exit");
    app.add_option("vertex_path", vertex_path, "Input vertex file path")->required();
    app.add_option("poi_path", poi_path, "Input POI list file path")->required();

    app.add_option("-l,--log-level", log_level, util::format("Set log level to one of [%s]", log_level_options))
        ->option_text("LEVEL")
        ->transform(CLI::CheckedTransformer(util::logging::log_level_map, CLI::ignore_case));
    app.add_flag("--no-color", disable_colored_log, "Disable colored logging");

    try {
      app.parse(argc, argv);
    } catch (CLI::ParseError const& e) {
      exit = true;
      status_code = app.exit(e);
    }
  }

 private:
#ifdef NDEBUG
  char const* log_level_options = "none, success, critical, error, warning, info, debug";
#else
  char const* log_level_options = "none, success, critical, error, warning, info, debug, trace";
#endif
};
}  // namespace filter
}  // namespace app
