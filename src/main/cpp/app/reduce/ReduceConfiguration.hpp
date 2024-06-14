#pragma once

#include <CLI11/CLI11.hpp>

#include "ds/map/Bimap.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace app {
namespace reduce {
char const* PROGRAM_NAME = "reduce-poi";
char const* PROGRAM_VERSION = "0.0.1-SNAPSHOT";

/** Choices of algorithms. */
enum Algorithm { Greedy, Random, KCenter, Cluster };

/**
 * @brief Handles command-line arguments and configurations.
 */
class ReduceConfiguration {
 public:
  bool exit = false;  // Immediately exit after parsing.
  int status_code = 0;

  // parameters and flags
  util::logging::LogLevel log_level = util::logging::LogLevel::INFO;
  bool disable_colored_log = false;

  std::string graph_path;
  std::string poi_path;
  uint32_t seed = 12345;
  int k_value = -1;
  bool sort_output = false;
  int scale = 2;
  int splits = 1;
  int source_vertex = 0;

  algorithm::reduce::PartitionAlgorithm partition_algorithm = algorithm::reduce::PartitionAlgorithm::Ordered;
  algorithm::reduce::PartitionTiming partition_timing = algorithm::reduce::PartitionTiming::AfterReduction;

  /** For algorithms (greedy, cluster, kcenter):
   *    true: randomly pick the first set.
   *    false: use POIs at vertex 0 as the starting set.
   **/
  bool kcenter_random_start = false;

  Algorithm alg = Greedy;

  ReduceConfiguration(int argc, char* argv[]) {
    // parse arguments
    CLI::App app{"POI Reducer for Inspection Planning"};

#ifdef NDEBUG
    std::string version = util::format("%s %s", PROGRAM_NAME, PROGRAM_VERSION);
#else
    std::string version = util::format("%s %s-DEBUG", PROGRAM_NAME, PROGRAM_VERSION);
#endif

    app.set_version_flag("-v,--version", version, "Print program's version number and exit");
    app.add_option("graph_path", graph_path, "Input graph file prefix")->required();
    app.add_option("poi_path", poi_path, "Input POI position file path")->required();
    app.add_option("-k", k_value, "Value of k: number of reduced POIs")->required();
    app.add_option("-s", scale, "Scale factor for the number of centers (default:2)");
    app.add_option("--splits", splits, "Number of splits to sort the POIs (default:1)");
    app.add_option("-l,--log-level", log_level, util::format("Set log level to one of [%s]", log_level_options))
        ->option_text("LEVEL")
        ->transform(CLI::CheckedTransformer(util::logging::log_level_map, CLI::ignore_case));
    app.add_flag("--no-color", disable_colored_log, "Disable colored logging");
    app.add_option("-a", alg, util::format("Algorithm to use; one of [%s] (default:greedy)", algorithm_options))
        ->option_text("ALGORITHM")
        ->transform(CLI::CheckedTransformer(alg_map_.right(), CLI::ignore_case));
    app.add_option("--seed", seed, "Random seed")->option_text("SEED");
    app.add_flag("--sort", sort_output,
                 "Sort output by label for each split if true (default: false -- preserve algorithm's ordering)");
    app.add_flag("--kcenter-random-start", kcenter_random_start,
                 "For algorithms using k-center: randomly pick the first POI if true (default: false -- start with "
                 "vertex 0's POIs)");
    app.add_option("--part-alg", partition_algorithm,
                   util::format("Partitioning algoirithm; one of [%s] (default:%s)", part_alg_options, part_alg().c_str()))
        ->option_text("PART_ALGORITHM")
        ->transform(CLI::CheckedTransformer(part_alg_map_.right(), CLI::ignore_case));
    app.add_option("--part-timing", partition_timing,
                   util::format("Partitioning timing; one of [%s] (default:%s)", part_time_options, part_timing().c_str()))
        ->option_text("PART_TIMING")
        ->transform(CLI::CheckedTransformer(part_time_map_.right(), CLI::ignore_case));
    app.add_option("--source", source_vertex, "Source vertex (default:0)");

    try {
      app.parse(argc, argv);
    } catch (CLI::ParseError const& e) {
      exit = true;
      status_code = app.exit(e);
    }
  }

  std::string algorithm() const { return alg_map_.g(alg); }
  std::string part_alg() const { return part_alg_map_.g(partition_algorithm); }
  std::string part_timing() const { return part_time_map_.g(partition_timing); }

 private:
  // Update here when a new algorithm is introduced.
  ds::map::Bimap<std::string, Algorithm> alg_map_ = std::vector<std::pair<std::string, Algorithm>>{
      {"greedy", Greedy}, {"random", Random}, {"kcenter", KCenter}, {"cluster", Cluster}};
  char const* algorithm_options = "greedy, random, kcenter, cluster";

  ds::map::Bimap<std::string, algorithm::reduce::PartitionAlgorithm> part_alg_map_ =
      std::vector<std::pair<std::string, algorithm::reduce::PartitionAlgorithm>>{
          {"ordered", algorithm::reduce::PartitionAlgorithm::Ordered},
          {"geometric", algorithm::reduce::PartitionAlgorithm::Geometric}};
  char const* part_alg_options = "ordered, geometric";

  ds::map::Bimap<std::string, algorithm::reduce::PartitionTiming> part_time_map_ =
      std::vector<std::pair<std::string, algorithm::reduce::PartitionTiming>>{
          {"before", algorithm::reduce::PartitionTiming::BeforeReduction},
          {"after", algorithm::reduce::PartitionTiming::AfterReduction}};
  char const* part_time_options = "before, after";

#ifdef NDEBUG
  char const* log_level_options = "none, success, critical, error, warning, info, debug";
#else
  char const* log_level_options = "none, success, critical, error, warning, info, debug, trace";
#endif
};
}  // namespace reduce
}  // namespace app
