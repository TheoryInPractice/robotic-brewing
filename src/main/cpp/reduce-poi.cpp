#include <memory>

#include "algorithm/reduce/ClusterReducer.hpp"
#include "algorithm/reduce/GreedyReducer.hpp"
#include "algorithm/reduce/KCenterReducer.hpp"
#include "algorithm/reduce/RandomReducer.hpp"
#include "app/reduce/ReduceConfiguration.hpp"
#include "ds/set/FastSet.hpp"
#include "readwrite/iris.hpp"
#include "readwrite/poi.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

using namespace std;

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
  app::reduce::ReduceConfiguration conf(argc, argv);
  if (conf.exit) return conf.status_code;

  // configure logging
  util::set_log_level(conf.log_level);  // log level
  util::set_colored_log(!conf.disable_colored_log);

  // pseudorandom number generator
  util::Random rand(conf.seed);
  log_info("Configuration: K_given=%d, algorithm=%s, seed=%d, part_alg=%s, part_timing=%s", conf.k_value,
           conf.algorithm().c_str(), conf.seed, conf.part_alg().c_str(), conf.part_timing().c_str());

#ifdef NDEBUG
  try {
#endif
    // load POI positions
    auto poi_pos = readwrite::load_poi_position(conf.poi_path.c_str());
    log_info("Loaded POI positions (C=%ld): %s", poi_pos.size(), conf.poi_path.c_str());

    // load graph in IRIS format
    ds::ProblemInstance instance;
    try {
      instance = readwrite::load_iris(conf.graph_path.c_str());
    } catch (std::invalid_argument const& e) {
      log_error("File format error: %s", e.what());
      return 2;
    }
    log_info("Loaded graph (n=%ld, m=%ld, |C|=%ld, |C_0|=%ld): %s", instance.graph.number_of_vertices(),
             instance.graph.number_of_edges(), instance.graph.number_of_colors(), instance.graph.get_colors(0).size(),
             conf.graph_path.c_str());

    // create reducer
    std::unique_ptr<algorithm::reduce::BaseReducer> reducer = nullptr;

    switch (conf.alg) {
      case app::reduce::Algorithm::Random: {
        reducer = std::make_unique<algorithm::reduce::RandomReducer>(  //
            poi_pos, instance.graph, conf.source_vertex, conf.partition_algorithm, conf.partition_timing);
        break;
      }
      case app::reduce::Algorithm::Greedy: {
        reducer = std::make_unique<algorithm::reduce::GreedyReducer>(  //
            poi_pos, instance.graph, conf.source_vertex, conf.partition_algorithm, conf.partition_timing, conf.kcenter_random_start);
        break;
      }
      case app::reduce::Algorithm::KCenter: {
        reducer = std::make_unique<algorithm::reduce::KCenterReducer>(  //
            poi_pos, instance.graph, conf.source_vertex, conf.partition_algorithm, conf.partition_timing, conf.kcenter_random_start);
        break;
      }
      case app::reduce::Algorithm::Cluster: {
        reducer = std::make_unique<algorithm::reduce::ClusterReducer>(  //
            poi_pos, instance.graph, conf.source_vertex, conf.partition_algorithm, conf.partition_timing,
            conf.kcenter_random_start, conf.scale);
        break;
      }
      default: {
        throw std::invalid_argument("never happens");
      }
    }

    // run solver
    auto reduced_clusters = reducer->reduce(conf.k_value, conf.splits, rand);

    // output result
    assert(static_cast<int>(reduced_clusters.size()) == conf.splits);

    for (auto& cluster : reduced_clusters) {
      assert(static_cast<int>(cluster.size()) <= conf.k_value);

      // sort output if the option is specified
      if (conf.sort_output) std::sort(cluster.begin(), cluster.end());

      // write POIs to stdout
      for (auto x : cluster) printf("%d\n", x);
    }

    log_success("Reduced POIs: K=%d, #splits=%d, algorithm=%s, part_alg=%s, part_timing=%s, elapsed=%.3fs", conf.k_value,
                conf.splits, conf.algorithm().c_str(), conf.part_alg().c_str(), conf.part_timing().c_str(), timer.stop());

#ifdef NDEBUG
  } catch (std::invalid_argument const& e) {
    log_error("Reducer error: invalid_argument: %s", e.what());
    return 2;
  } catch (std::exception const& e) {
    log_critical("Reducer error: %s", e.what());
    return 2;
  }
#endif
  return 0;
}
