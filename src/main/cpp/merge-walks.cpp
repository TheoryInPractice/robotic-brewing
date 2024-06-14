#include <memory>
#include <optional>

#include "algorithm/merge/ConcatMerger.hpp"
#include "algorithm/merge/ExactMerger.hpp"
#include "algorithm/merge/HeuristicMerger.hpp"
#include "app/merge/MergeConfiguration.hpp"
#include "ds/set/FastSet.hpp"
#include "readwrite/iris.hpp"
#include "readwrite/pace.hpp"
#include "readwrite/walk.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

using namespace std;
using namespace ds::graph;
using namespace app::merge;

static std::optional<ds::ProblemInstance> load_graph(MergeConfiguration const& conf) {
  // load input file(s)
  auto suffix = CLI::detail::to_lower(std::filesystem::path(conf.graph_path).extension());  // requires C++17
  auto reader = conf.reader == Reader::Auto ? suffix == ".gr" ? Reader::PACE : Reader::IRIS : conf.reader;
  ds::ProblemInstance instance;
  try {
    switch (reader) {
      case Reader::IRIS: {
        instance = readwrite::load_iris(conf.graph_path.c_str());
        break;
      }
      case Reader::PACE: {
        instance = readwrite::load_pace(conf.graph_path.c_str());
        break;
      }
      default: {
        throw std::invalid_argument("unsupported reader type");
      }
    }
  } catch (std::invalid_argument const& e) {
    log_error("File format error: %s", e.what());
    return std::nullopt;
  }

  log_info("Loaded graph (n=%ld, m=%ld, |C|=%ld, |C_0|=%ld): %s", instance.graph.number_of_vertices(),
           instance.graph.number_of_edges(), instance.graph.number_of_colors(), instance.graph.get_colors(0).size(),
           conf.graph_path.c_str());
  return instance;
}

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
  app::merge::MergeConfiguration conf(argc, argv);
  if (conf.exit) return conf.status_code;

  // configure logging
  util::set_log_level(conf.log_level);  // log level
  util::set_colored_log(!conf.disable_colored_log);

  // pseudorandom number generator
  util::Random rand(conf.seed);
  log_info("Configuration: algorithm=%s, #threads=%d, seed=%d", conf.algorithm().c_str(), conf.num_threads, conf.seed);

#ifdef NDEBUG
  try {
#endif
    // load graph
    auto instance_ret = load_graph(conf);
    if (!instance_ret.has_value()) return 2;
    ds::ProblemInstance instance = instance_ret.value();

    // load walks
    auto input_walks = readwrite::load_walks(conf.walk_path.c_str());
    log_info("Loaded walks (#walks=%lu): %s", input_walks.size(), conf.walk_path.c_str());

    // create merger
    std::unique_ptr<algorithm::merge::BaseMerger> merger = nullptr;

    switch (conf.alg) {
      case app::merge::Algorithm::Concatenation: {
        merger = std::make_unique<algorithm::merge::ConcatMerger>();
        break;
      }
      case app::merge::Algorithm::Heuristic: {
        merger = std::make_unique<algorithm::merge::HeuristicMerger>();
        break;
      }
      case app::merge::Algorithm::Exact: {
        merger = std::make_unique<algorithm::merge::ExactMerger>(conf.num_threads, conf.seed);
        break;
      }
      default: {
        throw std::invalid_argument("never happens");
      }
    }

    // run algorithms
    auto final_walk = merger->merge(instance.graph, input_walks);

    // output result
    double weight = 0.0;
    for (std::size_t i = 0; i < final_walk.size(); ++i) {
      weight += i == 0 ? 0.0 : instance.graph.get_weight(final_walk[i - 1], final_walk[i]);
      printf("%s%d", i == 0 ? "" : " ", final_walk[i]);
    }
    printf("\n");

    log_success("Merged walks: algorithm=%s, objective=%.10f, #hops=%lu, elapsed=%.3fs", conf.algorithm().c_str(),
                weight, final_walk.size(), timer.stop());

#ifdef NDEBUG
  } catch (std::invalid_argument const& e) {
    log_error("Merger error: invalid_argument: %s", e.what());
    return 2;
  } catch (std::exception const& e) {
    log_critical("Merger error: %s", e.what());
    return 2;
  }
#endif
  return 0;
}
