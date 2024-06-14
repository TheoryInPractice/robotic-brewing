#include <memory>

#include "app/filter/FilterConfiguration.hpp"
#include "ds/set/FastSet.hpp"
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
  app::filter::FilterConfiguration conf(argc, argv);
  if (conf.exit) return conf.status_code;

  // configure logging
  util::set_log_level(conf.log_level);  // log level
  util::set_colored_log(!conf.disable_colored_log);

#ifdef NDEBUG
  try {
#endif
    // load POI list
    auto to_keep = readwrite::load_poi_list(conf.poi_path.c_str());
    log_info("Loaded POI list (K=%ld): %s", to_keep.size(), conf.poi_path.c_str());

    ds::set::FastSet fs;

    // sort and register POIs to keep
    std::sort(to_keep.begin(), to_keep.end());
    if (!to_keep.empty()) fs.resize(to_keep.back() + 1);
    for (auto x : to_keep) fs.set(x);

    // load vertex file
    std::ifstream f(conf.vertex_path.c_str());
    if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", conf.vertex_path.c_str()));
    log_info("Loaded vertex file: %s", conf.vertex_path.c_str());

    int n = 0;
    for (std::string line; std::getline(f, line);) {
      if (line.empty()) continue;

      ++n;
      auto tokens = util::split(line);

      // index, time_vis, time_build
      std::string output = util::format("%s %s %s", tokens[0].c_str(), tokens[1].c_str(), tokens[2].c_str());

      // POIs
      for (std::size_t i = 3; i < tokens.size(); ++i) {
        if (tokens[i].empty()) continue;

        int x = std::stoi(tokens[i]);
        if (x >= static_cast<int>(fs.capacity()) || !fs.get(x)) continue;  // filter out
        output += util::format(" %d", x);
      }
      output += "\n";
      printf("%s", output.c_str());
    }

    log_success("Filtered POIs: n=%d, K=%ld, elapsed=%.3fs", n, to_keep.size(), timer.stop());
#ifdef NDEBUG
  } catch (std::invalid_argument const& e) {
    log_error("Filter error: invalid_argument: %s", e.what());
    return 2;
  } catch (std::exception const& e) {  //
    log_critical("Filter error: %s", e.what());
    return 2;
  }
#endif
  return 0;
}
