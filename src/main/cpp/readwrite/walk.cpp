#include "walk.hpp"

#include "util/util.hpp"

namespace readwrite {

/**
 * @brief Reads walks.
 *
 * @param is input stream
 * @return std::vector<std::vector<int>> walks
 */
std::vector<std::vector<int>> read_walks(std::istream &is) {
  std::vector<std::vector<int>> ret;
  for (std::string line; std::getline(is, line);) {
    if (line.empty()) continue;

    std::vector<int> walk;
    for (auto &t : util::split(line)) walk.push_back(std::stoi(t));
    if (walk.empty() || walk.front() != walk.back()) throw std::invalid_argument("not a closed walk");
    ret.push_back(walk);
  }
  return ret;
}

/**
 * @brief Loads walks from file.
 *
 * @param path path to the input file
 * @return std::vector<std::vector<int>> walks
 */
std::vector<std::vector<int>> load_walks(char const *path) {
  std::ifstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return read_walks(f);
}

}  // namespace readwrite
