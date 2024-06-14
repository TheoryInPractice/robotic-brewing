#include "poi.hpp"

#include "util/util.hpp"

namespace readwrite {

/**
 * @brief Reads positions of points of interest (POIs).
 *
 * @param is input stream
 * @return std::unordered_map<int, geometry::Point> POI positions
 */
std::unordered_map<int, geometry::Point> read_poi_position(std::istream &is) {
  std::unordered_map<int, geometry::Point> ret;
  int i = 0;
  for (std::string line; std::getline(is, line);) {
    if (line.empty()) continue;
    geometry::Point p;
    std::istringstream(line) >> p;
    ret[i++] = p;
  }
  return ret;
}

/**
 * @brief Loads positions of points of interest (POIs) from file.
 *
 * @param path path to the input file
 * @return std::unordered_map<int, geometry::Point> POI positions
 */
std::unordered_map<int, geometry::Point> load_poi_position(char const *path) {
  std::ifstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return read_poi_position(f);
}

/**
 * @brief Reads a list of important points of interest (POIs).
 *
 * @param is input stream
 * @return std::vector<int> list of POI indices
 */
std::vector<int> read_poi_list(std::istream &is) {
  std::vector<int> ret;
  for (std::string line; std::getline(is, line);) {
    if (line.empty()) continue;
    ret.push_back(std::stoi(line));
  }
  return ret;
}

/**
 * @brief Loads a list of important points of interest (POIs).
 *
 * @param path path to the input file
 * @return std::vector<int> list of POI indices
 */
std::vector<int> load_poi_list(char const *path) {
  std::ifstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return read_poi_list(f);
}

/**
 * @brief Writes a list of important points of interest (POIs).
 *
 * @param os output stream
 * @param poi list of POI indices
 */
void write_poi_list(std::ostream &os, std::vector<int> const &poi) {
  for (auto p : poi) os << p << std::endl;
}

/**
 * @brief Saves a list of important points of interest (POIs) to file.
 *
 * @param path path to the output file
 * @param poi list of POI indices
 */
void save_poi_list(char const *path, std::vector<int> const &poi) {
  std::ofstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return write_poi_list(f, poi);
}

}  // namespace readwrite
