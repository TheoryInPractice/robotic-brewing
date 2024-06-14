#pragma once
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "geometry/Point.hpp"

namespace readwrite {

/**
 * @brief Reads positions of points of interest (POIs).
 *
 * @param is input stream
 * @return std::unordered_map<int, geometry::Point> POI positions
 */
std::unordered_map<int, geometry::Point> read_poi_position(std::istream &is);

/**
 * @brief Loads positions of points of interest (POIs) from file.
 *
 * @param path path to the input file
 * @return std::unordered_map<int, geometry::Point> POI positions
 */
std::unordered_map<int, geometry::Point> load_poi_position(char const *path);

/**
 * @brief Reads a list of important points of interest (POIs).
 *
 * @param is input stream
 * @return std::vector<int> list of POI indices
 */
std::vector<int> read_poi_list(std::istream &is);

/**
 * @brief Loads a list of important points of interest (POIs).
 *
 * @param path path to the input file
 * @return std::vector<int> list of POI indices
 */
std::vector<int> load_poi_list(char const *path);

/**
 * @brief Writes a list of important points of interest (POIs).
 *
 * @param os output stream
 * @param poi list of POI indices
 */
void write_poi_list(std::ostream &os, std::vector<int> const &poi);

/**
 * @brief Saves a list of important points of interest (POIs) to file.
 *
 * @param path path to the output file
 * @param poi list of POI indices
 */
void save_poi_list(char const *path, std::vector<int> const &poi);
}  // namespace readwrite
