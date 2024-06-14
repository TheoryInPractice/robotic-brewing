#pragma once
#include <fstream>
#include <sstream>
#include <vector>

namespace readwrite {

/**
 * @brief Reads walks.
 *
 * @param is input stream
 * @return std::vector<std::vector<int>> walks
 */
std::vector<std::vector<int>> read_walks(std::istream &is);

/**
 * @brief Loads walks from file.
 *
 * @param path path to the input file
 * @return std::vector<std::vector<int>> walks
 */
std::vector<std::vector<int>> load_walks(char const *path);
}  // namespace readwrite
