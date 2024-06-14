#pragma once
#include <fstream>
#include <sstream>

#include "ds/ProblemInstance.hpp"

namespace readwrite {
ds::ProblemInstance read_iris(std::istream &vertex, std::istream &edge);

ds::ProblemInstance load_iris(char const *path);
}  // namespace readwrite