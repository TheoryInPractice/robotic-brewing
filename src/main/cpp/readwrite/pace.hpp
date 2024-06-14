#pragma once
#include <fstream>
#include <sstream>

#include "ds/ProblemInstance.hpp"

namespace readwrite {
ds::ProblemInstance read_pace(std::istream &is);

ds::ProblemInstance load_pace(char const *path);
}  // namespace readwrite
