#include <gtest/gtest.h>

#include "algorithm/merge/ConcatMerger.hpp"
#include "readwrite/pace.hpp"
#include "util/logger.hpp"

using namespace std;
using namespace algorithm::merge;

using VI = vector<int>;
using VVI = vector<VI>;

TEST(ConcatMergerTest, Merge) {
  auto g = readwrite::load_pace("src/test/resources/instances/001_tiny.gr").graph;
  ConcatMerger merger;

  VVI walks = {{0, 1, 0}, {0, 3, 0}, {0, 3, 4, 3, 0}};

  util::set_log_level(util::logging::LogLevel::NONE);
  auto merged = merger.merge(g, walks);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(merged, VI({0, 1, 0, 3, 0, 3, 4, 3, 0}));
}
