#include <gtest/gtest.h>
#include <set>
#include <unordered_map>
#include <vector>

#include "algorithm/reduce/RandomReducer.hpp"
#include "geometry/Point.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

using namespace std;
using namespace algorithm::reduce;
using namespace geometry;

using VI = vector<int>;
using VVI = vector<VI>;

TEST(RandomReducerTest, ReduceCorrectSize) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 6, 0);
  pos[1] = Point(4.1, 4.1, 0);
  pos[2] = Point(0, 0.9, 0);
  pos[3] = Point(3, 3, 0);
  pos[4] = Point(1, 0, 0);
  pos[5] = Point(0, 0, 0);
  pos[6] = Point(-4.8, 0, 0);
  pos[7] = Point(0, -4.7, 0);
  pos[8] = Point(3, -3, 0);
  pos[9] = Point(5, 0, 0);
  pos[10] = Point(100, 100, 100);

  auto g = ds::graph::Graph(
      {
          {0, {}}, {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}},  // 10 colors
      },
      {});

  util::Random rand(12345);
  util::set_log_level(util::logging::LogLevel::NONE);
  RandomReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction);

  for (int k = 0; k <= 10; ++k) {
    VI reduced = reducer.reduce(k, 1, rand)[0];
    sort(reduced.begin(), reduced.end());
    EXPECT_EQ(reduced.size(), k);

    set<int> unique_pois(reduced.begin(), reduced.end());
    EXPECT_EQ(unique_pois.size(), reduced.size());
    for (int poi : reduced) { EXPECT_TRUE(0 <= poi && poi < 10); }
  }
  util::set_log_level(util::logging::LogLevel::TRACE);
}
