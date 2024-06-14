#include <gtest/gtest.h>
#include <set>
#include <unordered_map>
#include <vector>

#include "algorithm/reduce/ClusterReducer.hpp"
#include "geometry/Point.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

using namespace std;
using namespace algorithm::reduce;
using namespace geometry;

typedef vector<int> VI;

TEST(ClusterReducerTest, ReduceCorrectSize) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 0, 0);
  pos[1] = Point(0.1, 0.1, 0);
  pos[2] = Point(0.2, 0.2, 0);
  pos[3] = Point(3, 3, 0);
  pos[4] = Point(3.1, 3.1, 0);
  pos[5] = Point(3.2, 3.2, 0);
  pos[6] = Point(6, 6, 0);
  pos[7] = Point(6.1, 6.1, 0);
  pos[8] = Point(6.2, 6.2, 0);

  // graph with 9 poi and 1 edge with weight 1
  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8}},
      },
      {
          {{0, 1}, 1},
      });

  int scale = 2;

  util::Random rand(12345);
  util::set_log_level(util::logging::LogLevel::NONE);
  ClusterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false, scale);

  VI centers = reducer.reduce(3, 1, rand)[0];
  EXPECT_EQ(centers.size(), 3);

  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(ClusterReducerTest, TestResultOrder) {
  unordered_map<int, Point> pos{
      {0, Point(0, 0, 0)}, {1, Point(0.1, 0.1, 0)}, {2, Point(0.2, 0.2, 0)}, {3, Point(100, 100, 0)}, {4, Point(101, 101, 0)}};
  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4}},
      },
      {
          {{0, 1}, 50},
      });
  int scale = 1;
  util::Random rand(12345);

  util::set_log_level(util::logging::LogLevel::NONE);
  ClusterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false, scale);
  vector<int> centers = reducer.reduce(2, 1, rand)[0];

  EXPECT_EQ(centers.size(), 2);
  // can be 0,1,2
  EXPECT_TRUE(centers[0] == 0 || centers[0] == 1 || centers[0] == 2);
  // can be 3,4
  EXPECT_TRUE(centers[1] == 3 || centers[1] == 4);
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(ClusterReducerTest, TestPoiStart) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 0, 0);
  pos[1] = Point(0.1, 0.1, 0);
  pos[2] = Point(0.2, 0.2, 0);
  pos[3] = Point(3, 3, 0);
  pos[4] = Point(3.1, 3.1, 0);
  pos[5] = Point(3.2, 3.2, 0);
  pos[6] = Point(3.3, 3.3, 0);
  pos[7] = Point(6, 6, 0);
  pos[8] = Point(6.1, 6.1, 0);
  pos[9] = Point(6.2, 6.2, 0);

  auto g = ds::graph::Graph(
      {
          {0, {0}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}},
      },
      {
          {{0, 1}, 1},
      });

  int scale = 1;
  util::Random rand(12345);

  util::set_log_level(util::logging::LogLevel::NONE);
  ClusterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false, scale);
  vector<int> centers = reducer.reduce(1, 1, rand)[0];

  EXPECT_EQ(centers.size(), 1);

  EXPECT_TRUE(centers[0] == 7 || centers[0] == 8 || centers[0] == 9);

  scale = 3;

  ClusterReducer reducer2(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false, scale);
  centers = reducer2.reduce(1, 1, rand)[0];

  EXPECT_EQ(centers.size(), 1);

  EXPECT_TRUE(centers[0] == 3 || centers[0] == 4 || centers[0] == 5 || centers[0] == 6);
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(ClusterReducerTest, FilterPois) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 0, 0);
  pos[1] = Point(0, 10, 0);
  pos[2] = Point(0, 10, 0.1);
  pos[3] = Point(0, 10, 0.2);
  pos[4] = Point(10, 0, 0);
  pos[5] = Point(0, -10, 0);
  pos[6] = Point(0, -10, 0.1);
  pos[7] = Point(0, -10, 0.2);
  pos[8] = Point(-20, 0, 0.1);  // farthest but not dense

  auto g = ds::graph::Graph(
      {
          {0, {0}},
          {1, {1, 2, 3, 4, 5, 6, 7, 8}},
      },
      {
          {{0, 1}, 1},
      });
  util::Random rand(12345);

  ClusterReducer r1(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false, 2);
  auto centers = r1.filter_pois(VI({1, 2, 3, 4, 5, 6, 7, 8}), 2, rand);
  EXPECT_EQ(util::sorted(centers), VI({3, 7}));

  ClusterReducer r2(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false, 5);
  centers = r2.filter_pois(VI({1, 2, 3, 4, 5, 6, 7, 8}), 2, rand);
  EXPECT_EQ(centers.size(), 2UL);
}
