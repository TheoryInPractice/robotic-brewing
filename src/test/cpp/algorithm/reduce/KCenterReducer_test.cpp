#include <gtest/gtest.h>
#include <set>
#include <unordered_map>
#include <vector>

#include "algorithm/reduce/KCenterReducer.hpp"
#include "geometry/Point.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

using namespace std;
using namespace algorithm::reduce;
using namespace geometry;

typedef vector<int> VI;

TEST(KCenterReducerTest, ReduceCorrectSize) {
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

  util::Random rand(12345);
  KCenterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);
  util::set_log_level(util::logging::LogLevel::NONE);
  VI centers = reducer.reduce(3, 1, rand)[0];
  util::set_log_level(util::logging::LogLevel::TRACE);
  EXPECT_EQ(centers.size(), 3);
}

TEST(KCenterReducerTest, TestResultOrder2) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 0, 0);
  pos[1] = Point(1, 0, 0);
  pos[2] = Point(0, 1, 0);
  pos[3] = Point(0, 0, 1);
  pos[4] = Point(1, 1, 0);
  pos[5] = Point(1, 0, 1);
  pos[6] = Point(0, 1, 1);
  pos[7] = Point(1, 1, 1);
  pos[8] = Point(0.5, 0.5, 0.5);

  pos[9] = Point(100, 100, 100);
  pos[10] = Point(101, 100, 100);
  pos[11] = Point(100, 101, 100);
  pos[12] = Point(100, 100, 101);
  pos[13] = Point(101, 101, 100);
  pos[14] = Point(101, 100, 101);
  pos[15] = Point(100, 101, 101);
  pos[16] = Point(101, 101, 101);
  pos[17] = Point(100.5, 100.5, 100.5);

  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}},
      },
      {
          {{0, 1}, 5},
      });

  util::Random rand(12345);

  KCenterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);
  util::set_log_level(util::logging::LogLevel::NONE);
  vector<int> centers = reducer.reduce(2, 1, rand)[0];
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(centers.size(), 2);

  EXPECT_TRUE(centers[0] == 8 || centers[0] == 17);
}

TEST(KCenterReducerTest, TestCenters) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 0, 0);
  pos[1] = Point(1, 0, 0);
  pos[2] = Point(0, 1, 0);
  pos[3] = Point(0, 0, 1);
  pos[4] = Point(1, 1, 0);
  pos[5] = Point(1, 0, 1);
  pos[6] = Point(0, 1, 1);
  pos[7] = Point(1, 1, 1);
  pos[8] = Point(0.5, 0.5, 0.5);

  pos[9] = Point(100, 100, 100);
  pos[10] = Point(101, 100, 100);
  pos[11] = Point(100, 101, 100);
  pos[12] = Point(100, 100, 101);
  pos[13] = Point(101, 101, 100);
  pos[14] = Point(101, 100, 101);
  pos[15] = Point(100, 101, 101);
  pos[16] = Point(101, 101, 101);
  pos[17] = Point(100.5, 100.5, 100.5);

  pos[18] = Point(200, 200, 200);
  pos[19] = Point(201, 200, 200);
  pos[20] = Point(200, 201, 200);
  pos[21] = Point(200, 200, 201);
  pos[22] = Point(201, 201, 200);
  pos[23] = Point(201, 200, 201);
  pos[24] = Point(200, 201, 201);
  pos[25] = Point(201, 201, 201);
  pos[26] = Point(200.5, 200.5, 200.5);

  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26}},
      },
      {
          {{0, 1}, 5},
      });

  util::Random rand(12345);

  KCenterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);
  util::set_log_level(util::logging::LogLevel::NONE);
  vector<int> centers = reducer.reduce(3, 1, rand)[0];
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(centers.size(), 3);

  EXPECT_TRUE(centers[0] == 8 || centers[0] == 17 || centers[0] == 26);

  EXPECT_TRUE(centers[1] == 8 || centers[1] == 17 || centers[1] == 26);

  EXPECT_TRUE(centers[2] == 8 || centers[2] == 17 || centers[2] == 26);
}

TEST(KCenterReducerTest, TestCenterOrders) {
  unordered_map<int, Point> pos;
  pos[0] = Point(1, 0, 0);
  pos[1] = Point(0, 1, 0);
  pos[2] = Point(0, 0, 1);
  pos[3] = Point(1, 1, 0);
  pos[4] = Point(1, 0, 1);
  pos[5] = Point(0, 1, 1);
  pos[6] = Point(0.5, 0.5, 0.5);

  pos[7] = Point(100, 100, 101);
  pos[8] = Point(101, 101, 100);
  pos[9] = Point(101, 100, 101);
  pos[10] = Point(100, 101, 101);
  pos[11] = Point(100.5, 100.5, 100.5);

  pos[12] = Point(200, 200, 200);
  pos[13] = Point(201, 200, 200);
  pos[14] = Point(200, 201, 200);
  pos[15] = Point(200, 200, 201);
  pos[16] = Point(201, 201, 200);
  pos[17] = Point(201, 200, 201);
  pos[18] = Point(200, 201, 201);
  pos[19] = Point(201, 201, 201);
  pos[20] = Point(200.5, 200.5, 200.5);

  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20}},
      },
      {
          {{0, 1}, 5},
      });

  util::Random rand(12345);

  KCenterReducer reducer(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);
  util::set_log_level(util::logging::LogLevel::NONE);
  vector<int> centers = reducer.reduce(10, 1, rand)[0];
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(centers.size(), 10);
}

TEST(KCenterReducerTest, TestCenterOrderSplit) {
  // testing order by split first, density second
  unordered_map<int, Point> pos;

  pos[0] = Point(1, 0, 0);
  pos[1] = Point(0, 1, 0);
  pos[2] = Point(0.5, 0.5, 0);

  pos[3] = Point(50, 51, 0);
  pos[4] = Point(51, 50, 0);
  pos[5] = Point(50.5, 50.5, 0);

  pos[6] = Point(5, 4, 0);
  pos[7] = Point(4, 5, 0);
  pos[8] = Point(4.5, 4.5, 0);

  pos[9] = Point(125, 126, 0);
  pos[10] = Point(126, 125, 0);
  pos[11] = Point(125.5, 125.5, 0);

  pos[12] = Point(75, 74, 0);
  pos[13] = Point(74, 75, 0);
  pos[14] = Point(74.5, 74.5, 0);

  pos[15] = Point(150, 151, 0);
  pos[16] = Point(151, 150, 0);
  pos[17] = Point(150.5, 150.5, 0);

  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}},
      },
      {
          {{0, 1}, 5},
      });

  util::Random rand(12345);
  int splits = 3;

  KCenterReducer reducer(pos, g, 0, PartitionAlgorithm::Geometric, PartitionTiming::AfterReduction, false);  // post-cluster
  util::set_log_level(util::logging::LogLevel::NONE);
  auto centers = reducer.reduce(2, splits, rand);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(centers[0].size(), 2);
  EXPECT_EQ(centers[1].size(), 2);
  EXPECT_EQ(centers[2].size(), 2);

  // 17 and 11
  // 5 and 14
  //  2 and 8
}

TEST(KCenterReducerTest, TestCenterOrderSplit2) {
  // testing order by split first, density second
  unordered_map<int, Point> pos;

  pos[0] = Point(1, 0, 0);
  pos[1] = Point(0, 1, 0);
  pos[2] = Point(0.5, 0.5, 0);

  pos[3] = Point(50, 51, 0);
  pos[4] = Point(51, 50, 0);
  pos[5] = Point(50.5, 50.5, 0);

  pos[6] = Point(5, 4, 0);
  pos[7] = Point(4, 5, 0);
  pos[8] = Point(4.5, 4.5, 0);

  pos[9] = Point(125, 126, 0);
  pos[10] = Point(126, 125, 0);
  pos[11] = Point(125.5, 125.5, 0);

  pos[12] = Point(75, 74, 0);
  pos[13] = Point(74, 75, 0);
  pos[14] = Point(74.5, 74.5, 0);

  pos[15] = Point(150, 151, 0);
  pos[16] = Point(151, 150, 0);
  pos[17] = Point(150.5, 150.5, 0);

  auto g = ds::graph::Graph(
      {
          {0, {}},
          {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}},
      },
      {
          {{0, 1}, 5},
      });

  util::Random rand(12345);
  int splits = 2;

  KCenterReducer reducer(pos, g, 0, PartitionAlgorithm::Geometric, PartitionTiming::AfterReduction, false);  // post-cluster
  util::set_log_level(util::logging::LogLevel::NONE);
  auto centers = reducer.reduce(3, splits, rand);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(centers[0].size(), 3);
  EXPECT_EQ(centers[1].size(), 3);

  // 17 and 11 and 14
  //  2 and 5 and 8
}

TEST(KCenterReducerTest, FilterPois) {
  unordered_map<int, Point> pos;
  pos[0] = Point(0, 0, 0);
  pos[1] = Point(0, 10, 0);
  pos[2] = Point(0, 10, 0.1);
  pos[3] = Point(0, 10, 0.3);
  pos[4] = Point(9.9, 0, 0);
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

  KCenterReducer r1(pos, g, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);
  auto centers = r1.filter_pois(VI({1, 2, 3, 4, 5, 6, 7, 8}), 2, rand);

  // First, {8, 3} will be picked by k-Center.
  // Then, recalculated to {8, 4} by k_Means.
  EXPECT_EQ(util::sorted(centers), VI({4, 8}));
}
