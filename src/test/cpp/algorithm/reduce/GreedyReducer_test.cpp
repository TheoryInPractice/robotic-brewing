#include <gtest/gtest.h>

#include "algorithm/reduce/GreedyReducer.hpp"

using namespace std;
using namespace algorithm::reduce;
using namespace geometry;

using VI = vector<int>;
using VVI = vector<VI>;

TEST(GreedyReducerTest, Reduce) {
  std::unordered_map<int, Point> pos;
  pos[0] = Point(0, 6, 0);
  pos[1] = Point(4.1, 4.1, 0);
  pos[2] = Point(0, 0.9, 0);
  pos[3] = Point(3, 2.99, 0);
  pos[4] = Point(1, 0, 0);
  pos[5] = Point(0, 0, 0);
  pos[6] = Point(-4.8, 0, 0);
  pos[7] = Point(0, -4.7, 0);
  pos[8] = Point(3, -3, 0);
  pos[9] = Point(5, 0, 0);
  pos[10] = Point(100, 100, 100);

  std::map<int, std::vector<int>> expected_seq = {
      {0, {7, 9, 6, 2, 1, 8, 3, 4, 5}}, {1, {6, 8, 2, 0, 9, 7, 3, 4, 5}}, {2, {7, 1, 6, 0, 9, 8, 3, 4, 5}},
      {3, {6, 7, 0, 5, 9, 8, 1, 4, 2}}, {4, {0, 6, 7, 1, 9, 8, 3, 2, 5}}, {5, {0, 9, 6, 7, 1, 8, 3, 4, 2}},
      {6, {9, 0, 7, 2, 1, 8, 3, 4, 5}}, {7, {0, 9, 6, 2, 1, 8, 3, 4, 5}}, {8, {0, 6, 2, 1, 9, 7, 3, 4, 5}},
      {9, {6, 0, 7, 2, 1, 8, 3, 4, 5}},
  };

  auto g1 = ds::graph::Graph(
      {
          {0, {2, 4, 5}}, {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}},  // 10 colors
      },
      {});

  util::set_log_level(util::logging::LogLevel::NONE);
  GreedyReducer r1(pos, g1, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);

  util::Random rand(12345);
  EXPECT_EQ(util::sorted(r1.reduce(0, 1, rand)[0]), VI({}));
  EXPECT_EQ(util::sorted(r1.reduce(1, 1, rand)[0]), VI({1}));  // POI 1 is the farthest
  EXPECT_EQ(util::sorted(r1.reduce(2, 1, rand)[0]), VI({1, 6}));
  EXPECT_EQ(util::sorted(r1.reduce(3, 1, rand)[0]), VI({1, 6, 7}));
  EXPECT_EQ(util::sorted(r1.reduce(4, 1, rand)[0]), VI({0, 1, 6, 7}));
  EXPECT_EQ(util::sorted(r1.reduce(5, 1, rand)[0]), VI({0, 1, 6, 7, 9}));
  EXPECT_EQ(util::sorted(r1.reduce(6, 1, rand)[0]), VI({0, 1, 6, 7, 8, 9}));
  EXPECT_EQ(util::sorted(r1.reduce(7, 1, rand)[0]), VI({0, 1, 3, 6, 7, 8, 9}));
  EXPECT_EQ(util::sorted(r1.reduce(8, 1, rand)[0]), VI({0, 1, 3, 6, 7, 8, 9}));
  EXPECT_EQ(util::sorted(r1.reduce(9, 1, rand)[0]), VI({0, 1, 3, 6, 7, 8, 9}));
  EXPECT_EQ(util::sorted(r1.reduce(10, 1, rand)[0]), VI({0, 1, 3, 6, 7, 8, 9}));
  EXPECT_EQ(util::sorted(r1.reduce(11, 1, rand)[0]), VI({0, 1, 3, 6, 7, 8, 9}));

  auto g2 = ds::graph::Graph(
      {
          {0, {5}}, {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}},  // 10 colors
      },
      {});
  GreedyReducer r2(pos, g2, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, false);

  rand.seed(12345);
  for (int i = 0; i <= 11; ++i) {
    std::vector<int> expected(expected_seq[5].begin(), expected_seq[5].begin() + std::min(9, i));
    EXPECT_EQ(r2.reduce(i, 1, rand)[0], expected);
  }

  auto g3 = ds::graph::Graph(
      {
          {0, {}}, {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}},  // 10 colors
      },
      {});
  GreedyReducer r3(pos, g3, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, true);  // random start

  for (int t = 0; t < 10; ++t) {
    for (int j = 0; j <= 11; ++j) {
      auto ret = r3.reduce(j, 1, rand)[0];
      EXPECT_EQ(ret.size(), std::min(j, 10));

      if (j >= 2) {
        auto first_poi = ret[0];
        std::vector<int> expected = {first_poi};

        for (int k = 0; k < std::min(j - 1, 9); ++k) expected.push_back(expected_seq[first_poi][k]);
        EXPECT_EQ(ret, expected);
      }
    }
  }

  GreedyReducer r4(pos, g2, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction, true);  // random start

  std::map<int, std::vector<int>> expected_seq2 = {
      {0, {7, 9, 6, 2, 1, 8, 3, 4}}, {1, {6, 8, 2, 0, 9, 7, 3, 4}}, {2, {7, 1, 6, 0, 9, 8, 3, 4}},
      {3, {6, 7, 0, 2, 9, 8, 1, 4}}, {4, {0, 6, 7, 1, 9, 8, 3, 2}}, {6, {9, 0, 7, 2, 1, 8, 3, 4}},
      {7, {0, 9, 6, 2, 1, 8, 3, 4}}, {8, {0, 6, 2, 1, 9, 7, 3, 4}}, {9, {6, 0, 7, 2, 1, 8, 3, 4}},
  };

  for (int t = 0; t < 10; ++t) {
    for (int j = 0; j <= 11; ++j) {
      auto ret = r4.reduce(j, 1, rand)[0];
      EXPECT_EQ(ret.size(), std::min(j, 9));

      if (j >= 2) {
        auto first_poi = ret[0];
        EXPECT_NE(first_poi, 5);
        std::vector<int> expected = {first_poi};

        for (int k = 0; k < std::min(j - 1, 8); ++k) { expected.push_back(expected_seq2[first_poi][k]); }
        EXPECT_EQ(ret, expected);
      }
    }
  }

  auto g5 = ds::graph::Graph(
      {
          {0, {5}}, {1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}},  // 10 colors
      },
      {});
  GreedyReducer r5(pos, g5, 0, PartitionAlgorithm::Geometric, PartitionTiming::BeforeReduction, false);  // pre-clustering
  rand.seed(12345);
  EXPECT_EQ(r5.reduce(2, 2, rand), VVI({{0, 6}, {9, 7}}));
  EXPECT_EQ(r5.reduce(3, 3, rand), VVI({{0, 7, 1}, {9, 8, 3}, {6, 4, 2}}));

  GreedyReducer r6(pos, g5, 0, PartitionAlgorithm::Geometric, PartitionTiming::AfterReduction, false);  // post-clustering
  rand.seed(12345);
  EXPECT_EQ(r6.reduce(2, 2, rand), VVI({{0, 6}, {9, 7}}));
  EXPECT_EQ(r6.reduce(3, 3, rand), VVI({{0, 1, 7}, {9, 3, 8}, {6, 2, 4}}));

  util::set_log_level(util::logging::LogLevel::TRACE);
}
