#include <gtest/gtest.h>

#include "algorithm/clustering/clustering.hpp"

using namespace std;
using namespace algorithm::clustering;
using namespace geometry;

using VI = std::vector<int>;
using VVI = std::vector<VI>;

TEST(ClusteringTest, KCenterGreedy) {
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
  VI ps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  util::Random rand(12345);
  EXPECT_EQ(k_center_greedy(ps, pos, 0, rand, VI({2, 4, 5})), VI());
  EXPECT_EQ(k_center_greedy(ps, pos, 1, rand, VI({2, 4, 5})), VI({1}));  // point 1 is the farthest
  EXPECT_EQ(k_center_greedy(ps, pos, 2, rand, VI({2, 4, 5})), VI({1, 6}));
  EXPECT_EQ(k_center_greedy(ps, pos, 3, rand, VI({2, 4, 5})), VI({1, 6, 7}));
  EXPECT_EQ(k_center_greedy(ps, pos, 4, rand, VI({2, 4, 5})), VI({1, 6, 7, 0}));
  EXPECT_EQ(k_center_greedy(ps, pos, 5, rand, VI({2, 4, 5})), VI({1, 6, 7, 0, 9}));
  EXPECT_EQ(k_center_greedy(ps, pos, 6, rand, VI({2, 4, 5})), VI({1, 6, 7, 0, 9, 8}));
  EXPECT_EQ(k_center_greedy(ps, pos, 7, rand, VI({2, 4, 5})), VI({1, 6, 7, 0, 9, 8, 3}));
  EXPECT_EQ(k_center_greedy(ps, pos, 8, rand, VI({2, 4, 5})), VI({1, 6, 7, 0, 9, 8, 3}));
  EXPECT_EQ(k_center_greedy(ps, pos, 9, rand, VI({2, 4, 5})), VI({1, 6, 7, 0, 9, 8, 3}));
  EXPECT_EQ(k_center_greedy(ps, pos, 10, rand, VI({2, 4, 5})), VI({1, 6, 7, 0, 9, 8, 3}));

  EXPECT_EQ(k_center_greedy(ps, pos, 0, rand, VI({5})), VI());
  EXPECT_EQ(k_center_greedy(ps, pos, 1, rand, VI({5})), VI({0}));  // point 0 is the farthest
  EXPECT_EQ(k_center_greedy(ps, pos, 2, rand, VI({5})), VI({0, 9}));
  EXPECT_EQ(k_center_greedy(ps, pos, 3, rand, VI({5})), VI({0, 9, 6}));
  EXPECT_EQ(k_center_greedy(ps, pos, 4, rand, VI({5})), VI({0, 9, 6, 7}));
  EXPECT_EQ(k_center_greedy(ps, pos, 5, rand, VI({5})), VI({0, 9, 6, 7, 1}));
  EXPECT_EQ(k_center_greedy(ps, pos, 6, rand, VI({5})), VI({0, 9, 6, 7, 1, 8}));
  EXPECT_EQ(k_center_greedy(ps, pos, 7, rand, VI({5})), VI({0, 9, 6, 7, 1, 8, 3}));
  EXPECT_EQ(k_center_greedy(ps, pos, 8, rand, VI({5})), VI({0, 9, 6, 7, 1, 8, 3, 4}));
  EXPECT_EQ(k_center_greedy(ps, pos, 9, rand, VI({5})), VI({0, 9, 6, 7, 1, 8, 3, 4, 2}));
  EXPECT_EQ(k_center_greedy(ps, pos, 10, rand, VI({5})), VI({0, 9, 6, 7, 1, 8, 3, 4, 2}));

  EXPECT_EQ(k_center_greedy(ps, pos, 0, rand, VI({0, 1, 2, 3, 4, 5, 6, 7, 8, 9})), VI());
  EXPECT_EQ(k_center_greedy(ps, pos, 2, rand, VI({0, 1, 2, 3, 4, 5, 6, 7, 8, 9})), VI());

  std::map<int, std::vector<int>> expected_seq = {
      {0, {7, 9, 6, 2, 1, 8, 3, 4, 5}}, {1, {6, 8, 2, 0, 9, 7, 3, 4, 5}}, {2, {7, 1, 6, 0, 9, 8, 3, 4, 5}},
      {3, {6, 7, 0, 5, 9, 8, 1, 4, 2}}, {4, {0, 6, 7, 1, 9, 8, 3, 2, 5}}, {5, {0, 9, 6, 7, 1, 8, 3, 4, 2}},
      {6, {9, 0, 7, 2, 1, 8, 3, 4, 5}}, {7, {0, 9, 6, 2, 1, 8, 3, 4, 5}}, {8, {0, 6, 2, 1, 9, 7, 3, 4, 5}},
      {9, {6, 0, 7, 2, 1, 8, 3, 4, 5}},
  };

  rand.seed(12345);
  std::set<int> chosen;
  for (int i = 0; i < 60; ++i) {
    for (int j = 0; j <= 11; ++j) {
      auto ret = k_center_greedy(ps, pos, j, rand);
      if (j == 0) {
        EXPECT_EQ(ret, VI());
      } else if (j == 1) {
        EXPECT_EQ(ret.size(), 1UL);  // can be anything
        chosen.insert(ret[0]);
      } else {
        auto a = ret[0];
        std::vector<int> expected = {a};

        for (int k = 0; k < std::min(j - 1, 9); ++k) { expected.push_back(expected_seq[a][k]); }
        EXPECT_EQ(ret, expected);
      }
    }
  }
  EXPECT_EQ(chosen.size(), 10);  // should cover all points
}

TEST(ClusteringTest, PartitionIntoPoints) {
  std::unordered_map<int, Point> pos;
  pos[0] = Point(-10, 0, 0);
  pos[1] = Point(0, 10, 0);
  pos[2] = Point(-9, 0, 0);
  pos[3] = Point(-1, -4, 0);
  pos[4] = Point(6, -3, 0);
  pos[5] = Point(10, 0, 0);
  pos[6] = Point(11, 0, 0);
  pos[7] = Point(11, 0.5, 0);
  pos[8] = Point(10, 1, 0);
  pos[9] = Point(0, -8, 0);
  pos[10] = Point(9999, 9999, 9999);

  VI points = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  util::Random rand(12345);

  EXPECT_EQ(partition_into_points(points, VI({4, 6, 2, 10}), pos, 0, rand), VVI({{4, 3, 9}, {6, 7, 5, 8}, {2, 0, 1}, {10}}));
  EXPECT_EQ(partition_into_points(points, VI({4, 6, 2, 10}), pos, 1, rand), VVI({{4, 3, 9}, {6, 7, 5, 8}, {2, 0, 1}, {10}}));
  EXPECT_EQ(partition_into_points(points, VI({4, 6, 2, 10}), pos, 2, rand), VVI({{4, 5, 3, 9}, {6, 7}, {2, 0, 1}, {10, 8}}));

  EXPECT_EQ(partition_into_points(points, VI({4, 6, 2}), pos, 3, rand), VVI({{4, 8, 3}, {6, 7, 5, 10}, {2, 0, 9, 1}}));

  // error case
  EXPECT_THROW(partition_into_points(points, VI({2, 5, 9}), pos, 4, rand), std::invalid_argument);
}
