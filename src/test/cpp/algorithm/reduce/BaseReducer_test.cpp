#include <gtest/gtest.h>

#include "algorithm/reduce/BaseReducer.hpp"

using namespace std;
using namespace algorithm::reduce;
using namespace geometry;

using VI = vector<int>;
using VVI = vector<VI>;

class DummyReducer : public BaseReducer {
 public:
  DummyReducer(std::unordered_map<int, ::geometry::Point> const& poi_position, ds::graph::Graph const& graph,
               ds::graph::Graph::Vertex source_vertex, PartitionAlgorithm partition_algorithm, PartitionTiming partition_timing)
      : BaseReducer("DummyReducer", poi_position, graph, source_vertex, partition_algorithm, partition_timing) {}

  std::vector<POI> filter_pois(std::vector<POI> const& pois, int k, util::Random& rand) const override {
    return std::vector<POI>(pois.begin(), pois.begin() + k);
  }
};

TEST(BaseReducerTest, PartitionPois) {
  std::unordered_map<int, Point> pos;
  std::vector<int> ps = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
  pos[25] = Point(100, 100, 100);
  for (auto p : ps) pos[p] = Point(p, p * 2, p * 3);

  auto g1 = ds::graph::Graph(
      {
          {0, {25}},
          {1, ps},
      },
      {});
  util::Random rand(12345);

  DummyReducer r1(pos, g1, 0, PartitionAlgorithm::Ordered, PartitionTiming::AfterReduction);
  EXPECT_EQ(r1.partition_pois(ps, 0, 0, rand), VVI({{1, 3, 5, 7, 9, 11, 13, 15, 17, 19}}));
  EXPECT_EQ(r1.partition_pois(ps, 1, 0, rand), VVI({{1, 3, 5, 7, 9, 11, 13, 15, 17, 19}}));
  EXPECT_EQ(r1.partition_pois(ps, 2, 0, rand), VVI({{1, 3, 5, 7, 9}, {11, 13, 15, 17, 19}}));
  EXPECT_EQ(r1.partition_pois(ps, 3, 0, rand), VVI({{1, 3, 5, 7}, {9, 11, 13}, {15, 17, 19}}));
  EXPECT_EQ(r1.partition_pois(ps, 4, 2, rand), VVI({{1, 3, 5}, {7, 9, 11}, {13, 15}, {17, 19}}));
  EXPECT_EQ(r1.partition_pois(ps, 5, 2, rand), VVI({{1, 3}, {5, 7}, {9, 11}, {13, 15}, {17, 19}}));
  EXPECT_EQ(r1.partition_pois(ps, 6, 0, rand), VVI({{1, 3}, {5, 7}, {9, 11}, {13, 15}, {17}, {19}}));
  EXPECT_EQ(r1.partition_pois(ps, 9, 0, rand), VVI({{1, 3}, {5}, {7}, {9}, {11}, {13}, {15}, {17}, {19}}));
  EXPECT_EQ(r1.partition_pois(ps, 10, 0, rand), VVI({{1}, {3}, {5}, {7}, {9}, {11}, {13}, {15}, {17}, {19}}));
  EXPECT_EQ(r1.partition_pois(ps, 15, 0, rand), VVI({{1}, {3}, {5}, {7}, {9}, {11}, {13}, {15}, {17}, {19}, {}, {}, {}, {}, {}}));
  EXPECT_THROW(r1.partition_pois(ps, 4, 3, rand), std::invalid_argument);
  EXPECT_THROW(r1.partition_pois(ps, 15, 1, rand), std::invalid_argument);

  DummyReducer r2(pos, g1, 0, PartitionAlgorithm::Geometric, PartitionTiming::BeforeReduction);
  EXPECT_EQ(r2.partition_pois(ps, 0, 0, rand), VVI({{1, 3, 5, 7, 9, 11, 13, 15, 17, 19}}));
  EXPECT_EQ(r2.partition_pois(ps, 1, 0, rand), VVI({{1, 3, 5, 7, 9, 11, 13, 15, 17, 19}}));
  EXPECT_EQ(r2.partition_pois(ps, 2, 0, rand), VVI({{1, 3, 5, 7, 9}, {19, 17, 15, 13, 11}}));

  for (std::size_t k = 3; k <= 4; ++k) {
    for (std::size_t t = 0; t <= 10 / k; ++t) {
      auto result = r2.partition_pois(ps, k, t, rand);
      int sum = 0;
      EXPECT_EQ(result.size(), k);
      for (auto& r : result) {
        EXPECT_GE(r.size(), std::max(1UL, t));
        for (auto x : r) sum += x;
      }
      EXPECT_EQ(sum, 1 + 3 + 5 + 7 + 9 + 11 + 13 + 15 + 17 + 19);
    }
  }
  EXPECT_THROW(r2.partition_pois(ps, 3, 4, rand), std::invalid_argument);

  EXPECT_THROW(r2.partition_pois(ps, 4, 3, rand), std::invalid_argument);
}
