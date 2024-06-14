#if GUROBI_ON
#include <gtest/gtest.h>

#include "algorithm/merge/ExactMerger.hpp"
#include "readwrite/pace.hpp"
#include "util/logger.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithm::merge;

using VI = vector<int>;
using VVI = vector<VI>;

static std::pair<double, std::set<int>> verify_eulerian_tour(ds::graph::Graph const& graph, std::vector<int> const& tour) {
  EXPECT_EQ(tour.front(), tour.back());

  double weight = 0.0;
  std::set<int> visited;

  for (std::size_t i = 1; i < tour.size(); ++i) {
    int u = tour[i - 1], v = tour[i];

    weight += graph.get_weight(u, v);
    visited.insert(u);
  }
  return {weight, visited};
}

static VI run_merge(ExactMerger& merger, Graph const& graph, VVI const& walks) {
  util::set_log_level(util::logging::LogLevel::NONE);
  auto merged = merger.merge(graph, walks);
  util::set_log_level(util::logging::LogLevel::TRACE);
  return merged;
}

TEST(ExactMergerTest, Merge) {
  uint32_t seed = 12345;

  auto g = readwrite::load_pace("src/test/resources/instances/001_tiny.gr").graph;
  ExactMerger merger(2, seed);

  VVI walks = {{0, 1, 0}, {0, 3, 0}, {0, 3, 4, 3, 0}};
  auto merged = run_merge(merger, g, walks);
  EXPECT_EQ(merged, VI({0, 1, 0, 3, 4, 3, 0}));

  //----------------------------------------------------------------------------

  Graph g1;
  for (int i = 0; i < 6; ++i) g1.add_vertex(i, {});
  g1.add_edge(0, 1, 1);
  g1.add_edge(1, 2, 3);
  g1.add_edge(2, 3, 1);
  g1.add_edge(3, 4, 1);
  g1.add_edge(4, 0, 1);
  g1.add_edge(1, 4, 1);
  g1.add_edge(2, 4, 1);

  VI walk = {0, 1, 2, 1, 4, 0, 0, 4, 3, 2, 4, 0};
  auto simplified = run_merge(merger, g1, {walk});
  auto verified = verify_eulerian_tour(g1, simplified);
  EXPECT_EQ(verified.first, 6);
  EXPECT_EQ(verified.second, std::set<int>({0, 1, 2, 3, 4}));

  walk = {0, 1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1, 0};
  simplified = run_merge(merger, g1, {walk});
  verified = verify_eulerian_tour(g1, simplified);
  EXPECT_EQ(verified.first, 12);
  EXPECT_EQ(verified.second, std::set<int>({0, 1, 2, 3, 4}));

  Graph g2;
  for (int i = 0; i < 6; ++i) g2.add_vertex(i, {});
  g2.add_edge(0, 1, 1);
  g2.add_edge(1, 2, 1);
  g2.add_edge(2, 3, 1);
  g2.add_edge(3, 4, 1);
  g2.add_edge(4, 0, 1);
  g2.add_edge(1, 4, 3);
  g2.add_edge(2, 4, 3);

  walk = {0, 1, 2, 1, 4, 0, 0, 4, 3, 2, 4, 0};
  simplified = run_merge(merger, g2, {walk});
  verified = verify_eulerian_tour(g2, simplified);
  EXPECT_EQ(verified.first, 5);
  EXPECT_EQ(verified.second, std::set<int>({0, 1, 2, 3, 4}));

  Graph g3;
  for (int i = 0; i < 6; ++i) g3.add_vertex(i, {});
  g3.add_edge(0, 1, 1);
  g3.add_edge(1, 2, 1);
  g3.add_edge(2, 3, 1);
  g3.add_edge(3, 4, 1);
  g3.add_edge(4, 5, 1);
  g3.add_edge(5, 0, 1);
  g3.add_edge(1, 4, 3);
  g3.add_edge(1, 5, 3);
  g3.add_edge(2, 4, 3);
  g3.add_edge(2, 5, 3);

  walk = {0, 1, 5, 0, 5, 2, 4, 1, 2, 3, 4, 3, 2, 1, 0};
  simplified = run_merge(merger, g3, {walk});
  verified = verify_eulerian_tour(g3, simplified);
  EXPECT_EQ(verified.first, 10);
  EXPECT_EQ(verified.second, std::set<int>({0, 1, 2, 3, 4, 5}));

  Graph g4;
  for (int i = 0; i < 6; ++i) g4.add_vertex(i, {});
  g4.add_edge(0, 1, 7);
  g4.add_edge(1, 2, 7);
  g4.add_edge(2, 3, 7);
  g4.add_edge(3, 0, 7);
  g4.add_edge(1, 4, 5);
  g4.add_edge(1, 5, 5);
  g4.add_edge(3, 4, 3);
  g4.add_edge(3, 5, 3);
  g4.add_edge(4, 5, 4);

  walk = {0, 1, 4, 5, 1, 2, 3, 5, 4, 3, 0};
  simplified = run_merge(merger, g4, {walk});
  verified = verify_eulerian_tour(g4, simplified);
  EXPECT_EQ(verified.first, 38);
  EXPECT_EQ(verified.second, std::set<int>({0, 1, 2, 3, 4, 5}));

  // Heuristic cannot give an optimal solution.
  Graph g5;
  for (int i = 0; i < 4; ++i) g5.add_vertex(i, {});
  g5.add_edge(0, 1, 1);
  g5.add_edge(1, 2, 1);
  g5.add_edge(2, 3, 1);
  g5.add_edge(3, 0, 100);

  walk = {0, 1, 0, 3, 2, 3, 2, 1, 2, 1, 0};
  simplified = run_merge(merger, g5, {walk});
  verified = verify_eulerian_tour(g5, simplified);
  EXPECT_EQ(verified.first, 6);
  EXPECT_EQ(verified.second, std::set<int>({0, 1, 2, 3}));
}
#endif
