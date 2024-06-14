#include <gtest/gtest.h>

#include "algorithm/graph/connectivity.hpp"

using namespace std;
using namespace ds::graph;

using VI = std::vector<int>;
using VII = std::vector<std::pair<int, int>>;

//
// ConnectivityTest
//
TEST(ConnectivityTest, ConnectedComponent) {
  Graph g;
  for (int i = 0; i < 8; ++i) g.add_vertex(i, {});
  g.add_edge(0, 1, 2);
  g.add_edge(0, 2, 6);
  g.add_edge(0, 3, 3);
  g.add_edge(1, 2, 6);
  g.add_edge(1, 4, 1);
  g.add_edge(1, 5, 3);
  g.add_edge(2, 3, 2);
  g.add_edge(2, 4, 5);
  g.add_edge(2, 5, 3);
  g.add_edge(2, 6, 4);
  g.add_edge(3, 5, 2);
  g.add_edge(3, 6, 8);
  g.add_edge(4, 5, 2);
  g.add_edge(4, 7, 13);
  g.add_edge(5, 6, 5);
  g.add_edge(5, 7, 11);
  g.add_edge(6, 7, 6);

  auto cc = algorithm::graph::connected_component(g, 0);
  EXPECT_EQ(cc.size(), 8);
  EXPECT_EQ(util::sorted(std::vector<int>(cc.begin(), cc.end())), VI({0, 1, 2, 3, 4, 5, 6, 7}));
}

TEST(ConnectivityTest, ConnectedComponentDisconnected) {
  Graph g;
  for (int i = 0; i < 4; ++i) g.add_vertex(i, {});
  g.add_edge(0, 3, 2.5);
  g.add_edge(2, 1, 3.1);

  auto cc0 = algorithm::graph::connected_component(g, 0);
  auto cc1 = algorithm::graph::connected_component(g, 1);
  auto cc2 = algorithm::graph::connected_component(g, 2);
  auto cc3 = algorithm::graph::connected_component(g, 3);

  EXPECT_EQ(util::sorted(std::vector<int>(cc0.begin(), cc0.end())), VI({0, 3}));
  EXPECT_EQ(util::sorted(std::vector<int>(cc1.begin(), cc1.end())), VI({1, 2}));
  EXPECT_EQ(util::sorted(std::vector<int>(cc2.begin(), cc2.end())), VI({1, 2}));
  EXPECT_EQ(util::sorted(std::vector<int>(cc3.begin(), cc3.end())), VI({0, 3}));
}

TEST(ConnectivityTest, FindBridges) {
  Graph g1;
  for (int i = 0; i < 13; ++i) g1.add_vertex(i, {});
  VII edges1 = {{0, 3}, {0, 5}, {0, 9}, {0, 4}, {3, 5}, {5, 9},  {4, 1},
                {1, 2}, {1, 6}, {1, 7}, {2, 7}, {7, 8}, {10, 11}};
  for (auto &e : edges1) g1.add_edge(e.first, e.second, 1);

  auto bridges = algorithm::graph::find_bridges(g1);
  EXPECT_EQ(util::sorted(bridges), VII({{0, 4}, {1, 4}, {1, 6}, {7, 8}, {10, 11}}));

  Graph g2;
  for (int i = 0; i < 6; ++i) g2.add_vertex(i, {});
  VII edges2 = {{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 5}, {5, 3}};
  for (auto &e : edges2) g2.add_edge(e.first, e.second, 1);
  bridges = algorithm::graph::find_bridges(g2);
  EXPECT_EQ(bridges, VII());

  Graph g3;
  for (int i = 0; i < 6; ++i) g3.add_vertex(i, {});
  VII edges3 = {{5, 1}, {5, 0}, {5, 2}, {5, 3}, {5, 4}};
  for (auto &e : edges3) g3.add_edge(e.first, e.second, 1);
  bridges = algorithm::graph::find_bridges(g3);
  EXPECT_EQ(util::sorted(bridges), VII({{0, 5}, {1, 5}, {2, 5}, {3, 5}, {4, 5}}));
}

TEST(ConnectivityTest, FindPath) {
  Graph g1;
  for (int i = 0; i < 6; ++i) g1.add_vertex(i, {});
  VII edges1 = {{0, 1}, {1, 2}, {2, 3}, {3, 4}};
  for (auto &e : edges1) g1.add_edge(e.first, e.second, 1);

  EXPECT_EQ(algorithm::graph::find_path(g1, 1, 3), VI({1, 2, 3}));
  EXPECT_EQ(algorithm::graph::find_path(g1, 3, 1), VI({3, 2, 1}));
  EXPECT_EQ(algorithm::graph::find_path(g1, 2, 2), VI({2}));
  EXPECT_EQ(algorithm::graph::find_path(g1, 0, 4), VI({0, 1, 2, 3, 4}));

  EXPECT_EQ(algorithm::graph::find_path(g1, 1, 5), VI());  // no such path
  EXPECT_EQ(algorithm::graph::find_path(g1, 5, 1), VI());  // no such path
  EXPECT_EQ(algorithm::graph::find_path(g1, 5, 5), VI({5}));

  EXPECT_THROW(algorithm::graph::find_path(g1, 0, 6), std::invalid_argument);
  EXPECT_THROW(algorithm::graph::find_path(g1, 6, 0), std::invalid_argument);
  EXPECT_THROW(algorithm::graph::find_path(g1, 6, 6), std::invalid_argument);

  Graph g2;
  for (int i = 0; i < 6; ++i) g2.add_vertex(i, {});
  VII edges2 = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 0}, {1, 3}, {2, 4}};
  for (auto &e : edges2) g2.add_edge(e.first, e.second, 1);

  EXPECT_EQ(algorithm::graph::find_path(g2, 4, 0), VI({4, 5, 0}));
  EXPECT_EQ(algorithm::graph::find_path(g2, 0, 3), VI({0, 1, 3}));
}

TEST(ConnectivityTest, FindBridgesInBetween) {
  Graph g1;
  for (int i = 0; i < 13; ++i) g1.add_vertex(i, {});
  VII edges1 = {{0, 1}, {1, 2}, {1, 3},  {3, 4},  {3, 5},  {4, 6},  {5, 6},  {6, 7},
                {7, 8}, {7, 9}, {7, 10}, {9, 10}, {10, 2}, {2, 11}, {11, 12}};
  for (auto &e : edges1) g1.add_edge(e.first, e.second, 1);

  auto bridges = algorithm::graph::find_bridges_in_between(g1, 1, 2);
  EXPECT_EQ(util::sorted(bridges), VII({{1, 3}, {2, 10}, {6, 7}}));

  Graph g2;
  for (int i = 0; i < 13; ++i) g2.add_vertex(i, {});
  VII edges2 = {{0, 1}, {1, 2}, {1, 3},  {3, 4},  {3, 5},  {4, 6},  {5, 6},   {6, 7},
                {7, 8}, {7, 9}, {7, 10}, {9, 10}, {10, 2}, {2, 11}, {11, 12}, {8, 12}};
  for (auto &e : edges2) g2.add_edge(e.first, e.second, 1);

  bridges = algorithm::graph::find_bridges_in_between(g2, 1, 2);
  EXPECT_EQ(util::sorted(bridges), VII({{1, 3}, {6, 7}}));

  Graph g3;
  for (int i = 0; i < 3; ++i) g3.add_vertex(i, {});
  g3.add_edge(0, 1, 1);
  g3.add_edge(0, 2, 1);
  bridges = algorithm::graph::find_bridges_in_between(g3, 0, 1);
  EXPECT_EQ(util::sorted(bridges), VII());

  g3.add_edge(2, 1, 1);
  bridges = algorithm::graph::find_bridges_in_between(g3, 0, 1);
  EXPECT_EQ(util::sorted(bridges), VII({{0, 2}, {1, 2}}));

  Graph g4;
  for (int i = 0; i < 18; ++i) g4.add_vertex(i, {});
  VII edges4 = {{1, 13}, {0, 1},  {1, 3},  {1, 4},  {1, 2},  {3, 5},   {3, 6},   {3, 7},   {7, 2},   {6, 8},  {6, 9},
                {6, 10}, {6, 11}, {9, 12}, {9, 13}, {9, 14}, {14, 10}, {13, 15}, {13, 16}, {15, 16}, {16, 17}};
  for (auto &e : edges4) g4.add_edge(e.first, e.second, 1);
  bridges = algorithm::graph::find_bridges_in_between(g4, 1, 13);
  EXPECT_EQ(util::sorted(bridges), VII({{3, 6}, {9, 13}}));

  bridges = algorithm::graph::find_bridges_in_between(g4, 13, 1);
  EXPECT_EQ(util::sorted(bridges), VII({{3, 6}, {9, 13}}));
}
