#include <gtest/gtest.h>

#include "algorithm/graph/shortest_path.hpp"

using namespace std;
using namespace ds::graph;

typedef std::vector<int> VI;

//
// ShortestPathTest
//
TEST(ShortestPathTest, ShortestPath) {
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
  EXPECT_EQ(algorithm::graph::shortest_path(g, 0, 7), VI({0, 3, 2, 6, 7}));
  EXPECT_EQ(algorithm::graph::shortest_path(g, 7, 0), VI({7, 6, 2, 3, 0}));
}

TEST(ShortestPathTest, ShortestPathDisconnected) {
  Graph g;
  for (int i = 0; i < 4; ++i) g.add_vertex(i, {});
  g.add_edge(0, 3, 2.5);
  g.add_edge(2, 1, 3.1);
  EXPECT_EQ(algorithm::graph::shortest_path(g, 0, 3), VI({0, 3}));
  EXPECT_EQ(algorithm::graph::shortest_path(g, 1, 2), VI({1, 2}));
  EXPECT_THROW(algorithm::graph::shortest_path(g, 0, 1), std::invalid_argument);
  EXPECT_THROW(algorithm::graph::shortest_path(g, 3, 2), std::invalid_argument);
}

TEST(ShortestPathTest, ShortestPathLength) {
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
  EXPECT_EQ(algorithm::graph::shortest_path_length(g, 0, 7), 15);
  EXPECT_EQ(algorithm::graph::shortest_path_length(g, 7, 0), 15);
}

TEST(ShortestPathTest, ShortestPathLengthDisconnected) {
  Graph g;
  for (int i = 0; i < 4; ++i) g.add_vertex(i, {});
  g.add_edge(0, 3, 2.5);
  g.add_edge(2, 1, 3.1);
  EXPECT_EQ(algorithm::graph::shortest_path_length(g, 0, 3), 2.5);
  EXPECT_EQ(algorithm::graph::shortest_path_length(g, 1, 2), 3.1);
  EXPECT_THROW(algorithm::graph::shortest_path_length(g, 0, 1), std::invalid_argument);
  EXPECT_THROW(algorithm::graph::shortest_path_length(g, 3, 2), std::invalid_argument);
}

TEST(ShortestPathTest, AllPairs) {
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

  unordered_map<uint64_t, double> expect = {
      {(0ULL << 32) + 0, 0},   //
      {(0ULL << 32) + 1, 2},   //
      {(0ULL << 32) + 2, 5},   //
      {(0ULL << 32) + 3, 3},   //
      {(0ULL << 32) + 4, 3},   //
      {(0ULL << 32) + 5, 5},   //
      {(0ULL << 32) + 6, 9},   //
      {(0ULL << 32) + 7, 15},  //
      //-------------------------
      {(1ULL << 32) + 1, 0},   //
      {(1ULL << 32) + 2, 6},   //
      {(1ULL << 32) + 3, 5},   //
      {(1ULL << 32) + 4, 1},   //
      {(1ULL << 32) + 5, 3},   //
      {(1ULL << 32) + 6, 8},   //
      {(1ULL << 32) + 7, 14},  //
      //-------------------------
      {(2ULL << 32) + 2, 0},   //
      {(2ULL << 32) + 3, 2},   //
      {(2ULL << 32) + 4, 5},   //
      {(2ULL << 32) + 5, 3},   //
      {(2ULL << 32) + 6, 4},   //
      {(2ULL << 32) + 7, 10},  //
      //-------------------------
      {(3ULL << 32) + 3, 0},   //
      {(3ULL << 32) + 4, 4},   //
      {(3ULL << 32) + 5, 2},   //
      {(3ULL << 32) + 6, 6},   //
      {(3ULL << 32) + 7, 12},  //
      //-------------------------
      {(4ULL << 32) + 4, 0},   //
      {(4ULL << 32) + 5, 2},   //
      {(4ULL << 32) + 6, 7},   //
      {(4ULL << 32) + 7, 13},  //
      //-------------------------
      {(5ULL << 32) + 5, 0},   //
      {(5ULL << 32) + 6, 5},   //
      {(5ULL << 32) + 7, 11},  //
      //-------------------------
      {(6ULL << 32) + 6, 0},  //
      {(6ULL << 32) + 7, 6},  //
      //-------------------------
      {(7ULL << 32) + 7, 0},  //
  };
  EXPECT_EQ(algorithm::graph::all_pairs_dijkstra_path_length(g), expect);
}

TEST(ShortestPathTest, AllPairsDisconnected) {
  Graph g;
  for (int i = 0; i < 4; ++i) g.add_vertex(i, {});
  g.add_edge(0, 3, 2.5);
  g.add_edge(2, 1, 3.1);

  unordered_map<uint64_t, double> expect = {
      {(0ULL << 32) + 0, 0},    //
      {(1ULL << 32) + 1, 0},    //
      {(2ULL << 32) + 2, 0},    //
      {(3ULL << 32) + 3, 0},    //
      {(0ULL << 32) + 3, 2.5},  //
      {(1ULL << 32) + 2, 3.1},  //
  };
  EXPECT_EQ(algorithm::graph::all_pairs_dijkstra_path_length(g), expect);
}

TEST(ShortestPathTest, AllPairsWithTarget) {
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

  unordered_map<uint64_t, double> expect = {
      {(1ULL << 32) + 1, 0},  //
      {(1ULL << 32) + 3, 5},  //
      {(1ULL << 32) + 5, 3},  //
      //-------------------------
      {(3ULL << 32) + 3, 0},  //
      {(3ULL << 32) + 5, 2},  //
      //-------------------------
      {(5ULL << 32) + 5, 0},  //
  };
  EXPECT_EQ(algorithm::graph::all_pairs_dijkstra_path_length(g, {1, 3, 5}), expect);
}
