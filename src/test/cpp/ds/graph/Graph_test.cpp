#include <gtest/gtest.h>

#include "ds/graph/Graph.hpp"

using namespace std;
using namespace ds::graph;

typedef std::vector<int> VI;

TEST(GraphTest, BasicOperations) {
  auto G = Graph(
      {
          {1, {3, 7}},
          {2, {}},
          {4, {9, 3, 1}},
      },
      {
          {{1, 2}, 0.5},
          {{4, 2}, 1.3},
      });

  // properties
  EXPECT_EQ(G.number_of_vertices(), 3);
  EXPECT_EQ(G.number_of_edges(), 2);
  EXPECT_EQ(G.number_of_colors(), 4);

  EXPECT_FALSE(G.has_vertex(0));
  EXPECT_TRUE(G.has_vertex(1));
  EXPECT_TRUE(G.has_vertex(2));
  EXPECT_FALSE(G.has_vertex(3));
  EXPECT_TRUE(G.has_vertex(4));

  EXPECT_FALSE(G.has_edge(0, 1));
  EXPECT_FALSE(G.has_edge(1, 1));
  EXPECT_TRUE(G.has_edge(1, 2));
  EXPECT_TRUE(G.has_edge(2, 1));
  EXPECT_TRUE(G.has_edge(2, 4));
  EXPECT_FALSE(G.has_edge(3, 3));
  EXPECT_FALSE(G.has_edge(3, 5));
  EXPECT_FALSE(G.has_edge(1, 4));

  EXPECT_EQ(G.degree(1), 1);
  EXPECT_EQ(G.degree(2), 2);
  EXPECT_EQ(G.degree(4), 1);

  EXPECT_EQ(G.neighbors(1), VI({2}));
  EXPECT_EQ(G.neighbors(2), VI({1, 4}));
  EXPECT_EQ(G.neighbors(4), VI({2}));

  EXPECT_EQ(G.get_colors(1).to_vector(), VI({3, 7}));
  EXPECT_EQ(G.get_colors(2).to_vector(), VI({}));
  EXPECT_EQ(G.get_colors(4).to_vector(), VI({1, 3, 9}));

  EXPECT_EQ(G.get_weight(1, 2), 0.5);
  EXPECT_EQ(G.get_weight(2, 1), 0.5);
  EXPECT_EQ(G.get_weight(2, 4), 1.3);
  EXPECT_EQ(G.get_weight(4, 2), 1.3);

  // I/O
  stringstream ss;
  ss << G;
  EXPECT_EQ(ss.str(), "Graph(n=3,m=2)");

  // modification
  G.add_vertex(7, VI(3, 6));
  G.add_vertex(9, VI(1, 4));
  G.add_edge(4, 7, 10);
  G.add_edge(1, 7, 3.1);
  G.remove_edge(2, 4);
  G.remove_vertex(1);

  EXPECT_EQ(G.number_of_vertices(), 4);
  EXPECT_EQ(G.number_of_edges(), 1);
  EXPECT_EQ(G.number_of_colors(), 5);

  EXPECT_EQ(G.neighbors(2), VI({}));
  EXPECT_EQ(G.neighbors(4), VI({7}));
  EXPECT_EQ(G.neighbors(7), VI({4}));
}
