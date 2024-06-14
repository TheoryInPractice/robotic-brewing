#include <gtest/gtest.h>

#include "algorithm/graph/spanning_tree.hpp"

using namespace std;
using namespace ds::graph;

typedef std::vector<int> VI;

//
// SpanningTreeTest
//
TEST(SpanningTreeTest, MinimumSpanningTree) {
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
  EXPECT_EQ(util::sorted(algorithm::graph::minimum_spanning_tree(g)),
            ds::graph::Graph::EdgeList({{0, 1}, {1, 4}, {2, 3}, {2, 6}, {3, 5}, {4, 5}, {6, 7}}));

  Graph g2({{1, {}}, {2, {}}}, {});  // disconnected
  EXPECT_THROW(algorithm::graph::minimum_spanning_tree(g2), std::invalid_argument);
}

TEST(SpanningTreeTest, MinimumSpanningTreeCost) {
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
  EXPECT_EQ(algorithm::graph::minimum_spanning_tree_cost(g), 19);

  Graph g2({{1, {}}, {2, {}}}, {});  // disconnected
  EXPECT_THROW(algorithm::graph::minimum_spanning_tree_cost(g2), std::invalid_argument);
}
