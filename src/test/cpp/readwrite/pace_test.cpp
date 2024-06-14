#include <gtest/gtest.h>

#include "readwrite/pace.hpp"

using namespace std;

typedef std::vector<int> VI;

TEST(PaceTest, LoadPace) {
  auto instance = readwrite::load_pace("src/test/resources/instances/001_tiny.gr");
  auto G = instance.graph;

  EXPECT_EQ(G.number_of_vertices(), 5);
  EXPECT_EQ(G.number_of_edges(), 4);
  EXPECT_EQ(instance.s, 0);
  EXPECT_EQ(instance.t, 0);
  EXPECT_EQ(instance.k, 4);

  EXPECT_EQ(G.get_colors(0), VI({}));
  EXPECT_EQ(G.get_colors(1), VI({0, 1, 2}));
  EXPECT_EQ(G.get_colors(2), VI({1, 3}));
  EXPECT_EQ(G.get_colors(3), VI({1}));
  EXPECT_EQ(G.get_colors(4), VI({3}));

  EXPECT_EQ(G.get_weight(0, 1), 3);
  EXPECT_EQ(G.get_weight(0, 2), 6);
  EXPECT_EQ(G.get_weight(0, 3), 2);
  EXPECT_EQ(G.get_weight(3, 4), 3);

  auto instance2 = readwrite::load_pace("src/test/resources/instances/005_real_weight.gr");
  auto G2 = instance2.graph;
  EXPECT_EQ(G2.number_of_vertices(), 5);
  EXPECT_EQ(G2.number_of_edges(), 4);
  EXPECT_EQ(instance2.s, 0);
  EXPECT_EQ(instance2.t, 0);
  EXPECT_EQ(instance2.k, 4);

  EXPECT_EQ(G2.get_colors(0), VI({}));
  EXPECT_EQ(G2.get_colors(1), VI({0, 1, 2}));
  EXPECT_EQ(G2.get_colors(2), VI({1, 3}));
  EXPECT_EQ(G2.get_colors(3), VI({1}));
  EXPECT_EQ(G2.get_colors(4), VI({3}));

  EXPECT_EQ(G2.get_weight(0, 1), 0.03);
  EXPECT_EQ(G2.get_weight(0, 2), 0.06);
  EXPECT_EQ(G2.get_weight(0, 3), 0.02);
  EXPECT_EQ(G2.get_weight(3, 4), 0.03);
}
