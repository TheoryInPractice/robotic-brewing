#include <gtest/gtest.h>

#include "readwrite/iris.hpp"

using namespace std;

typedef std::vector<int> VI;

TEST(IrisTest, LoadIris) {
  auto instance = readwrite::load_iris("src/test/resources/test_files/graph");
  auto G = instance.graph;

  EXPECT_EQ(G.number_of_vertices(), 40);
  EXPECT_EQ(G.number_of_edges(), 235);
  EXPECT_EQ(G.get_colors(2),
            VI({6897, 6910, 6911, 6915, 6917, 16790, 16791, 16800, 17001, 17003, 17009, 17036, 17045, 17046, 17047}));
  EXPECT_EQ(G.get_colors(3), VI({16760, 16784}));
  EXPECT_EQ(G.get_colors(5), VI({}));

  EXPECT_EQ(G.get_weight(0, 1), 0.01);
  EXPECT_EQ(G.get_weight(1, 2), 0.012);
  EXPECT_EQ(G.get_weight(1, 4), 0.003);
  EXPECT_EQ(G.get_weight(39, 16), 0.0223483);

  auto instance2 = readwrite::load_iris("src/test/resources/test_files/crisp");
  auto G2 = instance2.graph;
  EXPECT_EQ(G2.number_of_vertices(), 209);
  EXPECT_EQ(G2.number_of_edges(), 2132);

  EXPECT_EQ(G2.get_colors(2),
            VI({6897, 6910, 6911, 6915, 6917, 16790, 16791, 16800, 17001, 17003, 17009, 17036, 17045, 17046, 17047}));
  EXPECT_EQ(G2.get_colors(3), VI({16760, 16784}));
  EXPECT_EQ(G2.get_colors(5), VI({}));
  EXPECT_EQ(G2.get_colors(198), VI({18223}));
  EXPECT_EQ(G2.get_colors(204), VI({13770, 13780}));

  EXPECT_EQ(G2.get_weight(0, 1), 0.01);
  EXPECT_EQ(G2.get_weight(1, 2), 0.012);
  EXPECT_EQ(G2.get_weight(1, 4), 0.003);
  EXPECT_EQ(G2.get_weight(190, 114), 0.00465011);
}