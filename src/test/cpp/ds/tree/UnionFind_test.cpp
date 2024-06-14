#include <gtest/gtest.h>

#include "ds/tree/UnionFind.hpp"

using namespace std;
using namespace ds::tree;

TEST(UnionFind, Union) {
  UnionFind s;

  EXPECT_EQ(s.rank(1), 1);
  EXPECT_EQ(s.rank(3), 1);

  EXPECT_EQ(s.root(1), 1);
  EXPECT_EQ(s.root(3), 3);
  EXPECT_EQ(s.root(5), 5);

  EXPECT_TRUE(s.Union(1, 3));
  EXPECT_EQ(s.rank(1), 2);
  EXPECT_EQ(s.rank(3), 2);

  EXPECT_TRUE(s.Union(101, 102));
  EXPECT_TRUE(s.Union(102, 103));
  EXPECT_TRUE(s.Union(103, 104));
  EXPECT_TRUE(s.Union(104, 105));
  EXPECT_FALSE(s.Union(104, 103));
  EXPECT_FALSE(s.Union(102, 102));
  EXPECT_EQ(s.rank(101), 5);
  EXPECT_EQ(s.rank(104), 5);

  EXPECT_TRUE(s.Union(3, 103));
  EXPECT_EQ(s.rank(1), 7);
  EXPECT_FALSE(s.Union(1, 104));
}
