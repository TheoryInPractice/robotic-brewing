#include <gtest/gtest.h>

#include "ds/map/Bimap.hpp"

using namespace std;
using namespace ds::map;

TEST(BimapTest, BasicOperations) {
  Bimap<int, int> m1 = {{1, 3}, {2, 10}, {5, -2}};
  Bimap<int, int> m2(vector<pair<int, int>>({{1, 3}, {2, 9}, {5, -2}}));

  EXPECT_FALSE(m1 == m2);
  EXPECT_TRUE(m1 != m2);
  EXPECT_EQ(m1.size(), 3);

  EXPECT_EQ(m1.f(1), 3);
  EXPECT_EQ(m1.f(2), 10);
  EXPECT_EQ(m1.f(5), -2);
  EXPECT_EQ(m1.g(3), 1);
  EXPECT_EQ(m1.g(10), 2);
  EXPECT_EQ(m1.g(-2), 5);

  EXPECT_EQ(m1.f(1), m1.right().at(1));
  EXPECT_EQ(m1.g(3), m1.left().at(3));

  m2.update(2, 10);
  EXPECT_TRUE(m1 == m2);
  EXPECT_FALSE(m1 != m2);

  m2.clear();
  EXPECT_EQ(m2.size(), 0);
}
