#include <gtest/gtest.h>

#include "geometry/Point.hpp"

using namespace std;
using namespace geometry;

typedef std::vector<int> VI;

TEST(PointTest, Properties) {
  Point p1(0, 0, 0), p2(0, 3, 0), p3(4, 0, 0);
  EXPECT_TRUE(p1 == Point());
  EXPECT_FALSE(p1 != Point());
  EXPECT_FALSE(p1 == p2);
  EXPECT_TRUE(p1 != p2);
  EXPECT_FALSE(bool(p1));
  EXPECT_TRUE(bool(p2));

  EXPECT_EQ(p2 + p3, Point(4, 3, 0));
  EXPECT_EQ(-p3, Point(-4, 0, 0));
  EXPECT_EQ(p2 - p3, Point(-4, 3, 0));
  EXPECT_EQ(p2 * 2.5, Point(0, 7.5, 0));
  EXPECT_EQ(p2 / 0.6, Point(0, 5.0, 0));

  EXPECT_EQ(Point::distance(p1, p2), 3.0);
  EXPECT_EQ(Point::distance(p2, p3), 5.0);
}
