#include <gtest/gtest.h>

#include "readwrite/poi.hpp"

using namespace std;
using namespace geometry;

typedef std::vector<int> VI;

TEST(POITest, LoadPOIPosition) {
  auto crisp_poi = readwrite::load_poi_position("data/poi/crisp_poi.txt");

  EXPECT_EQ(crisp_poi.size(), 49506);

  EXPECT_EQ(crisp_poi[0], Point(330, 249, 260));
  EXPECT_EQ(crisp_poi[1], Point(330, 249, 261));
  EXPECT_EQ(crisp_poi[2], Point(330, 249, 262));
  EXPECT_EQ(crisp_poi[3], Point(330, 248, 263));
  EXPECT_EQ(crisp_poi[4], Point(330, 247, 264));
  EXPECT_EQ(crisp_poi[5], Point(330, 246, 265));
  EXPECT_EQ(crisp_poi[6], Point(330, 246, 266));
  EXPECT_EQ(crisp_poi[7], Point(329, 245, 267));
  EXPECT_EQ(crisp_poi[8], Point(330, 246, 267));
  EXPECT_EQ(crisp_poi[9], Point(330, 245, 268));

  auto drone_poi = readwrite::load_poi_position("data/poi/drone_poi.txt");

  EXPECT_EQ(drone_poi.size(), 3817);

  EXPECT_EQ(drone_poi[0], Point(95.6122970581, -5, -17.7385005951));
  EXPECT_EQ(drone_poi[1], Point(99.2548980713, 0, -24.6259002686));
  EXPECT_EQ(drone_poi[2], Point(99.7789993286, -0.5586460233, -25.6168994904));
}

TEST(POITest, ReadWritePOIList) {
  auto s = stringstream("1\n3\n5\n");
  auto xs = readwrite::read_poi_list(s);

  EXPECT_EQ(xs, VI({1, 3, 5}));

  auto t = stringstream();
  readwrite::write_poi_list(t, xs);
  EXPECT_EQ(t.str(), "1\n3\n5\n");
}
