#include <gtest/gtest.h>

#include "algebra/ring/TropicalSemiring.hpp"

using namespace std;
using namespace algebra::ring;

typedef std::vector<int> VI;
typedef std::pair<int, int> II;
typedef std::vector<II> VII;
typedef std::map<II, int> M;

//
// TropicalSemiringTest
//
TEST(TropicalSemiringTest, Addition) {
  TropicalSemiring a, b(2), c(5);

  EXPECT_EQ(a + a, TropicalSemiring(0));
  EXPECT_EQ(a + b, TropicalSemiring(2));
  EXPECT_EQ(c + a, TropicalSemiring(5));
  EXPECT_EQ(b + c, TropicalSemiring(5));

  EXPECT_EQ(a * a, TropicalSemiring(0));
  EXPECT_EQ(a * b, TropicalSemiring(2));
  EXPECT_EQ(c * a, TropicalSemiring(5));
  EXPECT_EQ(b * c, TropicalSemiring(7));
}
