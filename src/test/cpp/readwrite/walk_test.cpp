#include <gtest/gtest.h>

#include "readwrite/walk.hpp"

using namespace std;

using VI = std::vector<int>;
using VVI = std::vector<VI>;

TEST(WalkTest, LoadWalks) {
  auto walks = readwrite::load_walks("src/test/resources/walks/001_tiny.txt");

  EXPECT_EQ(walks, VVI({
                       {0, 1, 0},
                       {0, 3, 0},
                       {0, 3, 4, 3, 0},
                   }));
}
