#include <gtest/gtest.h>

#include "ds/set/FastSet.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace ds::set;

typedef std::vector<int> VI;

TEST(FastSetTest, BasicOperations) {
  FastSet fs1(10), fs2(10);
  EXPECT_TRUE(fs1 == fs2);
  EXPECT_FALSE(fs1 != fs2);

  fs1.set(5);
  EXPECT_FALSE(fs1 == fs2);
  EXPECT_TRUE(fs1 != fs2);

  EXPECT_EQ(fs1.capacity(), 10);
  EXPECT_EQ(fs1.size(), 1);

  fs1.set(7);
  fs1.reset(3);
  fs1.reset(5);

  EXPECT_EQ(fs1.to_vector(), VI({7}));

  fs1.resize(20);  // should retain existing data
  EXPECT_EQ(fs1.to_vector(), VI({7}));

  fs1.set(19);
  fs1.set(18);
  EXPECT_EQ(fs1.to_vector(), VI({7, 18, 19}));

  fs1.initialize(20);
  EXPECT_EQ(fs1.capacity(), 20);
  EXPECT_EQ(fs1.size(), 0);
  EXPECT_EQ(fs1.to_vector(), VI());

  fs1.set(5);
  fs1.clear();
  fs1.set(6);
  fs1.clear();
  fs1.set(7);
  EXPECT_EQ(fs1.to_vector(), VI({7}));

  fs1.resize(3);
  EXPECT_EQ(fs1.to_vector(), VI());
}

TEST(FastSetTest, RandomInput) {
  util::Random rand(12345);

  FastSet fs(11);
  std::set<int> s;

  for (int t = 0; t < 10; ++t) {
    fs.clear();
    s.clear();

    for (int i = 0; i < 100; ++i) {
      int x = rand.randint(0, 10);
      int y = rand.randint(5, 10);
      fs.set(x);
      EXPECT_TRUE(fs.get(x));
      fs.reset(y);
      EXPECT_FALSE(fs.get(y));

      s.insert(x);
      s.erase(y);

      EXPECT_EQ(fs.size(), s.size());
      EXPECT_EQ(fs.size(), fs.to_vector().size());

      auto fs2 = fs;
      auto s2 = s;
      EXPECT_EQ(fs2.size(), fs2.size());
      EXPECT_EQ(fs2.size(), fs2.to_vector().size());
    }
  }
}
