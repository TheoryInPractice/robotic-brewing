#include <gtest/gtest.h>

#include <set>

#include "ds/set/ArrayBitset.hpp"
#include "util/Random.hpp"

using namespace ds::set;
typedef std::vector<std::size_t> VI;

TEST(ArrayBitsetTest, BasicOperations) {
  auto s = ArrayBitset(2000);
  EXPECT_TRUE(s.empty());

  s.set(0);
  EXPECT_FALSE(s.empty());
  EXPECT_TRUE(s.get(0));
  EXPECT_FALSE(s.get(1));
  EXPECT_EQ(s.size(), 1);
  EXPECT_EQ(s.to_vector(), VI({0}));

  s.reset(0);
  EXPECT_TRUE(s.empty());
  EXPECT_FALSE(s.get(0));
  EXPECT_FALSE(s.get(1));
  EXPECT_EQ(s.size(), 0);
  EXPECT_EQ(s.to_vector(), VI());

  s.set(1024);
  s.set(1025);
  s.set(1023);
  s.set(1020);
  s.set(1024);
  s.set(1020);
  s.set(1019);
  EXPECT_FALSE(s.empty());
  EXPECT_TRUE(s.get(1019));
  EXPECT_TRUE(s.get(1020));
  EXPECT_FALSE(s.get(1021));
  EXPECT_FALSE(s.get(1022));
  EXPECT_TRUE(s.get(1023));
  EXPECT_TRUE(s.get(1024));
  EXPECT_TRUE(s.get(1025));
  EXPECT_EQ(s.size(), 5);

  s.reset(1024);
  EXPECT_FALSE(s.get(1024));
  EXPECT_EQ(s.size(), 4);

  s.clear();
  EXPECT_TRUE(s.empty());
  EXPECT_FALSE(s.get(1019));
  EXPECT_EQ(s.size(), 0);
}

TEST(ArrayBitsetTest, RandomInput) {
  util::Random rand(12345);

  auto b = ArrayBitset(10000);
  std::set<int> s;

  for (int t = 0; t < 10; ++t) {
    b.clear();
    s.clear();

    for (int i = 0; i < 100; ++i) {
      int x = rand.randint(0, 9999);
      int y = rand.randint(5000, 9999);
      b.set(x);
      EXPECT_TRUE(b.get(x));
      b.reset(y);
      EXPECT_FALSE(b.get(y));

      s.insert(x);
      s.erase(y);

      EXPECT_EQ(b.size(), s.size());
      EXPECT_EQ(b.size(), b.to_vector().size());

      auto b2 = b;
      auto s2 = s;
      EXPECT_EQ(b2.size(), s2.size());
      EXPECT_EQ(b2.size(), b2.to_vector().size());
    }
  }
}

TEST(ArrayBitsetTest, PopFront) {
  auto s = ArrayBitset(10000);
  s.set(10);
  s.set(20);
  s.set(5);
  EXPECT_EQ(s.front(), 5);
  EXPECT_EQ(s.pop_front(), 5);
  EXPECT_EQ(s.pop_front(), 10);
  EXPECT_EQ(s.pop_front(), 20);
  EXPECT_EQ(s.pop_front(), ArrayBitset::npos);
}

TEST(ArrayBitsetTest, PopBack) {
  auto s = ArrayBitset(10000);
  s.set(10);
  s.set(20);
  s.set(5);
  EXPECT_EQ(s.back(), 20);
  EXPECT_EQ(s.pop_back(), 20);
  EXPECT_EQ(s.pop_back(), 10);
  EXPECT_EQ(s.pop_back(), 5);
  EXPECT_EQ(s.pop_back(), ArrayBitset::npos);
}

TEST(ArrayBitsetTest, IsSubsetOf) {
  auto s = ArrayBitset(10000);
  s.set(10);
  s.set(20);
  s.set(5);
  auto t = ArrayBitset(10000);
  t.set(20);
  t.set(5);

  EXPECT_FALSE(s.is_subset_of(t));
  EXPECT_TRUE(t.is_subset_of(s));
}

TEST(ArrayBitsetTest, Copy) {
  auto s = ArrayBitset(10000);
  s.set(10);
  s.set(20);
  s.set(5);

  ArrayBitset t(s);
  ArrayBitset u = s;

  EXPECT_EQ(t.to_vector(), VI({5, 10, 20}));
  EXPECT_EQ(u.to_vector(), VI({5, 10, 20}));

  t.reset(10);
  u.reset(10);

  EXPECT_EQ(t.to_vector(), VI({5, 20}));
  EXPECT_EQ(u.to_vector(), VI({5, 20}));

  t = s;
  u = s;

  EXPECT_EQ(t.to_vector(), VI({5, 10, 20}));
  EXPECT_EQ(u.to_vector(), VI({5, 10, 20}));
}

TEST(ArrayBitsetTest, ConstIterator) {
  auto s = ArrayBitset(10000);
  VI xs;
  for (auto x : s) xs.push_back(x);
  EXPECT_TRUE(xs.empty());

  s.set(10);
  s.set(20);
  s.set(5);
  for (auto x : s) xs.push_back(x);
  EXPECT_EQ(xs, s.to_vector());
}
