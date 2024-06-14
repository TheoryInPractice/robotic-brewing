#include <gtest/gtest.h>

#include "util/Random.hpp"

using namespace std;

//
// RandomTest
//
TEST(RandomTest, WeightedChoice) {
  util::Random rand(12345);
  std::vector<int> uniform = {3, 3, 3, 3, 3};
  std::vector<int> biased1 = {0, 29, 1, 0, 72, 5, 0};
  std::vector<double> biased2 = {0, 0, 0, 0, 1e-30};

  for (int t = 0; t < 20; ++t) {
    int ret = rand.weighted_choice(uniform);
    EXPECT_GE(ret, 0);
    EXPECT_LE(ret, 4);

    ret = rand.weighted_choice(biased1);
    EXPECT_GE(ret, 1);
    EXPECT_LE(ret, 5);
    EXPECT_NE(ret, 3);

    ret = rand.weighted_choice(biased1.begin() + 4, biased1.begin() + 6);
    EXPECT_GE(ret, 0);
    EXPECT_LE(ret, 1);

    ret = rand.weighted_choice(biased2);
    EXPECT_EQ(ret, 4);
  }
}

TEST(RandomTest, RandInt) {
  util::Random rand(12345);
  uint64_t min_val = -1, max_val = 0;
  for (int t = 0; t < 100; ++t) {
    auto x = rand.randint<uint64_t>(0, -1);
    min_val = std::min(min_val, x);
    max_val = std::max(max_val, x);
  }
  EXPECT_LT(min_val, 1ULL << 63);
  EXPECT_GT(max_val, 1ULL << 63);

  // other types
  for (int t = 0; t < 100; ++t) {
    auto a = rand.randint<int32_t>(-10, 20);
    auto b = rand.randint<uint32_t>(10, 20);
    auto c = rand.randint<int64_t>(-10, 20);
    auto d = rand.randint<uint64_t>(10, 20);
    EXPECT_LE(a, 20);
    EXPECT_GE(a, -10);
    EXPECT_LE(b, 20);
    EXPECT_GE(b, 10);
    EXPECT_LE(c, 20);
    EXPECT_GE(c, -10);
    EXPECT_LE(d, 20);
    EXPECT_GE(d, 10);
  }
}

TEST(RandomTest, SampleInt) {
  util::Random rand(12345);

  auto T = 1000;  // number of iterations
  auto b = 100;   // number of buckets

  std::vector<std::pair<int, int>> config = {
      {1000000000, 30}, {100000, 50}, {1, 0}, {1, 1}, {27, 2}, {40, 3},
  };

  for (auto &c : config) {
    auto n = c.first;
    auto k = c.second;

    std::map<int, int> distribution;

    for (int t = 0; t < T; ++t) {
      auto xs = rand.sampleint(n, k);

      // all elements must be distinct
      EXPECT_EQ(xs.size(), k);

      auto s = std::set<int>(xs.begin(), xs.end());
      EXPECT_EQ(s.size(), k);

      // elements must be in [0,n)
      for (auto x : xs) {
        EXPECT_GE(x, 0);
        EXPECT_LT(x, n);

        if (n >= 10000) ++distribution[x / (n / b)];
      }
    }

    // check for distribution
    if (n >= 10000) {
      for (int i = 0; i < b; ++i) {
        EXPECT_GE(distribution[i], 1);
        EXPECT_LE(distribution[i], T * k / b * 3);
      }
    }
  }

  // other types
  for (int t = 0; t < 100; ++t) {
    auto a = rand.sampleint<int32_t>(40, 3);
    auto b = rand.sampleint<uint32_t>(40, 3);
    auto c = rand.sampleint<int64_t>(40, 3);
    auto d = rand.sampleint<uint64_t>(40, 3);
    EXPECT_EQ(a.size(), 3);
    EXPECT_EQ(b.size(), 3);
    EXPECT_EQ(c.size(), 3);
    EXPECT_EQ(d.size(), 3);
  }
}

TEST(RandomTest, Sample) {
  util::Random rand(12345);
  std::vector<int> xs = {1, 3, 5, 7, 9};
  auto ys = rand.sample(xs, 2);
  EXPECT_EQ(ys.size(), 2);
}

TEST(RandomTest, Shuffle) {
  util::Random rand(12345);
  std::vector<int> xs = {1, 3, 5, 7, 9};
  std::vector<int> ys = xs;
  rand.shuffle(ys);
  int diff = 0;
  for (std::size_t i = 0; i < xs.size(); ++i) diff += std::abs(xs[i] - ys[i]);
  EXPECT_GT(diff, 0);
}

TEST(RandomTest, Seed) {
  util::Random rand(12345);
  auto x = rand.randint(0, 100);
  auto y = rand.randint(0, 100);
  rand.seed(12345);
  auto z = rand.randint(0, 100);
  EXPECT_EQ(x, z);
  EXPECT_NE(x, y);
}
