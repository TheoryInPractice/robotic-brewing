#include <gtest/gtest.h>

#include "math/number.hpp"

typedef std::vector<int> VI;

//
// NumberTest
//
TEST(NumberTest, PrimeFactors) {
  EXPECT_EQ(math::prime_factors(1), VI({}));
  EXPECT_EQ(math::prime_factors(2), VI({2}));
  EXPECT_EQ(math::prime_factors(3), VI({3}));
  EXPECT_EQ(math::prime_factors(4), VI({2}));
  EXPECT_EQ(math::prime_factors(6), VI({2, 3}));
  EXPECT_EQ(math::prime_factors(30), VI({2, 3, 5}));
  EXPECT_EQ(math::prime_factors(31), VI({31}));
  EXPECT_EQ(math::prime_factors(99999997), VI({1297, 77101}));
  EXPECT_EQ(math::prime_factors(99999989), VI({99999989}));
}

TEST(NumberTest, IsPrime) {
  EXPECT_FALSE(math::is_prime(1));
  EXPECT_TRUE(math::is_prime(2));
  EXPECT_TRUE(math::is_prime(3));
  EXPECT_FALSE(math::is_prime(4));
  EXPECT_FALSE(math::is_prime(6));
  EXPECT_FALSE(math::is_prime(30));
  EXPECT_TRUE(math::is_prime(31));
  EXPECT_FALSE(math::is_prime(99999997));
  EXPECT_TRUE(math::is_prime(99999989));
}

TEST(NumberTest, ModInv) {
  EXPECT_EQ(math::modinv(1, 7), 1);
  EXPECT_EQ(math::modinv(2, 7), 4);
  EXPECT_EQ(math::modinv(3, 7), 5);
  EXPECT_EQ(math::modinv(4, 7), 2);
  EXPECT_EQ(math::modinv(5, 7), 3);
  EXPECT_EQ(math::modinv(6, 7), 6);
}
