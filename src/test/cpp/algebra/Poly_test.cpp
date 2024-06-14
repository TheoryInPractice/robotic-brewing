#include <gtest/gtest.h>

#include "algebra/Poly.hpp"
#include "algebra/ring/GaloisField.hpp"
#include "util/Random.hpp"

using namespace std;

typedef algebra::ring::GaloisField<103> F;
typedef algebra::Poly<F> P;

//
// PolyTest
//
TEST(PolyTest, Equality) {
  EXPECT_EQ(P(), P({0}));
  EXPECT_EQ(P(0), P({0}));
  EXPECT_EQ(P(1), P({1}));
  EXPECT_EQ(P(), 0);
  EXPECT_EQ(P({}), 0);
  EXPECT_EQ(0, P());
  EXPECT_NE(P({3, 2}), P({2, 3}));
  EXPECT_EQ(P({0, 1, 0, 1, 0}), P({0, 1, 0, 1, 0}));
  EXPECT_EQ(P({0, 1, 0, 2}), P({0, 1, 0, 2, 0, 0}));
  EXPECT_EQ(P({0, 1, 0, 2, 0, 0}), P({0, 1, 0, 2}));
}

TEST(PolyTest, Size) {
  EXPECT_EQ(P().size(), 1);
  EXPECT_EQ(P(2).size(), 1);
  EXPECT_EQ(P({3, 4, 5}).size(), 3);
  EXPECT_EQ(P({1, 2, 0, 3, 4, 0, 0}).size(), 7);
}

TEST(PolyTest, Degree) {
  EXPECT_EQ(P().degree(), 0);
  EXPECT_EQ(P(2).degree(), 0);
  EXPECT_EQ(P({3, 4, 5}).degree(), 2);
  EXPECT_EQ(P({1, 2, 0, 3, 4, 0, 0}).degree(), 4);
}

TEST(PolyTest, Addition) {
  P a = {0, 1, 3};
  P b = {2, 0, 9};
  P c = {0, 0, 0, 1};
  EXPECT_EQ(a + b, P({2, 1, 12}));
  EXPECT_EQ(a + c, P({0, 1, 3, 1}));
  EXPECT_EQ(c + b, P({2, 0, 9, 1}));
}

TEST(PolyTest, AdditionProperty) {
  util::Random rand(12345);
  int d = 5;

  for (int t = 0; t < 10; ++t) {
    // Create a random polynomial.
    std::vector<F> coef;
    for (int i = 0; i <= d; ++i) coef.push_back(rand.randint(1, 9));

    P p(coef);
    EXPECT_EQ(p + P(0), p);  // additive identity
  }
}

TEST(PolyTest, Subtraction) {
  P a = {0, 1, 3};
  P b = {2, 0, 9};
  P c = {0, 0, 0, 1};
  EXPECT_EQ(a - b, P({101, 1, 97}));
  EXPECT_EQ(a - c, P({0, 1, 3, 102}));
  EXPECT_EQ(c - b, P({101, 0, 94, 1}));
}

TEST(PolyTest, ScalarMultiplication) {
  P a = {0, 1, 3};
  P c = {0, 0, 0, 1};

  EXPECT_EQ(a * 3, P({0, 3, 9}));
  EXPECT_EQ(3 * a, P({0, 3, 9}));
  EXPECT_EQ(c * 2, P({0, 0, 0, 2}));
  EXPECT_EQ(2 * c, P({0, 0, 0, 2}));
}

TEST(PolyTest, Multiplication) {
  P a = {0, 1, 3};
  P b = {2, 0, 9};
  P c = {0, 0, 0, 1};
  EXPECT_EQ(a * b, P({0, 2, 6, 9, 27}));
  EXPECT_EQ(a * c, P({0, 0, 0, 0, 1, 3}));
}

TEST(PolyTest, MultiplicationProperty) {
  util::Random rand(12345);
  int d = 5;

  for (int t = 0; t < 10; ++t) {
    // Create a random polynomial.
    std::vector<F> coef;
    for (int i = 0; i <= d; ++i) coef.push_back(rand.randint(1, 9));

    P p(coef);
    EXPECT_EQ(p * P(1), p);     // multiplicative identity
    EXPECT_EQ(p * P(0), P(0));  // additive identity
  }
}

TEST(PolyTest, Modulo) {
  P a = {6, 7, 4, 3, 2};
  P b = {2, 0, 1};
  P c = b * P({1, 2, 3});

  EXPECT_EQ(a.nonzero_degrees(), std::vector<std::size_t>({0, 1, 2, 3, 4}));
  EXPECT_EQ(b.nonzero_degrees(), std::vector<std::size_t>({0, 2}));

  EXPECT_EQ(a % b, P({6, 1}));
  EXPECT_EQ(b % a, b);
  EXPECT_EQ(c % b, P(0));

  // corner cases
  EXPECT_EQ(P(0) % a, P(0));
}

TEST(PolyTest, GCD) {
  P a = {1, 2, 3};
  P b = {1, 4, 3};
  P c = {1, 1};
  P d = {1, 1, 1, 1};
  P e = {1, 0, 1};
  P f = {2, 0, 2, 0};

  EXPECT_EQ(gcd(a, b), 1);
  EXPECT_EQ(gcd(b, c), c);
  EXPECT_EQ(gcd(b, d), c);
  EXPECT_EQ(gcd(d, d), d);
  EXPECT_EQ(gcd(c, e), 1);
  EXPECT_EQ(gcd(d, e), e);
  EXPECT_EQ(gcd(e, f), e);
  EXPECT_EQ(gcd(f, e), e);
}

TEST(PolyTest, Power) {
  P a = {1, 1};
  P b = {0, 0, 0, 0, 0, 0, 1};

  EXPECT_EQ(pow(a, 0, b), P({1}));
  EXPECT_EQ(pow(a, 1, b), P({1, 1}));
  EXPECT_EQ(pow(a, 2, b), P({1, 2, 1}));
  EXPECT_EQ(pow(a, 3, b), P({1, 3, 3, 1}));
  EXPECT_EQ(pow(a, 4, b), P({1, 4, 6, 4, 1}));
  EXPECT_EQ(pow(a, 5, b), P({1, 5, 10, 10, 5, 1}));
  EXPECT_EQ(pow(a, 6, b), P({1, 6, 15, 20, 15, 6}));
  EXPECT_EQ(pow(a, 100, b), P({1, 100, 6, 93, 15, 82}));  // n choose i mod 103
}

TEST(PolyTest, Print) {
  P a = {4, 3, 2};
  std::stringstream ss;
  ss << a;
  EXPECT_EQ(ss.str(), "Poly(GF_103(4), GF_103(3), GF_103(2))");
}
