#include <gtest/gtest.h>

#include "algebra/FinitePoly.hpp"
#include "algebra/Poly.hpp"
#include "util/Random.hpp"

using namespace std;

typedef algebra::FinitePoly<uint32_t> P;

//
// FinitePolyTest
//
TEST(FinitePolyTest, Equality) {
  int ell = 7;
  EXPECT_EQ(P(), P(0));
  EXPECT_NE(P(), P(1));
  EXPECT_EQ(P(ell), P(ell, {0}));
  EXPECT_EQ(P(ell, 0), P(ell, {0}));
  EXPECT_EQ(P(ell, 1), P(ell, {1}));
  EXPECT_NE(P(ell, {3, 2}), P(ell, {2, 3}));
  EXPECT_EQ(P(ell, {0, 1, 0, 1, 0}), P(ell, {0, 1, 0, 1, 0}));
  EXPECT_EQ(P(ell, {0, 1, 0, 2}), P(ell, {0, 1, 0, 2, 0, 0}));
  EXPECT_EQ(P(ell, {0, 1, 0, 2, 0, 0}), P(ell, {0, 1, 0, 2}));
}

TEST(FinitePolyTest, Size) {
  int ell = 7;
  EXPECT_EQ(P(ell).size(), 1);
  EXPECT_EQ(P(ell, 2).size(), 1);
  EXPECT_EQ(P(ell, {3, 4, 5}).size(), 3);
  EXPECT_EQ(P(ell, {1, 2, 0, 3, 4, 0, 0}).size(), 7);
}

TEST(FinitePolyTest, Addition) {
  int ell = 7;
  P a(ell, {0, 1, 3});
  P b(ell, {2, 0, 9});
  P c(ell, {0, 0, 0, 1});
  EXPECT_EQ(a + b, P(ell, {2, 1, 12}));
  EXPECT_EQ(a + c, P(ell, {0, 1, 3, 1}));
  EXPECT_EQ(c + b, P(ell, {2, 0, 9, 1}));
}

TEST(FinitePolyTest, Subtraction) {
  int ell = 7;
  P a(ell, {0, 1, 3});
  P b(ell, {2, 0, 9});
  P c(ell, {0, 0, 0, 1});
  EXPECT_EQ(a - b, P(ell, {4294967294U, 1, 4294967290U}));
  EXPECT_EQ(a - c, P(ell, {0, 1, 3, 4294967295U}));
  EXPECT_EQ(c - b, P(ell, {4294967294U, 0, 4294967287U, 1}));
}

TEST(FinitePolyTest, ScalarMultiplication) {
  int ell = 7;
  P a(ell, {0, 1, 3});
  P c(ell, {0, 0, 0, 1});

  EXPECT_EQ(a * 3, P(ell, {0, 3, 9}));
  EXPECT_EQ(3 * a, P(ell, {0, 3, 9}));
  EXPECT_EQ(c * 2, P(ell, {0, 0, 0, 2}));
  EXPECT_EQ(2 * c, P(ell, {0, 0, 0, 2}));
}

TEST(FinitePolyTest, Multiplication) {
  P a(5, {1, 1, 1, 1, 1});
  P b(5, {1, 2, 3, 2, 1});

  EXPECT_EQ(a * b, P(5, {4294967290U, 4294967293U, 4294967292U, 1, 6}));
}

TEST(FinitePolyTest, MultiplicationProperty) {
  util::Random rand(12345);

  for (std::size_t ell = 1; ell <= 64; ++ell) {
    for (int t = 0; t < 100; ++t) {
      std::vector<uint32_t> ac, bc;
      auto acs = rand.randint<int>(1, ell);
      auto bcs = rand.randint<int>(1, ell);
      for (int i = 0; i < acs; ++i) ac.push_back(rand.randint(0U, 4294967295U));
      for (int i = 0; i < bcs; ++i) bc.push_back(rand.randint(0U, 4294967295U));

      auto a = P(ell, ac);
      auto b = P(ell, bc);
      auto x = a * b;

      EXPECT_LE(x.size(), ell);
      EXPECT_EQ(x, b * a);

      using Poly = algebra::Poly<uint32_t>;
      auto y = Poly(ac) * Poly(bc) % Poly(a.irreducible_poly());
      EXPECT_EQ(Poly(x.coefficients()), y);
    }
  }
}

TEST(FinitePolyTest, ScalarModulo) {
  int ell = 7;
  P a(ell, {5, 10, 2, 8, 7, 6, 9});

  EXPECT_EQ(a % 3, P(ell, {2, 1, 2, 2, 1, 0, 0}));
}

TEST(FinitePolyTest, Print) {
  P a(4, {4, 3, 2});
  std::stringstream ss;
  ss << a;
  EXPECT_EQ(ss.str(), "FinitePoly(ell=4, coef=4, 3, 2)");
}
