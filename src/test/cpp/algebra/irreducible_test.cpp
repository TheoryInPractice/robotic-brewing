#include <gtest/gtest.h>

#include <algorithm>

#include "algebra/Poly.hpp"
#include "algebra/irreducible.hpp"
#include "algebra/ring/GaloisField.hpp"
#include "math/number.hpp"

using namespace std;
using namespace algebra;
using namespace algebra::ring;

typedef GaloisField<2> GF;

/**
 * @brief Implements Rabin's irreducibility test.
 *
 * @param f polynomial to test
 * @return true the given polynomial is irreducible
 * @return false the given polynomial is not irreducible
 *
 * @see Rabin, M. Probabilistic algorithms in finite fields. SIAM Journal on Computing (1980), 273-280.
 *      https://apps.dtic.mil/sti/pdfs/ADA078416.pdf
 */
bool is_irreducible(Poly<GF> const& f) {
  int m = f.degree();  // q = 2

  EXPECT_GE(m, 2);
  EXPECT_NE(f.coef[0], 0);
  EXPECT_EQ(f.nonzero_degrees().size() % 2, 1);

  auto primes = math::prime_factors(m);
  std::reverse(primes.begin(), primes.end());  // in decreasing order

  Poly<GF> x = {0, 1}, one = {1};

  // Condition (1): gcd(f(x), x^(q^{m_i})) = 1 where m_i = m/p_i for prime factors p_i of m
  for (auto i : primes) {
    if (gcd(f, pow2(x, m / i, f)) != one) return false;
  }

  // Condition (2): f(x) divides x^(q^m)-x
  return (pow2(x, m, f) - x) % f == GF(0);
}

bool check_irreducibility(int k, std::vector<int> const& coef) {
  vector<GF> coeffs(k + 1);
  coeffs[0] = coeffs[k] = 1;
  for (auto j : coef) coeffs[j] = 1;

  Poly<GF> poly(coeffs);
  return is_irreducible(poly);
}

//
// IrreducibleTest
//
TEST(IrreducibleTest, IsIrreducible) {
  for (int k = 2; k <= 64; ++k) {
    // Test table entries.
    EXPECT_TRUE(check_irreducibility(k, BINARY_IRREDUCIBLE_POLYNOMIALS[k]));
  }

  // These are wrong values.
  EXPECT_FALSE(check_irreducibility(4, vector<int>({2})));
  EXPECT_FALSE(check_irreducibility(63, vector<int>({2})));
  EXPECT_FALSE(check_irreducibility(64, vector<int>({5, 3, 1})));
}
