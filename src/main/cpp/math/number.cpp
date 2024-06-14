#include "math/number.hpp"

#include <cassert>

namespace math {

/**
 * @brief Computes a list of prime factors of the given integer.
 *
 * @param n number
 * @return std::vector<int> list of prime factors
 *
 * Time complexity: O(n^{0.5})
 */
std::vector<int> prime_factors(int n) {
  if (n <= 1) return {};

  std::vector<int> ret;

  for (int m = 2; m * m <= n && n > 1; ++m) {
    if (n % m == 0) {
      ret.push_back(m);
      while (n % m == 0) n /= m;
    }
  }

  if (n > 1) ret.push_back(n);
  return ret;
}

/**
 * @brief Checks if the given integer is prime or not.
 *
 * @param n number
 * @return true `n` is prime
 * @return false `n` is not prime
 *
 * Time complexity: O(n^{0.5})
 */
bool is_prime(int n) {
  if (n <= 1) return false;
  for (int m = 2; m * m <= n; ++m) {
    if (n % m == 0) return false;
  }
  return true;
}

/**
 * @brief Computes modular inverse, n^{-1} modulo m.
 *
 * @param n number
 * @param m modulus; this must be a prime number
 * @return int modular inverse
 *
 * Time complexity: O(log n)
 */
int modinv(int n, int m) {
  if (m == 1) return 0;
  assert(n > 0);
  assert(is_prime(m));

  long long m0 = m;
  long long y = 0;
  long long x = 1;

  while (n > 1) {
    if (m == 0) return -1;
    long long q = n / m;
    long long t = m;

    m = n % m;
    n = t;
    t = y;
    y = x - q * y;
    x = t;
  }

  if (x < 0) x = x + m0;
  return x;
}
}  // namespace math
