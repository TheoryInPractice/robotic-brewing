#pragma once
/**
 * @file number.hpp
 * @brief Functions related to number theory.
 */

#include <vector>

namespace math {

/**
 * @brief Computes a list of prime factors of the given integer.
 *
 * @param n number
 * @return std::vector<int> list of prime factors
 *
 * Time complexity: O(n^{0.5})
 */
std::vector<int> prime_factors(int n);

/**
 * @brief Checks if the given integer is prime or not.
 *
 * @param n number
 * @return true `n` is prime
 * @return false `n` is not prime
 *
 * Time complexity: O(n^{0.5})
 */
bool is_prime(int n);

/**
 * @brief Computes modular inverse, n^{-1} modulo m.
 *
 * @param n number
 * @param m modulus; this must be a prime number
 * @return int modular inverse
 *
 * Time complexity: O(log n)
 */
int modinv(int n, int m);
}  // namespace math
