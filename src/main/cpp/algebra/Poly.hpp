#pragma once

#include <algorithm>
#include <iostream>
#include <vector>

namespace algebra {

/**
 * @brief Represents a polynomial of one variable.
 *
 * @tparam Data data type to store each coefficient
 */
template <typename Data>
class Poly {
 private:
  typedef Poly<Data> Self;

 public:
  /** Coefficients; coef[i] represents the coefficient of x^i. */
  std::vector<Data> coef;

  Poly() : coef(1) {}

  Poly(Data value) : coef({value}) {}

  Poly(std::vector<Data> const& coef) : coef(coef.empty() ? std::vector<Data>({0}) : coef) {}

  Poly(std::initializer_list<Data> const& coef) {
    for (auto x : coef) this->coef.push_back(x);
    if (this->coef.empty()) this->coef.push_back(0);
  }

  /**
   * @brief Returns the size of the coefficient vector.
   *
   * @return std::size_t size of coefficients
   */
  std::size_t size() const { return coef.size(); }

  /**
   * @brief Returns the degree of the polynomial.
   *
   * @return int degree
   */
  std::size_t degree() const {
    assert(size() != 0);
    std::size_t i = size() - 1;
    while (i >= 1 && coef[i] == 0) --i;
    return i;
  }

  /**
   * @brief Returns the number of nonzero monomials.
   *
   * @return std::size_t number of nonzero monomials
   */
  std::size_t number_of_nonzeros() const { return size() - std::count(coef.begin(), coef.end(), 0); }

  std::vector<std::size_t> nonzero_degrees() const {
    std::vector<std::size_t> ret;
    for (std::size_t i = 0; i < size(); ++i) {
      if (coef[i] != 0) ret.push_back(i);
    }
    return ret;
  }

  void compact() { coef.resize(degree() + 1); }

  //----------------------------------------------------------------------------
  //    Equality
  //----------------------------------------------------------------------------
  friend bool operator==(Self const& lhs, Self const& rhs) {
    for (std::size_t i = 0; i < std::max(lhs.size(), rhs.size()); ++i) {
      if (i >= lhs.size()) {
        if (rhs.coef[i] != 0) return false;
      } else if (i >= rhs.size()) {
        if (lhs.coef[i] != 0) return false;
      } else {
        if (lhs.coef[i] != rhs.coef[i]) return false;
      }
    }
    return true;
  }

  friend bool operator==(Self const& lhs, Data rhs) { return lhs == Self(rhs); }
  friend bool operator==(Data rhs, Self const& lhs) { return lhs == Self(rhs); }
  friend bool operator!=(Self const& lhs, Self const& rhs) { return !(lhs == rhs); }
  explicit operator bool() { return *this != Self(0); }

  //----------------------------------------------------------------------------
  //    Addition
  //----------------------------------------------------------------------------
  Self& operator+=(Self const& rhs) {
    for (std::size_t i = 0; i < rhs.size(); ++i) {
      if (i >= size()) {
        coef.push_back(rhs.coef[i]);
      } else {
        coef[i] += rhs.coef[i];
      }
    }
    return *this;
  }
  friend Self operator+(Self lhs, Self const& rhs) { return lhs += rhs; }

  //----------------------------------------------------------------------------
  //    Subtraction
  //----------------------------------------------------------------------------
  Self& operator-=(Self const& rhs) {
    for (std::size_t i = 0; i < rhs.size(); ++i) {
      if (i >= size()) {
        coef.push_back(-rhs.coef[i]);
      } else {
        coef[i] -= rhs.coef[i];
      }
    }
    return *this;
  }
  friend Self operator-(Self lhs, Self const& rhs) { return lhs -= rhs; }

  //----------------------------------------------------------------------------
  //    Scalar Multiplication
  //----------------------------------------------------------------------------
  Self& operator*=(Data value) {
    for (std::size_t i = 0; i < size(); ++i) coef[i] *= value;
    return *this;
  }
  friend Self operator*(Self lhs, Data rhs) { return lhs *= rhs; }
  friend Self operator*(Data rhs, Self lhs) { return lhs *= rhs; }

  //----------------------------------------------------------------------------
  //    Multiplication
  //----------------------------------------------------------------------------
  Self& operator*=(Self const& rhs) {
    auto a = nonzero_degrees();
    auto b = rhs.nonzero_degrees();
    std::vector<Data> ret;

    if (a.empty() || b.empty()) {
      ret.push_back(0);
    } else {
      ret.resize(a.back() + b.back() + 1);
      for (auto i : a) {
        for (auto j : b) ret[i + j] += coef[i] * rhs.coef[j];
      }
    }
    coef = ret;
    return *this;
  }
  friend Self operator*(Self lhs, Self const& rhs) { return lhs *= rhs; }

  //----------------------------------------------------------------------------
  //    Scalar Division
  //----------------------------------------------------------------------------
  Self& operator/=(Data value) {
    for (std::size_t i = 0; i < size(); ++i) coef[i] /= value;
    return *this;
  }
  friend Self operator/(Self lhs, Data rhs) { return lhs /= rhs; }
  friend Self operator/(Data rhs, Self lhs) { return lhs /= rhs; }

  //----------------------------------------------------------------------------
  //    Modulo
  //----------------------------------------------------------------------------
  /**
   * @brief Computes polynomial modulo.
   *
   * Precondition: `Data` is a field or `rhs` is monic (the leading coefficient is 1).
   *
   * @param rhs polynomial
   * @return Self& reference to this instance
   */
  Self& operator%=(Self const& rhs) {
    auto a = nonzero_degrees();
    auto b = rhs.nonzero_degrees();
    if (b.empty()) throw std::invalid_argument("zero division");
    if (a.empty()) return *this;
    if (b.back() == 0) {  // modulo by a constant
      coef.clear();
      coef.push_back(0);
      return *this;
    }

    for (std::size_t i = size() - 1; i >= b.back(); --i) {
      if (coef[i] == 0) continue;

      auto factor = coef[i] / rhs.coef[b.back()];
      for (std::size_t j = 0; j < b.size() - 1; ++j) coef[i - (b.back() - b[j])] -= factor * rhs.coef[b[j]];
    }
    coef.resize(b.back());
    return *this;
  }
  friend Self operator%(Self lhs, Self const& rhs) { return lhs %= rhs; }

  //----------------------------------------------------------------------------
  //    Power
  //----------------------------------------------------------------------------
  /**
   * @brief Computes the `k`-th power of `x` modulo `m`.
   *
   * @param x base polynomial
   * @param k exponent
   * @param m modulus polynomial
   * @return Self k-th power of x modulo m
   *
   * Time complexity: O(log k) * {time for multiplication and modulo}
   */
  friend Self pow(Self const& x, int k, Self const& m) {
    assert(k >= 0);
    Self ret(1), p(x);

    while (k) {
      if (k & 1) ret = ret * p % m;
      p = p * p % m;
      k >>= 1;
    }
    return ret;
  }

  /**
   * @brief Computes the (2^`k`)-th power of `x` modulo `m`.
   *
   * @param x base polynomial
   * @param k log 2 of exponent
   * @param m modulus polynomial
   * @return Self (2^k)-th power of x modulo m
   */
  friend Self pow2(Self const& x, int k, Self const& m) {
    assert(k >= 0);
    Self ret(x);
    for (int i = 0; i < k; ++i) ret = ret * ret % m;
    return ret;
  }

  //----------------------------------------------------------------------------
  //    Greatest Common Divisor
  //----------------------------------------------------------------------------
  /**
   * @brief Computes the greatest common divisor of two polynomials.
   *
   * Precondition: `Data` must be a field.
   *
   * @param lhs Poly instance
   * @param rhs Poly instance
   * @return Self GCD polynomial
   */
  friend Self gcd(Self const& lhs, Self const& rhs) {
    auto x = lhs;
    auto y = rhs;

    while (y) {
      auto z = x % y;
      x = y;
      y = z;
    }

    // Make the GCD polynomial monic.
    x.compact();
    if (x.coef.back() != 1) x /= x.coef.back();
    return x;
  }

  //----------------------------------------------------------------------------
  //    Printing
  //----------------------------------------------------------------------------
  friend std::ostream& operator<<(std::ostream& stream, Poly<Data> const& poly) {
    stream << "Poly(";
    for (std::size_t i = 0; i < poly.size(); ++i) {
      if (i > 0) stream << ", ";
      stream << poly.coef[i];
    }
    stream << ")";

    return stream;
  }
};

}  // namespace algebra
