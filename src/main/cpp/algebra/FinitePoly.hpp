#pragma once

#include "algebra/irreducible.hpp"
#include <vector>

namespace algebra {

template <typename Data>
class FinitePoly {
 private:
  using Self = FinitePoly<Data>;

  std::size_t ell_;

  /** Coefficients; coef_[i] for 0 <= i < ell_ represents the coefficient of x^i. */
  std::vector<Data> coef_;

 public:
  //----------------------------------------------------------------------------
  //    Constructors
  //----------------------------------------------------------------------------
  FinitePoly(std::size_t ell = 0) : ell_(ell), coef_(ell ? 1 : 0) {
    if (ell > 64) throw std::invalid_argument("ell must be at most 64");
  }

  FinitePoly(std::size_t ell, Data value) : ell_(ell), coef_({value}) {
    if (ell > 64) throw std::invalid_argument("ell must be at most 64");
  }

  FinitePoly(std::size_t ell, std::vector<Data> const& coef)
      : ell_(ell), coef_(coef.empty() ? std::vector<Data>({0}) : coef) {
    if (ell > 64) throw std::invalid_argument("ell must be at most 64");
    if (coef.size() > ell) throw std::invalid_argument("too many coefficients");
  }

  FinitePoly(std::size_t ell, std::initializer_list<Data> const& coef) : ell_(ell) {
    if (ell > 64) throw std::invalid_argument("ell must be at most 64");
    if (coef.size() > ell) throw std::invalid_argument("too many coefficients");

    for (auto x : coef) coef_.push_back(x);
    if (coef_.empty()) coef_.push_back(0);
  }

  //----------------------------------------------------------------------------
  //    Accessors
  //----------------------------------------------------------------------------
  /**
   * @brief Returns if the instance represents a valid polynomial.
   *
   * @return true instance is valid
   * @return false instance is invalid
   */
  bool is_valid() const { return ell_; }

  /**
   * @brief Returns the coefficients of the irreducible polynomial for the instance.
   *
   * @return std::vector<Data> coefficients of the irreducible polynomial
   */
  std::vector<Data> irreducible_poly() const {
    std::vector<Data> ret(ell_ + 1);
    ret[0] = ret[ell_] = 1;
    for (auto d : BINARY_IRREDUCIBLE_POLYNOMIALS[ell_]) ret[d] = 1;
    return ret;
  }

  /**
   * @brief Returns a reference to the coefficient vector.
   *
   * @return std::vector<Data> const& coefficients
   */
  std::vector<Data> const& coefficients() const { return coef_; }

  //----------------------------------------------------------------------------
  //    Properties
  //----------------------------------------------------------------------------

  /**
   * @brief Returns the size of the coefficient vector.
   *
   * @return std::size_t size of coefficients
   */
  std::size_t size() const { return coef_.size(); }

  //----------------------------------------------------------------------------
  //    Equality
  //----------------------------------------------------------------------------
  friend bool operator==(Self const& lhs, Self const& rhs) {
    if (lhs.ell_ != rhs.ell_) return false;
    for (std::size_t i = 0; i < std::max(lhs.size(), rhs.size()); ++i) {
      if (i >= lhs.size()) {
        if (rhs.coef_[i] != 0) return false;
      } else if (i >= rhs.size()) {
        if (lhs.coef_[i] != 0) return false;
      } else {
        if (lhs.coef_[i] != rhs.coef_[i]) return false;
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
    assert(ell_ && ell_ == rhs.ell_);
    for (std::size_t i = 0; i < rhs.size(); ++i) {
      if (i >= size()) {
        coef_.push_back(rhs.coef_[i]);
      } else {
        coef_[i] += rhs.coef_[i];
      }
    }
    return *this;
  }
  friend Self operator+(Self lhs, Self const& rhs) { return lhs += rhs; }

  //----------------------------------------------------------------------------
  //    Subtraction
  //----------------------------------------------------------------------------
  friend Self operator-(Self const& p) {
    std::vector<Data> coef;
    for (auto c : p.coef_) coef.push_back(-c);
    return Self(p.ell_, coef);
  }

  Self& operator-=(Self const& rhs) { return *this += (-rhs); }
  friend Self operator-(Self lhs, Self const& rhs) { return lhs -= rhs; }

  //----------------------------------------------------------------------------
  //    Scalar Multiplication
  //----------------------------------------------------------------------------
  Self& operator*=(Data value) {
    assert(ell_);
    for (std::size_t i = 0; i < size(); ++i) coef_[i] *= value;
    return *this;
  }
  friend Self operator*(Self lhs, Data rhs) { return lhs *= rhs; }
  friend Self operator*(Data rhs, Self lhs) { return lhs *= rhs; }

  //----------------------------------------------------------------------------
  //    Multiplication
  //----------------------------------------------------------------------------
  Self& operator*=(Self const& rhs) {
    assert(ell_ && ell_ == rhs.ell_);

    auto a = size();
    auto b = rhs.size();
    std::vector<Data> ret(a + b - 1);

    // polynomial multiplication (FFT may be effective if ell > 1000)
    for (std::size_t i = 0; i < a; ++i) {
      if (!coef_[i]) continue;
      for (std::size_t j = 0; j < b; ++j) {
        if (!rhs.coef_[j]) continue;
        ret[i + j] += coef_[i] * rhs.coef_[j];
      }
    }

    // polynomial modulo
    auto ds = BINARY_IRREDUCIBLE_POLYNOMIALS[ell_];
    ds.push_back(0);
    for (std::size_t i = a + b - 2; i >= ell_; --i) {
      for (auto d : ds) ret[i - ell_ + d] -= ret[i];
    }
    ret.resize(std::min(ret.size(), ell_));
    coef_ = ret;
    return *this;
  }
  friend Self operator*(Self lhs, Self const& rhs) { return lhs *= rhs; }

  //----------------------------------------------------------------------------
  //    Scalar Modulo
  //----------------------------------------------------------------------------
  Self& operator%=(Data value) {
    assert(ell_);
    for (std::size_t i = 0; i < size(); ++i) coef_[i] %= value;
    return *this;
  }
  friend Self operator%(Self lhs, Data rhs) { return lhs %= rhs; }

  //----------------------------------------------------------------------------
  //    Printing
  //----------------------------------------------------------------------------
  friend std::ostream& operator<<(std::ostream& stream, FinitePoly<Data> const& poly) {
    stream << "FinitePoly(ell=" << poly.ell_ << ", coef=";
    for (std::size_t i = 0; i < poly.size(); ++i) {
      if (i > 0) stream << ", ";
      stream << poly.coef_[i];
    }
    stream << ")";

    return stream;
  }
};

}  // namespace algebra
