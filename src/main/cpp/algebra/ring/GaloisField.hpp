#pragma once

#include <iostream>

#include "math/number.hpp"

namespace algebra {
namespace ring {

template <int p>
class GaloisField {
 private:
  typedef uint64_t Value;

 public:
  Value value;

  //----------------------------------------------------------------------------
  //    Constructor
  //----------------------------------------------------------------------------
  GaloisField(Value value = 0) : value(value % p) {
    if (!math::is_prime(p)) throw std::invalid_argument("non-prime p is not supported");
  }

  //----------------------------------------------------------------------------
  //    Equality
  //----------------------------------------------------------------------------
  friend bool operator==(GaloisField const& lhs, GaloisField const& rhs) { return lhs.value == rhs.value; }
  friend bool operator!=(GaloisField const& lhs, GaloisField const& rhs) { return !(lhs == rhs); }
  explicit operator bool() { return value != 0; }

  //----------------------------------------------------------------------------
  //    Addition
  //----------------------------------------------------------------------------
  GaloisField& operator+=(GaloisField const& rhs) {
    value = (value + rhs.value) % p;
    return *this;
  }
  friend GaloisField operator+(GaloisField lhs, GaloisField const& rhs) { return lhs += rhs; }

  //----------------------------------------------------------------------------
  //    Subtraction
  //----------------------------------------------------------------------------
  GaloisField operator-() const { return GaloisField<p>(p - value); }
  GaloisField& operator-=(GaloisField const& rhs) { return *this += -rhs; }
  friend GaloisField operator-(GaloisField lhs, GaloisField const& rhs) { return lhs -= rhs; }

  //----------------------------------------------------------------------------
  //    Multiplication
  //----------------------------------------------------------------------------
  GaloisField& operator*=(Value rhs) {
    value = value * rhs % p;
    return *this;
  }
  GaloisField& operator*=(GaloisField const& rhs) { return *this *= rhs.value; }
  friend GaloisField operator*(GaloisField lhs, GaloisField const& rhs) { return lhs *= rhs; }

  //----------------------------------------------------------------------------
  //    Division
  //----------------------------------------------------------------------------
  GaloisField& operator/=(GaloisField const& rhs) { return *this *= math::modinv(rhs.value, p); }
  friend GaloisField operator/(GaloisField lhs, GaloisField const& rhs) { return lhs /= rhs; }

  //----------------------------------------------------------------------------
  //    Printing
  //----------------------------------------------------------------------------
  friend std::ostream& operator<<(std::ostream& stream, GaloisField const& x) {
    return stream << "GF_" << p << "(" << x.value << ")";
  }
};
}  // namespace ring
}  // namespace algebra
