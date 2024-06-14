#pragma once

#include <iostream>

namespace algebra {
namespace ring {

class TropicalSemiring {
 private:
  typedef int Value;

 public:
  Value value;
  TropicalSemiring(Value value = 0) : value(value) {}

  // equality
  friend bool operator==(TropicalSemiring const& lhs, TropicalSemiring const& rhs) { return lhs.value == rhs.value; }
  friend bool operator!=(TropicalSemiring const& lhs, TropicalSemiring const& rhs) { return !(lhs == rhs); }

  // addition
  TropicalSemiring& operator+=(Value rhs) {
    value = std::max(value, rhs);
    return *this;
  }
  TropicalSemiring& operator+=(TropicalSemiring rhs) { return *this += rhs.value; }
  friend TropicalSemiring operator+(TropicalSemiring lhs, TropicalSemiring const& rhs) { return lhs += rhs; }
  friend TropicalSemiring operator+(TropicalSemiring lhs, Value rhs) { return lhs += rhs; }
  friend TropicalSemiring operator+(Value rhs, TropicalSemiring lhs) { return lhs += rhs; }

  // multiplication
  TropicalSemiring& operator*=(Value rhs) {
    value += rhs;
    return *this;
  }
  TropicalSemiring& operator*=(TropicalSemiring const& rhs) { return *this *= rhs.value; }
  friend TropicalSemiring operator*(TropicalSemiring lhs, TropicalSemiring const& rhs) { return lhs *= rhs; }
  friend TropicalSemiring operator*(TropicalSemiring lhs, Value rhs) { return lhs *= rhs; }
  friend TropicalSemiring operator*(Value rhs, TropicalSemiring lhs) { return lhs *= rhs; }

  // printing
  friend std::ostream& operator<<(std::ostream& stream, TropicalSemiring const& x) { return stream << x.value; }
};
}  // namespace ring
}  // namespace algebra
