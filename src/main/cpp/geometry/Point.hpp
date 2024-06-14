#pragma once

#include <cmath>
#include <iostream>

namespace geometry {
/**
 * @brief Represents a point in the 3D Cartesian coordinates.
 */
class Point {
 public:
  double x, y, z;

  //----------------------------------------------------------------------------
  //    Constructor
  //----------------------------------------------------------------------------
  Point(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}

  //----------------------------------------------------------------------------
  //    Equality
  //----------------------------------------------------------------------------
  friend bool operator==(Point const& lhs, Point const& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
  }
  friend bool operator!=(Point const& lhs, Point const& rhs) { return !(lhs == rhs); }
  explicit operator bool() { return *this != Point(); }

  //----------------------------------------------------------------------------
  //    Addition
  //----------------------------------------------------------------------------
  Point& operator+=(Point const& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
  friend Point operator+(Point lhs, Point const& rhs) { return lhs += rhs; }

  //----------------------------------------------------------------------------
  //    Subtraction
  //----------------------------------------------------------------------------
  Point operator-() const { return Point(-x, -y, -z); }
  Point& operator-=(Point const& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }
  friend Point operator-(Point lhs, Point const& rhs) { return lhs -= rhs; }

  //----------------------------------------------------------------------------
  //    Scalar multiplication
  //----------------------------------------------------------------------------
  Point& operator*=(double s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }
  friend Point operator*(Point lhs, double s) { return lhs *= s; }

  //----------------------------------------------------------------------------
  //    Scalar division
  //----------------------------------------------------------------------------
  Point& operator/=(double s) {
    x /= s;
    y /= s;
    z /= s;
    return *this;
  }
  friend Point operator/(Point lhs, double s) { return lhs /= s; }

  //----------------------------------------------------------------------------
  //    Norm and distance
  //----------------------------------------------------------------------------
  /**
   * @brief Computes the L2 norm.
   *
   * @return double L2 norm
   */
  double norm() const { return std::sqrt(x * x + y * y + z * z); }

  /**
   * @brief Comptues the distance between two points.
   *
   * @param lhs point 1
   * @param rhs point 2
   * @return double distance
   */
  static double distance(Point const& lhs, Point const& rhs) { return (lhs - rhs).norm(); }

  //----------------------------------------------------------------------------
  //    I/O
  //----------------------------------------------------------------------------
  friend std::istream& operator>>(std::istream& stream, Point& p) { return stream >> p.x >> p.y >> p.z; }

  friend std::ostream& operator<<(std::ostream& stream, Point const& p) {
    return stream << "Point(" << p.x << ", " << p.y << ", " << p.z << ")";
  }

  //----------------------------------------------------------------------------
  //    Hash
  //----------------------------------------------------------------------------
  struct PointHash {
   private:
    std::size_t hash_combine(std::size_t& seed, std::size_t value) const {
      return seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

   public:
    std::size_t operator()(const geometry::Point& point) const {
      std::size_t seed = 0;
      this->hash_combine(seed, std::hash<double>()(point.x));
      this->hash_combine(seed, std::hash<double>()(point.y));
      this->hash_combine(seed, std::hash<double>()(point.z));
      return seed;
    }
  };

  struct PointEqual {
    bool operator()(const geometry::Point& lhs, const geometry::Point& rhs) const {
      return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }
  };
};
}  // namespace geometry
