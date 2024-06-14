#pragma once

#include <unordered_map>
#include <vector>

#include "util/Random.hpp"

namespace ds {
namespace map {

/**
 * @brief Bi-directional map.
 *
 * @tparam Left hashable type of the domain
 * @tparam Right hashable type of the range
 */
template <typename Left, typename Right>
class Bimap {
 private:
  std::unordered_map<Left, Right> right_;
  std::unordered_map<Right, Left> left_;

 public:
  Bimap(std::vector<Right> const& right_values) {
    for (std::size_t i = 0; i < right_values.size(); ++i) update(i, right_values[i]);
  }

  Bimap(std::vector<std::pair<Left, Right>> const& relation = {}) {
    for (auto& p : relation) update(p.first, p.second);
  }

  Bimap(std::initializer_list<std::pair<Left, Right>> const& relation) {
    for (auto& p : relation) update(p.first, p.second);
  }

  friend bool operator==(Bimap<Left, Right> const& lhs, Bimap<Left, Right> const& rhs) {
    return lhs.right_ == rhs.right_;
  }
  friend bool operator!=(Bimap<Left, Right> const& lhs, Bimap<Left, Right> const& rhs) {
    return !(lhs.right_ == rhs.right_);
  }

  /**
   * @brief Clears the entire map.
   */
  void clear() {
    right_.clear();
    left_.clear();
  }

  /**
   * @brief Returns the size of the map.
   *
   * @return std::size_t map size
   */
  std::size_t size() const { return right_.size(); }

  /**
   * @brief Updates a relation.
   *
   * @param left left element
   * @param right right element
   */
  void update(Left const& left, Right const& right) {
    right_[left] = right;
    left_[right] = left;
  }

  /**
   * @brief Returns the forward map.
   *
   * @return std::unordered_map<Left, Right> const& forward map
   */
  std::unordered_map<Left, Right> const& right() const { return right_; }

  /**
   * @brief Returns the backward (reverse) map.
   *
   * @return std::unordered_map<Right, Left> const& backward map
   */
  std::unordered_map<Right, Left> const& left() const { return left_; }

  /**
   * @brief Forward lookup.
   *
   * @param x element
   * @return Right value of f(x)
   */
  Right f(Left x) const { return right_.at(x); }

  /**
   * @brief Backward lookup.
   *
   * @param y element
   * @return Left value of f^{-1}(y)
   */
  Left g(Right y) const { return left_.at(y); }
};
}  // namespace map
}  // namespace ds
