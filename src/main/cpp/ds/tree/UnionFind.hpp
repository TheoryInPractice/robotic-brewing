#pragma once

#include "util/util.hpp"

namespace ds {
namespace tree {

/**
 * @brief Implements the union-find disjoint set data structure.
 */
class UnionFind {
 public:
  UnionFind() {}

  bool Union(int x, int y) {
    if ((x = root(x)) == (y = root(y))) return false;

    if (get(y) < get(x)) std::swap(x, y);
    data_[x] = get(x) + get(y);
    data_[y] = x;
    return true;
  }

  int root(int x) {
    if (x < 0) throw std::invalid_argument("label must be non-negative");
    return get(x) < 0 ? x : data_[x] = root(data_.at(x));  // compress tree
  }

  int rank(int x) {
    if (x < 0) throw std::invalid_argument("label must be non-negative");
    return -get(root(x));
  }

 private:
  std::unordered_map<int, int> data_; /* stores parent or count */

  inline int get(int x) { return util::contains(data_, x) ? data_.at(x) : -1; }
};
}  // namespace tree
}  // namespace ds
