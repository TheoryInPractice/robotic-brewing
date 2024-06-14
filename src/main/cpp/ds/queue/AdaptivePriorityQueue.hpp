#pragma once

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <vector>

namespace ds {
namespace queue {

/**
 * @brief A priority queue implementation with a uniqueness guarantee.
 *
 * Each element in the queue is the pair of a key and a value (the lower, the better).
 * There are two cases when a key-value pair is pushed into the queue.
 * If the key is not in the queue, the push operation works the same as
 * the standard priority queue does. If the key already exists in the queue, then
 * the value associated with the existing key will be updated, and no new element will be inserted.
 *
 * @tparam Key type of hashable keys
 * @tparam Value type of comparable values
 */
template <typename Key, typename Value>
class AdaptivePriorityQueue {
 private:
  using Index = int;
  int const NOT_FOUND = -1;
  // int n_;
  std::unordered_map<Key, Index> location_;  // mapping from Key to Index
  std::unordered_map<Key, Value> values_;    // mapping from Key to Value
  std::vector<Index> data_;                  // stores a binary tree of Keys

 public:
  AdaptivePriorityQueue() {}

  std::size_t size() const { return data_.size(); }

  bool empty() const { return size() == 0; }

  void clear() {
    location_.clear();
    values_.clear();
    data_.clear();
  }

  void push(Key key, Value value) {
    Index index;

    auto it = location_.find(key);
    if (it == location_.end()) {
      // insert
      index = size();
      data_.push_back(key);
      location_[key] = index;
    } else if (value == values_.at(key)) {
      return;  // same value; early return
    } else {
      index = it->second;
    }

    // update value
    values_[key] = value;

    // sort nodes
    if (index > 0 && values_.at(data_[parent(index)]) > value) {
      bubble_up(index);
    } else {
      bubble_down(index);
    }
  }

  std::pair<Key, Value> top() const {
    if (empty()) throw std::invalid_argument("empty queue");
    auto key = data_[0];
    return {key, values_.at(key)};
  }

  void pop() {
    if (empty()) throw std::invalid_argument("empty queue");

    auto key = data_[0];

    // move the last element to the root
    auto j = size() - 1;
    data_[0] = data_[j];
    location_[data_[j]] = 0;

    data_.pop_back();
    location_.erase(key);

    if (!empty()) bubble_down(0);
  }

 private:
  inline Index parent(Index index) const { return (index - 1) / 2; }
  inline Index left_child(Index index) const { return index * 2 + 1; }
  inline Index right_child(Index index) const { return index * 2 + 2; }

  void swap(Index i, Index j) {
    if (i == j) return;
    auto x = data_[i];
    auto y = data_[j];
    data_[i] = y;
    data_[j] = x;
    location_[x] = j;
    location_[y] = i;
  }

  void bubble_up(Index i) {
    while (i > 0) {
      auto j = parent(i);
      if (values_[data_[j]] <= values_.at(data_[i])) return;  // done
      swap(i, j);
      i = j;
    }
  }

  void bubble_down(int i) {
    Index n = size();
    auto p = values_[data_[i]];

    while (i < n) {
      Index j1 = left_child(i);
      Index j2 = right_child(i);
      auto p1 = (j1 < n) ? values_.at(data_[j1]) : Value();
      auto p2 = (j2 < n) ? values_.at(data_[j2]) : Value();

      if ((j1 >= n || p <= p1) && (j2 >= n || p <= p2)) return;  // done

      // @note j2 < n implies j1 < n because j1 < j2
      Index swap_with = (j2 < n && p1 > p2) ? j2 : j1;
      swap(i, swap_with);
      i = swap_with;
    }
  }
};
}  // namespace queue
}  // namespace ds
