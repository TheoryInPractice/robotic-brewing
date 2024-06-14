#pragma once

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "util/util.hpp"

namespace ds {
namespace set {
/**
 * @brief Fixed-size set representation efficient for set operations.
 */
class ArrayBitset {
 private:
  using Self = ArrayBitset;
  using IndexType = std::size_t;  // expect to be unsigned integers
  using DataType = uint64_t;
  static constexpr std::size_t const B = sizeof(DataType) * 8;  // should be 64
  static constexpr DataType const ONE = 1;

  std::size_t capacity_;
  std::size_t data_size_;
  DataType* data_ = nullptr;

 public:
  constexpr static IndexType const npos = static_cast<IndexType>(-1);

  ArrayBitset(std::size_t n = 1) : capacity_(n), data_size_((n + B - 1) / B) {
    if (n == 0) throw std::invalid_argument("n must be positive");
    if (data_size_ > 0) {
      data_ = new DataType[data_size_]();  // initialize with 0's
      if (!data_) throw std::invalid_argument("memory allocation error");
    }
  }

  /**
   * @brief Constructs from a list.
   */
  ArrayBitset(std::size_t n, std::vector<IndexType> const& xs) : ArrayBitset(n) {
    for (auto x : xs) *this |= x;
  }

  /**
   * @brief Constructs from a bitmask of the first block.
   */
  ArrayBitset(std::size_t n, DataType mask) : ArrayBitset(n) { data_[0] = mask; }

  /**
   * @brief Copy constructor.
   */
  ArrayBitset(ArrayBitset const& other) {
    capacity_ = other.capacity_;
    data_size_ = other.data_size_;
    data_ = new DataType[data_size_];
    if (!data_) throw std::invalid_argument("memory allocation error");

    std::copy(other.data_, other.data_ + other.data_size_, data_);
  }

  /**
   * @brief Assignment operator.
   */
  void operator=(ArrayBitset const& other) {
    capacity_ = other.capacity_;
    data_size_ = other.data_size_;
    if (data_) delete[] data_;
    data_ = new DataType[data_size_];
    if (!data_) throw std::invalid_argument("memory allocation error");

    std::copy(other.data_, other.data_ + other.data_size_, data_);
  }

  ~ArrayBitset() {
    if (data_) {
      delete[] data_;
      data_ = nullptr;
    }
  }

  inline void verify_argument(IndexType x, char const* label) const {
    assert(x < capacity_);  // bound check
  }

  void clear() { std::fill(data_, data_ + data_size_, 0); }

  bool empty() const {
    for (std::size_t i = 0; i < data_size_; ++i) {
      if (data_[i]) return false;
    }
    return true;
  }

  IndexType front() const {
    IndexType ret = npos;
    for (std::size_t i = 0; i < data_size_; ++i) {
      if (data_[i]) {
        ret = i * B + __builtin_ctzll(data_[i]);
        break;
      }
    }
    return ret < capacity_ ? ret : npos;
  }

  IndexType pop_front() {
    auto ret = front();
    if (ret != npos) reset(ret);
    return ret;
  }

  IndexType back() const {
    IndexType ret = npos;
    for (IndexType i = data_size_ - 1; i != npos; --i) {
      if (data_[i]) {
        ret = i * B + B - 1 - __builtin_clzll(data_[i]);
        break;
      }
    }
    return ret < capacity_ ? ret : npos;
  }

  IndexType pop_back() {
    auto ret = back();
    if (ret != npos) reset(ret);
    return ret;
  }

  //--------------------------------------------------------
  //    Operators
  //--------------------------------------------------------

  /**
   * negation (not)
   */
  Self operator~() const {
    Self ret(capacity_);
    for (std::size_t i = 0; i < data_size_; ++i) ret.data_[i] = ~data_[i];
    // trim extra bits
    if (data_size_ > 0 && capacity_ % B) ret.data_[data_size_ - 1] &= (ONE << (capacity_ % B)) - 1;
    return ret;
  }

  /**
   * set/union (or)
   */
  Self& operator|=(IndexType x) {
    verify_argument(x, "|=");

    data_[x / B] |= ONE << (x % B);
    return *this;
  }

  Self& operator|=(Self const& rhs) {
    if (capacity_ != rhs.capacity_) throw std::invalid_argument("inconsistent size");

    for (std::size_t i = 0; i < data_size_; ++i) data_[i] |= rhs.data_[i];
    return *this;
  }

  /**
   * exclusive or (xor)
   */
  Self& operator^=(IndexType x) {
    verify_argument(x, "^=");

    data_[x / B] ^= ONE << (x % B);
    return *this;
  }

  Self& operator^=(Self const& rhs) {
    if (capacity_ != rhs.capacity_) throw std::invalid_argument("inconsistent size");

    for (std::size_t i = 0; i < data_size_; ++i) data_[i] ^= rhs.data_[i];
    return *this;
  }

  /**
   * intersection (and)
   */
  Self operator&=(IndexType x) {
    verify_argument(x, "&=");

    for (std::size_t i = 0; i < data_size_; ++i) {
      if (i == x / B) {
        data_[x / B] &= ONE << (x % B);
      } else {
        data_[i] = 0;
      }
    }
    return *this;
  }

  Self& operator&=(Self const& rhs) {
    if (capacity_ != rhs.capacity_) throw std::invalid_argument("inconsistent size");

    for (std::size_t i = 0; i < data_size_; ++i) data_[i] &= rhs.data_[i];
    return *this;
  }

  /**
   * reset/set minus
   */
  Self& operator-=(IndexType x) {
    verify_argument(x, "-=");

    data_[x / B] &= ~(ONE << (x % B));
    return *this;
  }

  Self& operator-=(Self const& rhs) {
    if (capacity_ != rhs.capacity_) throw std::invalid_argument("inconsistent size");

    for (std::size_t i = 0; i < data_size_; ++i) data_[i] &= ~rhs.data_[i];
    return *this;
  }

  // clang-format off
  friend Self operator|(Self const& lhs, IndexType x) { Self ret(lhs); ret |= x; return ret; }
  friend Self operator|(Self const& lhs, Self const& rhs) { Self ret(lhs); ret |= rhs; return ret; }
  friend Self operator^(Self const& lhs, IndexType x) { Self ret(lhs); ret ^= x; return ret; }
  friend Self operator^(Self const& lhs, Self const& rhs) { Self ret(lhs); ret ^= rhs; return ret; }
  friend Self operator&(Self const& lhs, IndexType x) { Self ret(lhs); ret &= x; return ret; }
  friend Self operator&(Self const& lhs, Self const& rhs) { Self ret(lhs); ret &= rhs; return ret; }
  friend Self operator-(Self const& lhs, IndexType x) { Self ret(lhs); ret -= x; return ret; }
  friend Self operator-(Self const& lhs, Self const& rhs) { Self ret(lhs); ret -= rhs; return ret; }
  // clang-format on

  /**
   * get
   */
  bool operator[](IndexType x) const {
    verify_argument(x, "[]");
    return (data_[x / B] >> (x % B)) & ONE;
  }

  //----------------------------------------------------------------------------
  //    Support for range-based for-loop
  //----------------------------------------------------------------------------
  class ConstIterator {
   private:
    DataType const* const data_;
    std::size_t const data_size_;
    std::size_t index_;
    DataType x_;

   public:
    ConstIterator(DataType const* data, std::size_t data_size, std::size_t index, DataType x)
        : data_(data), data_size_(data_size), index_(index), x_(x) {}

    bool operator!=(ConstIterator const& other) const { return index_ != other.index_ || x_ != other.x_; }
    IndexType operator*() const { return index_ * B + __builtin_ctzll(x_); }
    ConstIterator const& operator++() {
      if (!x_) return *this;  // already done

      x_ &= x_ - 1;  // drop the least significant bit
      if (!x_) {
        ++index_;
        while (index_ < data_size_ && !(x_ = data_[index_])) ++index_;
      }
      return *this;
    }
  };

  ConstIterator begin() const {
    for (std::size_t i = 0; i < data_size_; ++i) {
      if (data_[i]) return ConstIterator(data_, data_size_, i, data_[i]);
    }
    return end();
  }
  ConstIterator end() const { return ConstIterator(data_, data_size_, data_size_, 0); }

  //----------------------------------------------------------------------------
  //    Convert to a vector
  //----------------------------------------------------------------------------
  std::vector<IndexType> to_vector() const {
    std::vector<IndexType> ret;
    for (std::size_t i = 0; i < data_size_; ++i) {
      for (auto x = data_[i]; x; x &= x - 1) ret.push_back(i * B + __builtin_ctzll(x));
    }
    return ret;
  }

  /**
   * equality
   */
 private:
  int compare(Self const& other) const {
    if (capacity() < other.capacity()) return -1;
    if (capacity() > other.capacity()) return 1;
    for (std::size_t i = 0; i < data_size_; ++i) {
      if (data_[i] < other.data_[i]) return -1;
      if (data_[i] > other.data_[i]) return 1;
    }
    return 0;
  }

 public:
  friend inline bool operator==(Self const& lhs, Self const& rhs) { return lhs.compare(rhs) == 0; }
  friend inline bool operator!=(Self const& lhs, Self const& rhs) { return !(lhs == rhs); }
  friend inline bool operator<(Self const& lhs, Self const& rhs) { return lhs.compare(rhs) < 0; }
  friend inline bool operator<=(Self const& lhs, Self const& rhs) { return lhs.compare(rhs) <= 0; }
  friend inline bool operator>(Self const& lhs, Self const& rhs) { return lhs.compare(rhs) > 0; }
  friend inline bool operator>=(Self const& lhs, Self const& rhs) { return lhs.compare(rhs) >= 0; }

  /**
   * @brief Returns the number of 1-bits.
   */
  std::size_t size() const {
    std::size_t ret = 0;
    for (std::size_t i = 0; i < data_size_; ++i) ret += __builtin_popcountll(data_[i]);
    return ret;
  }

  std::size_t capacity() const { return capacity_; }

  //--------------------------------------------------------
  //    Set operations
  //--------------------------------------------------------
  Self intersect(Self const& other) const { return (*this) & other; }
  Self Union(Self const& other) const { return (*this) | other; }
  bool is_subset_of(Self const& rhs) const {
    for (std::size_t i = 0; i < data_size_; ++i) {
      if ((data_[i] & rhs.data_[i]) != data_[i]) return false;
    }
    return true;
  }

  //--------------------------------------------------------
  //    Aliases
  //--------------------------------------------------------
  void set(IndexType x) { *this |= x; }
  void reset(IndexType x) { *this -= x; }
  bool get(IndexType x) const { return (*this)[x]; }
};
}  // namespace set
}  // namespace ds
