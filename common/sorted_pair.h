#pragma once

#include <algorithm>
#include <iostream>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/is_less_than_comparable.h"

/// @file
/// Provides drake::MakeSortedPair and drake::SortedPair for storing two
/// values of a certain type in sorted order.

namespace drake {

/// This class is similar to the std::pair class. However, this class uses a
/// pair of homogeneous types (std::pair can use heterogeneous types) and sorts
/// the first and second values such that the first value is less than or equal
/// to the second one). Note that the sort is a stable one. Thus the SortedPair
/// class is able to be used to generate keys (e.g., for std::map, etc.) from
/// pairs of objects.
///
/// @tparam T A template type that provides `operator<` and supports default
///           construction.
template <class T>
struct SortedPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SortedPair)
  static_assert(is_less_than_comparable<T>::value, "SortedPair can only be used"
      "with types that can be compared using the less-than operator "
      "(operator<).");

  /// The default constructor creates `first()` and `second()` using their
  /// respective default constructors.
  SortedPair() = default;

  /// Rvalue reference constructor, permits constructing with std::unique_ptr
  /// types, for example.
  SortedPair(T&& a, T&& b) {
    if (b < a) {
      first_ = std::move(b);
      second_ = std::move(a);
    } else {
      first_ = std::move(a);
      second_ = std::move(b);
    }
  }

  /// Constructs a %SortedPair from two objects.
  SortedPair(const T& a, const T& b) : first_(a), second_(b) {
    if (second_ < first_)
      std::swap(first_, second_);
  }

  /// Type-converting copy constructor.
  template <class U>
  SortedPair(SortedPair<U>&& u) : first_{std::forward<T>(u.first())},
      second_{std::forward<T>(u.second())} {}

  /// Resets the stored objects.
  template <class U>
  void set(U&& a, U&& b) {
    first_ = std::forward<U>(a);
    second_ = std::forward<U>(b);
    if (second_ < first_)
      std::swap(first_, second_);
  }

  /// Gets the first (according to `operator<`) of the objects.
  const T& first() const { return first_; }

  /// Gets the second (according to `operator<`) of the objects.
  const T& second() const { return second_; }

  /// Swaps `this` and `t`.
  void Swap(drake::SortedPair<T>& t) {
    std::swap(t.first_, first_);
    std::swap(t.second_, second_);
  }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const SortedPair& p) noexcept {
     using drake::hash_append;
    hash_append(hasher, p.first_);
    hash_append(hasher, p.second_);
  }

 private:
  T first_{};          // The first of the two objects, according to operator<.
  T second_{};         // The second of the two objects, according to operator<.
};

/// Support writing a SortedPair to a stream (conditional on the support of
/// writing the underlying type T to a stream).
template <typename T>
inline std::ostream& operator<<(std::ostream& out, const SortedPair<T>& pair) {
  out << "(" << pair.first() << ", " << pair.second() << ")";
  return out;
}

/// Two pairs of the same type are equal iff their members are equal after
/// sorting.
template <class T>
inline bool operator==(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(x < y) && !(y < x);
}

/// Compares two pairs using lexicographic ordering.
template <class T>
inline bool operator<(const SortedPair<T>& x, const SortedPair<T>& y) {
  return std::tie(x.first(), x.second()) < std::tie(y.first(), y.second());
}

/// Determine whether two SortedPair objects are not equal using `operator==`.
template <class T>
inline bool operator!=(
    const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(x == y);
}

/// Determines whether `x > y` using `operator<`.
template <class T>
inline bool operator>(const SortedPair<T>& x, const SortedPair<T>& y) {
  return y < x;
}

/// Determines whether `x <= y` using `operator<`.
template <class T>
inline bool operator<=(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(y < x);
}

/// Determines whether `x >= y` using `operator<`.
template <class T>
inline bool
operator>=(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(x < y);
}

/// @brief A convenience wrapper for creating a sorted pair from two objects.
/// @param x  The first_ object.
/// @param y  The second_ object.
/// @return A newly-constructed SortedPair object.
template <class T>
inline constexpr SortedPair<typename std::decay<T>::type>
MakeSortedPair(T&& x, T&& y) {
  return SortedPair<
      typename std::decay<T>::type>(std::forward<T>(x), std::forward<T>(y));
}

}  // namespace drake

namespace std {

/// Implements std::swap().
template <class T>
void swap(drake::SortedPair<T>& t, drake::SortedPair<T>& u) {
  t.Swap(u);
}

/// Provides std::hash<SortedPair<T>>.
template <class T>
struct hash<drake::SortedPair<T>>
    : public drake::DefaultHash {};
#if defined(__GLIBCXX__)
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
template <class T>
struct __is_fast_hash<hash<drake::SortedPair<T>>> : std::false_type {};
#endif

}  // namespace std
