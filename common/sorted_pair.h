#pragma once

#include <algorithm>
#include <cstddef>
#include <ostream>
#include <type_traits>
#include <utility>

#include "drake/common/drake_deprecated.h"
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
/// The availability of construction and assignment operations (i.e., default
/// constructor, copy constructor, copy assignment, move constructor, move
/// assignment) is the same as whatever T provides .  All comparison operations
/// (including equality, etc.) are always available.
///
/// To format this class for logging, include `<fmt/ranges.h>` (exactly the same
/// as for `std::pair`).
///
/// @tparam T A template type that provides `operator<`.
template <class T>
struct SortedPair {
  static_assert(is_less_than_comparable<T>::value,
                "SortedPair can only be used with types that can be compared "
                "using the less-than operator (operator<).");

  /// The default constructor creates `first()` and `second()` using T's default
  /// constructor, iff T has a default constructor.  Otherwise, this constructor
  /// is not available.
#ifndef DRAKE_DOXYGEN_CXX
  template <typename T1 = T,
            typename std::enable_if_t<std::is_constructible_v<T1>, bool> = true>
#endif
  SortedPair() {
  }

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
    if (second_ < first_) {
      std::swap(first_, second_);
    }
  }

  /// Constructs a %SortedPair from a std::pair
  explicit SortedPair(const std::pair<T, T>& pair)
      : first_(pair.first), second_(pair.second) {
    if (second_ < first_) {
      std::swap(first_, second_);
    }
  }

  /// Type-converting copy constructor.
  template <class U>
  SortedPair(SortedPair<U>&& u)
      : first_{std::forward<T>(u.first())},
        second_{std::forward<T>(u.second())} {}

  // N.B. We leave all of the copy/move/assign operations implicitly declared,
  // so that iff T provides that operation, then we will also provide it.  Do
  // not declare any of those operations nor a destructor here, or else the
  // implicitly declared functions might not longer be implicitly declared.

  /// Resets the stored objects.
  template <class U>
  void set(U&& a, U&& b) {
    first_ = std::forward<U>(a);
    second_ = std::forward<U>(b);
    if (second_ < first_) {
      std::swap(first_, second_);
    }
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

  /// @name Support for using SortedPair in structured bindings.
  //@{
  template <size_t Index>
  const T& get() const {
    if constexpr (Index == 0) return first_;
    if constexpr (Index == 1) return second_;
  }
  template <std::size_t Index>
  friend const T& get(const SortedPair<T>& self) {
    return self.get<Index>();
  }
  //@}

 private:
  T first_{};   // The first of the two objects, according to operator<.
  T second_{};  // The second of the two objects, according to operator<.
};

template <typename T>
DRAKE_DEPRECATED(
    "2023-06-01",
    "Use fmt or spdlog for logging, not operator<<. "
    "Add an #include <fmt/ranges.h> to format SortedPair as a range."
    "See https://github.com/RobotLocomotion/drake/issues/17742 for background.")
// TODO(jwnimmer-tri) On 2023-06-01 also remove the <ostream> include.
inline std::ostream&
operator<<(std::ostream& out, const SortedPair<T>& pair) {
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
inline bool operator!=(const SortedPair<T>& x, const SortedPair<T>& y) {
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
inline bool operator>=(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(x < y);
}

/// @brief A convenience wrapper for creating a sorted pair from two objects.
/// @param x  The first_ object.
/// @param y  The second_ object.
/// @return A newly-constructed SortedPair object.
template <class T>
inline constexpr SortedPair<typename std::decay<T>::type> MakeSortedPair(
    T&& x, T&& y) {
  return SortedPair<typename std::decay<T>::type>(std::forward<T>(x),
                                                  std::forward<T>(y));
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
struct hash<drake::SortedPair<T>> : public drake::DefaultHash {};
#if defined(__GLIBCXX__)
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/unordered_associative.html
template <class T>
struct __is_fast_hash<hash<drake::SortedPair<T>>> : std::false_type {};
#endif

/// Support using `SortedPair<T>` in structured bindings.  E.g.,
///
///    SortedPair<Foo> pair(Foo(1), Foo(2));
///    const auto& [a, b] = pair;
template <typename T>
struct tuple_size<drake::SortedPair<T>> : std::integral_constant<size_t, 2> {};

template <size_t Index, typename T>
struct tuple_element<Index, drake::SortedPair<T>> {
  using type = const T;
};

}  // namespace std
