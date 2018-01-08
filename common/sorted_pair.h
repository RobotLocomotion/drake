#pragma once

#include <algorithm>
#include <utility>

/// @file
/// Provides drake::MakeSortedPair and drake::SortedPair for storing two
/// values of a certain type in sorted order.

namespace drake {

/// This class is similar to the std::pair class. However, this class uses a
/// pair of homogeneous types (std::pair can use heterogeneous types) and sorts
/// the first and second values (the first value is lower). Thus the SortedPair
/// class is able to be used to generate keys (e.g., for std::map, etc.) from
/// pairs of objects.
///
/// The data are stored as `const` types to prevent user modification- which
/// could break the sorted ordering- while allowing the familiar `first`
/// and `second` member data accesses provided by std::pair.
template <class T>
struct SortedPair {
  typedef T type;

  const T first{};                 ///< The smaller of the two objects.
  const T second{};                ///< The larger of the two objects.

  /// The default constructor creates `first`and `second` using their
  /// respective default constructors.
  SortedPair() = default;
  SortedPair(const SortedPair& s) = default;
  SortedPair(SortedPair&& s) = default;

  /// Constructs a %SortedPair from two objects.
  SortedPair(const T& a, const T& b) : first(a), second(b) {
    if (first > second) {
      T* first_non_const = const_cast<T*>(&first);
      T* second_non_const = const_cast<T*>(&second);
      std::swap(*first_non_const, *second_non_const);
    }
  }

  /// Type-converting copy constructor.
  template <class U>
  SortedPair(const SortedPair<U>& p) : first(p.first), second(p.second) { }

  SortedPair& operator=(const SortedPair& p) {
    // This block of code is necessary since `first` and `second` are const
    // objects.
    T* first_non_const = const_cast<T*>(&first);
    T* second_non_const = const_cast<T*>(&second);
    *first_non_const = p.first;
    *second_non_const = p.second;
    return *this;
  }
};

/// Two pairs of the same type are equal iff their members are equal.
template <class T>
inline bool operator==(const SortedPair<T>& x, const SortedPair<T>& y) {
  return x.first == y.first && x.second == y.second;
}

// Compares two pairs in the following way: `x < y` is true iff when
// `x.first < y.first` or `x.first == y.first and x.second < y.second)`.
template <class T>
inline bool operator<(const SortedPair<T>& x, const SortedPair<T>& y) {
  return x.first < y.first || (x.first == y.first && x.second < y.second);
}

/// Determine whether two SortedPair objects are equal using `operator==`.
template <class T> inline bool operator!=(
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
/// @param x  The first object.
/// @param y  The second object.
/// @return A newly-constructed SortedPair object.
template <class T>
inline SortedPair<T>
MakeSortedPair(const T& x, const T& y) {
  return SortedPair<T>(x, y);
}

}  // namespace drake

namespace std {

/// Implements std::swap().
template <class T>
void swap(drake::SortedPair<T>& t, drake::SortedPair<T>& u) {
  std::swap(const_cast<T&>(t.first), const_cast<T&>(u.first));
  std::swap(const_cast<T&>(t.second), const_cast<T&>(u.second));
}

/// Implements std::hash().
template <class T>
struct hash<drake::SortedPair<T>> {
  std::size_t operator()(const drake::SortedPair<T>& s) const noexcept {
    const std::size_t h1 = std::hash<T>()(s.first);
    const std::size_t h2 = std::hash<T>()(s.second);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

}  // namespace std

