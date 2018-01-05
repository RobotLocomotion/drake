#pragma once

#include <algorithm>

/// @file
/// Provides drake::make_SortedPair and drake::SortedPair for storing two
/// values of a certain type in sorted order.

namespace drake {

/// This class is similar to the std::pair class. However, this class uses a
/// pair of homogeneous types (std::pair can use heterogeneous types) and sorts
/// the first and second values (the first value is lower). Thus the SortedPair
/// class is able to be used to generate keys (e.g., for std::map, etc.) from
/// pairs of objects.
template <class T>
struct SortedPair {
  typedef T type;    

  const T first;                 ///< @c first is a copy of the first object
  const T second;                ///< @c second is a copy of the second object

  /// The default constructor creates @c first and @c second using their
  ///  respective default constructors. 
  SortedPair() : first(), second() { }

  /// Two objects may be passed to a @c pair constructor to be copied. 
  SortedPair(const T& a, const T& b) : first(a), second(b) 
  { 
    if (first > second) {
      T* first_non_const = const_cast<T*>(&first);
      T* second_non_const = const_cast<T*>(&second);
      std::swap(*first_non_const, *second_non_const);
    }
  }

  /// There is also a templated copy constructor for the @c pair class itself.
  template <class U>
  SortedPair(const SortedPair<U>& p) : first(p.first), second(p.second) { }

  /// Assignment operator. 
  SortedPair& operator=(const SortedPair& p) {
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

template <class T>
inline bool operator<(const SortedPair<T>& x, const SortedPair<T>& y) {
  return x.first < y.first || (!(y.first < x.first) && x.second < y.second); 
}

/// Uses @c operator== to find the result.
template <class T> inline bool operator!=(
    const SortedPair<T>& x, const SortedPair<T>& y) { 
  return !(x == y);
}

/// Uses @c operator< to find the result.
template <class T>
inline bool operator>(const SortedPair<T>& x, const SortedPair<T>& y) {
  return y < x;
}

/// Uses @c operator< to find the result.
template <class T>
inline bool operator<=(const SortedPair<T>& x, const SortedPair<T>& y) {
  return !(y < x);
}

/// Uses @c operator< to find the result.
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

