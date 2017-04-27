#pragma once

/// @file
/// This header provides a std::make_unique implementation to be used when the
/// compiler does not support C++14, or is not in C++14 mode.  This file is
/// included from a few basic headers within the drake/common subdirectory,
/// such that almost every file in Drake is exposed to it.

#ifndef DRAKE_DOXYGEN_CXX
#if __cplusplus <= 201103L  // If C++11 or earlier, we need our own helpers.

#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

namespace std {

/// This allows us to write code using the std::make_unique phrasing, but still
/// support GCC 4.8.
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template<bool B, class T = void>
using enable_if_t = typename enable_if<B, T>::type;


/// This implements the 4-iterator `equal` overload available in c++14, in a
/// way compatible with GCC 4.8.
template<class InputIt1, class InputIt2, class BinaryPredicate>
bool equal(InputIt1 first1, InputIt1 last1,
           InputIt2 first2, InputIt2 last2,
           BinaryPredicate p) {
  if (std::distance(first1, last1) != std::distance(last2, first2)) {
    return false;  // Lengths differ.
  }
  return std::equal(first1, last1, first2, p);
}


}  // namespace std

#endif  // version check
#endif  // doxygen check

