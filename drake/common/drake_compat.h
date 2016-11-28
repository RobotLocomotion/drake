#pragma once

/// @file
/// This header provides a std::make_unique implementation to be used when the
/// compiler does not support C++14, or is not in C++14 mode.  This file is
/// included from a few basic headers within the drake/common subdirectory,
/// such that almost every file in Drake is exposed to it.  This allows us to
/// write code using the std::make_unique phrasing, but still support GCC 4.8.

#ifndef DRAKE_DOXYGEN_CXX
#if __cplusplus <= 201103L  // If C++11 or earlier, we need our own make_unique.

#include <memory>

namespace std {
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}  // namespace std

#endif  // version check
#endif  // doxygen check
