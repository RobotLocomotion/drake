#pragma once

/// @file
/// This header provides a std::make_unique implementation if and only if the
/// compiler identifies as GCC with a version prior to 4.9.  This file is
/// included from a few basic headers within the drake/common subdirectory,
/// such that almost every file in Drake is exposed to it.  This allows us to
/// write code using the std::make_unique phrasing, but still support GCC 4.8.

#ifndef DRAKE_DOXYGEN_CXX
#if defined(__GNUG__) && !defined(__clang__)
#if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__) < 40900

#include <memory>

namespace std {
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}  // namespace std

#endif  // version check
#endif  // compiler check
#endif  // doxygen check
