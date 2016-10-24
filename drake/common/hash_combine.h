#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {

/// Combines two hash values into one. The following code is public domain
/// according to http://www.burtleburtle.net/bob/hash/doobs.html.
template <class T>
inline size_t hash_combine(size_t seed, const T& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

/// Computes a hash value of a given vector \p vec. It assumes that a given
/// vector is non-empty.
template <class T>
inline size_t hash_combine(const std::vector<T>& vec) {
  DRAKE_ASSERT(!vec.empty());
  auto it(vec.cbegin());
  std::hash<T> hasher;
  size_t seed{hasher(*it)};
  while (it != vec.cend()) {
    seed = hash_combine(seed, *it);
    ++it;
  }
  return seed;
}
}  // namespace drake
