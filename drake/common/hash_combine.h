#pragma once

#include <cstddef>
#include <functional>
#include <iostream>
#include <map>
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

/// Computes a hash value of a given vector @p vec. It assumes that a given
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

/// Computes a hash value of a given std::map @p map. It assumes that a given
/// map is non-empty.
template <class T1, class T2>
inline size_t hash_combine(const std::map<T1, T2>& map) {
  DRAKE_ASSERT(!map.empty());
  auto it(map.cbegin());
  std::hash<T1> hasher1;
  size_t seed{hash_combine(hasher1(it->first), it->second)};
  while (it != map.cend()) {
    seed = hash_combine(seed, hash_combine(hasher1(it->first), it->second));
    ++it;
  }
  return seed;
}

}  // namespace drake
