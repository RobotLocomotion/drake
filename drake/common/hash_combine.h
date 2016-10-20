#pragma once

#include <cstddef>
#include <functional>

namespace drake {

/// Combines two hash values into one. The following code is public domain
/// according to http://www.burtleburtle.net/bob/hash/doobs.html.
template <class T>
inline size_t hash_combine(size_t seed, const T& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}
}  // namespace drake
