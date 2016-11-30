#pragma once

#include <cstddef>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {

/** Combines a given hash value @p seed and a hash of parameter @p v. */
template <class T>
size_t hash_combine(size_t seed, const T& v);

template <class T, class... Rest>
size_t hash_combine(size_t seed, const T& v, Rest... rest) {
  return hash_combine(hash_combine(seed, v), rest...);
}

/** Computes the combined hash value of the elements of an iterator range. */
template <typename It>
size_t hash_range(It first, It last) {
  size_t seed{};
  for (; first != last; ++first) {
    seed = hash_combine<typename It::value_type>(seed, *first);
  }
  return seed;
}

/** Computes the hash value of @p v using std::hash. */
template <class T>
struct hash_value {
  size_t operator()(const T& v) { return std::hash<T>{}(v); }
};

/** Computes the hash value of a pair @p p. */
template <class T1, class T2>
struct hash_value<std::pair<T1, T2>> {
  size_t operator()(const std::pair<T1, T2>& p) {
    return hash_combine(0, p.first, p.second);
  }
};

/** Computes the hash value of a vector @p vec. */
template <class T>
struct hash_value<std::vector<T>> {
  size_t operator()(const std::vector<T>& vec) {
    return hash_range(vec.begin(), vec.end());
  }
};

/** Computes the hash value of a set @p s. */
template <class T>
struct hash_value<std::set<T>> {
  size_t operator()(const std::set<T>& s) {
    return hash_range(s.begin(), s.end());
  }
};

/** Computes the hash value of a map @p map. */
template <class T1, class T2>
struct hash_value<std::map<T1, T2>> {
  size_t operator()(const std::map<T1, T2>& map) {
    return hash_range(map.begin(), map.end());
  }
};

/** Combines two hash values into one. The following code is public domain
 *  according to http://www.burtleburtle.net/bob/hash/doobs.html. */
template <class T>
inline size_t hash_combine(size_t seed, const T& v) {
  seed ^= hash_value<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}
}  // namespace drake
