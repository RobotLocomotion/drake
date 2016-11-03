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

/** has_hash_value is true for the types for which hash_value function is
 *  provided. This type trait is used in hash_combine(size_t, const T& v) to
 *  choose a proper hash function to compute the hash value of v.
 */
template <typename T>
struct has_hash_value {
  static constexpr bool value = false;
};

template <class T1, class T2>
struct has_hash_value<std::pair<T1, T2>> {
  static constexpr bool value = true;
};

template <class T>
struct has_hash_value<std::vector<T>> {
  static constexpr bool value = true;
};

template <class T>
struct has_hash_value<std::set<T>> {
  static constexpr bool value = true;
};

template <class T1, class T2>
struct has_hash_value<std::map<T1, T2>> {
  static constexpr bool value = true;
};

/** Combines two hash values into one. The following code is public domain
 *  according to http://www.burtleburtle.net/bob/hash/doobs.html. */
inline size_t hash_combine_impl(size_t seed1, const size_t seed2) {
  seed1 ^= seed2 + 0x9e3779b9 + (seed1 << 6) + (seed1 >> 2);
  return seed1;
}

/** Combines a given hash value @p seed and a hash of parameter @p v. It uses
 * hash_value function defined for @v. */
template <class T>
size_t hash_combine(
    size_t seed,
    const typename std::enable_if<has_hash_value<T>::value, T>::type& v);

/** Combines a given hash value @p seed and a hash of parameter @p v. It uses
 * std::hash to compute the hash of @p v. */
template <class T>
size_t hash_combine(size_t seed, const T& v);

/** Computes the combined hash value of the elements of an iterator range. */
template <typename It>
std::size_t hash_range(It first, It last) {
  size_t seed = 0;
  for (; first != last; ++first) {
    seed = hash_combine<typename It::value_type>(seed, *first);
  }
  return seed;
}

/** Computes the hash value of a pair @p p. */
template <class T1, class T2>
std::size_t hash_value(const std::pair<T1, T2>& p) {
  size_t seed = 0;
  seed = hash_combine(seed, p.first);
  seed = hash_combine(seed, p.second);
  return seed;
}

/** Computes the hash value of a vector @p vec. */
template <class T>
size_t hash_value(const std::vector<T>& vec) {
  return hash_range(vec.begin(), vec.end());
}

/** Computes the hash value of a set @p s. */
template <class T>
size_t hash_value(const std::set<T>& s) {
  return hash_range(s.begin(), s.end());
}

/** Computes the hash value of a map @p map. */
template <class T1, class T2>
size_t hash_value(const std::map<T1, T2>& map) {
  return hash_range(map.begin(), map.end());
}

template <class T>
size_t hash_combine(
    size_t seed,
    const typename std::enable_if<has_hash_value<T>::value, T>::type& v) {
  return hash_combine_impl(seed, hash_value(v));
}

template <class T>
size_t hash_combine(size_t seed, const T& v) {
  return hash_combine_impl(seed, std::hash<T>{}(v));
}

}  // namespace drake
