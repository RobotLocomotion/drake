#pragma once

#include <cstddef>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_throw.h"

/// @defgroup hash_append hash_append generic hashing
/// @{
/// @brief Drake has hashing, yay!
///
/// The hash_append function concept:
/// - TBD
///
/// The HashAlgorithm class concept:
/// - TBD
///
/// @ingroup cxx
/// @}

namespace drake {

/// Provides @ref hash_append for integral constants.
template <class HashAlgorithm, class T>
std::enable_if_t<std::is_integral<T>::value>
hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher, const T& item) noexcept {
  hasher(std::addressof(item), sizeof(item));
}

/// Provides @ref hash_append for enumerations.
template <class HashAlgorithm, class T>
std::enable_if_t<std::is_enum<T>::value>
hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher, const T& item) noexcept {
  hasher(std::addressof(item), sizeof(item));
}

/// Provides @ref hash_append for floating point values.
template <class HashAlgorithm, class T>
std::enable_if_t<std::is_floating_point<T>::value>
hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher, const T& item) noexcept {
  // +0.0 and -0.0 are equal, so must hash identically.
  if (item == 0.0) {
    const T zero{0.0};
    hasher(std::addressof(zero), sizeof(zero));
  } else {
    hasher(std::addressof(item), sizeof(item));
  }
}

/// Provides @ref hash_append for std::string.
/// (Technically, any string based on `CharT = char`.)
template <class HashAlgorithm, class Traits, class Allocator>
void hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher,
    const std::basic_string<char, Traits, Allocator>& item) noexcept {
  using drake::hash_append;
  hasher(item.data(), item.size());
  hash_append(hasher, item.size());
}

/// Provides @ref hash_append for std::pair.
template <class HashAlgorithm, class T1, class T2>
void hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher, const std::pair<T1, T2>& item) noexcept {
  using drake::hash_append;
  hash_append(hasher, item.first);
  hash_append(hasher, item.second);
}

/// Provides @ref hash_append for a range, as given by two iterators.
template <class HashAlgorithm, class Iter>
void hash_append_range(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher, Iter begin, Iter end) noexcept {
  using drake::hash_append;
  size_t count{0};
  for (Iter iter = begin; iter != end; ++iter, ++count) {
    hash_append(hasher, *iter);
  }
  hash_append(hasher, count);
}

/// Provides @ref hash_append for std::map.
template <
  class HashAlgorithm,
  class T1,
  class T2,
  class Compare,
  class Allocator>
void hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher,
    const std::map<T1, T2, Compare, Allocator>& item) noexcept {
  return hash_append_range(hasher, item.begin(), item.end());
};

/// Provides @ref hash_append for std::set.
template <
  class HashAlgorithm,
  class Key,
  class Compare,
  class Allocator>
void hash_append(
    // NOLINTNEXTLINE(runtime/references)
    HashAlgorithm& hasher,
    const std::set<Key, Compare, Allocator>& item) noexcept {
  return hash_append_range(hasher, item.begin(), item.end());
};

/// A hashing functor, somewhat like `std::hash`.  Given an item of type @p T,
/// applies @ref hash_append to it, directing the bytes to append into the
/// given @p HashAlgorithm, and then finally returning the algorithm's result.
template <class HashAlgorithm>
struct uhash {
  using result_type = typename HashAlgorithm::result_type;

  template <class T>
  result_type operator()(const T& item) const noexcept {
    HashAlgorithm hasher;
    using drake::hash_append;
    hash_append(hasher, item);
    return static_cast<result_type>(hasher);
  }
};

namespace detail {
/// The FNV1a hash algorithm, used for @ref hash_append.
/// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
class FNV1aHasher {
 public:
  using result_type = size_t;

  void operator()(const void* data, size_t length) noexcept {
    const uint8_t* const begin = static_cast<const uint8_t*>(data);
    const uint8_t* const end = begin + length;
    for (const uint8_t* iter = begin; iter < end; ++iter) {
      hash_ = (hash_ ^ *iter) * 1099511628211u;
    }
  }

  explicit operator size_t() noexcept {
    return hash_;
  }

 private:
  static_assert(sizeof(result_type) == (64 / 8), "We require a 64-bit size_t");
  result_type hash_{0xcbf29ce484222325u};
};
}  // namespace detail

/// The default HashAlgorithm concept implementation across Drake.  This is
/// guaranteed to have a result_type of size_t to be compatible with std::hash.
using DefaultHasher = detail::FNV1aHasher;

/// An adapter that forwards the HashAlgorithm::operator(data, length) function
/// concept into a runtime-provided std::function of the same signature.  This
/// is useful for passing a concrete HashAlgorithm implementation through into
/// non-templated code, such as with an Impl or Cell pattern.
struct DelegatingHasher {
  /// A std::function whose signature matches HashAlgorithm::operator().
  using Func = std::function<void(const void*, size_t)>;

  /// Create a delegating hasher that calls the given @p func.
  explicit DelegatingHasher(Func func) : func_(std::move(func)) {
    // In order for operator() to be noexcept, it must have a non-empty func_.
    DRAKE_THROW_UNLESS(static_cast<bool>(func_));
  }

  /// Append [data, data + length) bytes into the wrapped algorithm.
  void operator()(const void* data, size_t length) noexcept {
    func_(data, length);
  }

 private:
  Func func_;
};

}  // namespace drake
