#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

/// @defgroup hash_append hash_append generic hashing
/// @{
/// @ingroup cxx
/// @brief Drake uses the hash_append pattern as described by N3980.
///
/// For a full treatment of the hash_append pattern, refer to:
/// http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2014/n3980.html
///
/// <h3>Providing hash_append support within a class</h3>
///
/// Drake types may implement a `hash_append` function.
/// The function appends every hash-relevant member field into the hasher:
/// @code
/// class MyValue {
///  public:
///   ...
///   /// Implements the @ref hash_append concept.
///   template <class HashAlgorithm>
///   friend void hash_append(
///       HashAlgorithm& hasher, const MyValue& item) noexcept {
///     using drake::hash_append;
///     hash_append(hasher, item.my_data_);
///   }
///   ...
///  private:
///   std::string my_data_;
/// };
/// @endcode
///
/// Checklist for reviewing a `hash_append` implementation:
///
/// - The function cites `@ref hash_append` in its Doxygen comment.
/// - The function is marked `noexcept`.
///
/// <h3>Using hashable types</h3>
///
/// Types that implement this pattern may be used in unordered collections:
/// @code
/// std::unordered_set<MyValue, drake::DefaultHash> foo;
/// @endcode
///
/// Some Drake types may also choose to specialize `std::hash<MyValue>` to use
/// `DefaultHash`, so that the second template argument to `std::unordered_set`
/// can be omitted.  For example, Drake's `symbolic::Expression` header says:
/// @code
/// namespace std {
/// struct hash<drake::symbolic::Expression> : public drake::DefaultHash {};
/// }  // namespace std
/// @endcode
/// so that users are able to simply write:
/// @code
/// std::unordered_set<drake::symbolic::Expression> foo;
/// @endcode
///
/// @}

namespace drake {

/// Provides @ref hash_append for integral constants.
template <class HashAlgorithm, class T>
std::enable_if_t<std::is_integral<T>::value>
hash_append(
    HashAlgorithm& hasher, const T& item) noexcept {
  hasher(std::addressof(item), sizeof(item));
}

/// Provides @ref hash_append for enumerations.
template <class HashAlgorithm, class T>
std::enable_if_t<std::is_enum<T>::value>
hash_append(
    HashAlgorithm& hasher, const T& item) noexcept {
  hasher(std::addressof(item), sizeof(item));
}

/// Provides @ref hash_append for floating point values.
template <class HashAlgorithm, class T>
std::enable_if_t<std::is_floating_point<T>::value>
hash_append(
    HashAlgorithm& hasher, const T& item) noexcept {
  // Hashing a NaN makes no sense, since they cannot compare as equal.
  DRAKE_ASSERT(!std::isnan(item));
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
    HashAlgorithm& hasher,
    const std::basic_string<char, Traits, Allocator>& item) noexcept {
  using drake::hash_append;
  hasher(item.data(), item.size());
  // All collection types must send their size, after their contents.
  // See the #hash_append_vector anchor in N3980.
  hash_append(hasher, item.size());
}

/// Provides @ref hash_append for std::pair.
template <class HashAlgorithm, class T1, class T2>
void hash_append(
    HashAlgorithm& hasher, const std::pair<T1, T2>& item) noexcept {
  using drake::hash_append;
  hash_append(hasher, item.first);
  hash_append(hasher, item.second);
}

/// Provides @ref hash_append for std::optional.
///
/// Note that `std::hash<std::optional<T>>` provides the peculiar invariant
/// that the hash of an `optional` bearing a value `v` shall evaluate to the
/// same hash as that of the value `v` itself.  Hash operations implemented
/// with this `hash_append` do *not* provide that invariant.
template <class HashAlgorithm, class T>
void hash_append(
    HashAlgorithm& hasher, const std::optional<T>& item) noexcept {
  if (item) {
    hash_append(hasher, *item);
  }
  hash_append(hasher, item.has_value());
};

/// Provides @ref hash_append for a range, as given by two iterators.
template <class HashAlgorithm, class Iter>
void hash_append_range(
    // NOLINTNEXTLINE(runtime/references) Per hash_append convention.
    HashAlgorithm& hasher, Iter begin, Iter end) noexcept {
  using drake::hash_append;
  size_t count{0};
  for (Iter iter = begin; iter != end; ++iter, ++count) {
    hash_append(hasher, *iter);
  }
  // All collection types must send their size, after their contents.
  // See the #hash_append_vector anchor in N3980.
  hash_append(hasher, count);
}

/// Provides @ref hash_append for std::map.
///
/// Note that there is no `hash_append` overload for `std::unordered_map`, and
/// such an overload must never appear.  See n3980.html#unordered for details.
template <
  class HashAlgorithm,
  class T1,
  class T2,
  class Compare,
  class Allocator>
void hash_append(
    HashAlgorithm& hasher,
    const std::map<T1, T2, Compare, Allocator>& item) noexcept {
  return hash_append_range(hasher, item.begin(), item.end());
};

/// Provides @ref hash_append for std::set.
///
/// Note that there is no `hash_append` overload for `std::unordered_set`, and
/// such an overload must never appear.  See n3980.html#unordered for details.
template <
  class HashAlgorithm,
  class Key,
  class Compare,
  class Allocator>
void hash_append(
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

namespace internal {
/// The FNV1a hash algorithm, used for @ref hash_append.
/// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
class FNV1aHasher {
 public:
  using result_type = size_t;

  /// Feeds a block of memory into this hash.
  void operator()(const void* data, size_t length) noexcept {
    const uint8_t* const begin = static_cast<const uint8_t*>(data);
    const uint8_t* const end = begin + length;
    for (const uint8_t* iter = begin; iter < end; ++iter) {
      hash_ = (hash_ ^ *iter) * kFnvPrime;
    }
  }

  /// Feeds a single byte into this hash.
  constexpr void add_byte(uint8_t byte) noexcept {
    hash_ = (hash_ ^ byte) * kFnvPrime;
  }

  /// Returns the hash.
  explicit constexpr operator size_t() noexcept {
    return hash_;
  }

 private:
  static_assert(sizeof(result_type) == (64 / 8), "We require a 64-bit size_t");
  result_type hash_{0xcbf29ce484222325u};
  static constexpr size_t kFnvPrime = 1099511628211u;
};
}  // namespace internal

/// The default HashAlgorithm concept implementation across Drake.  This is
/// guaranteed to have a result_type of size_t to be compatible with std::hash.
using DefaultHasher = internal::FNV1aHasher;

/// The default hashing functor, akin to std::hash.
using DefaultHash = drake::uhash<DefaultHasher>;

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
  const Func func_;
};

}  // namespace drake
