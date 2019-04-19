#pragma once

#include <functional>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/hash.h"

namespace drake {
namespace maliput {
namespace api {

/// %TypeSpecificIdentifier<T> represents an identifier specifically identifying
/// an entity of type `T`.
///
/// A new %TypeSpecificIdentifier is constructed from a non-empty string;
/// TypeSpecificIdentifiers constructed from equal strings are considered to
/// be equal.  There is currently no other semantic value attributed to the
/// contents of the string.
///
/// Construction from empty strings is not allowed; there is no notion of
/// an "unassigned" value for a %TypeSpecificIdentifier.  To represent a
/// possibly-unassigned %TypeSpecificIdentifier, use
/// drake::optional<TypeSpecificIdentifier<T>>.
///
/// %TypeSpecificIdentifier is EqualityComparable (and provides == and !=
/// operators), but it is not LessThanComparable; there is no particular
/// ordering ascribed to %TypeSpecificIdentifier instances.  However,
/// %TypeSpecificIdentifier does provide a strict weak ordering via a
/// specialization of std::less for use in ordered containers such as std::set
/// and std::map.  This ordering may change in future implementations of
/// %TypeSpecificIdentifier.
///
/// %TypeSpecificIdentifier also provides a specialization of std::hash to make
/// it easy to use with std::unordered_set and std::unordered_map.
template <typename T>
class TypeSpecificIdentifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypeSpecificIdentifier);

  /// The type whose instances are identified by this TypeSpecificIdentifier.
  typedef T identified_type;

  /// Constructs a %TypeSpecificIdentifier from the given `string`.
  ///
  /// @throws std::runtime_error if `string` is empty.
  explicit TypeSpecificIdentifier(std::string string)
      : string_(std::move(string)) {
    DRAKE_THROW_UNLESS(!string_.empty());
  }

  /// Returns the string representation of the %TypeSpecificIdentifier.
  const std::string& string() const { return string_; }

  /// Tests for equality with another %TypeSpecificIdentifier.
  bool operator==(const TypeSpecificIdentifier<T>& rhs) const {
    return string_ == rhs.string_;
  }

  /// Tests for inequality with another %TypeSpecificIdentifier, specifically
  /// returning the opposite of operator==().
  bool operator!=(const TypeSpecificIdentifier<T>& rhs) const {
    return !(*this == rhs);
  }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher, const TypeSpecificIdentifier& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.string_);
  }

 private:
  std::string string_;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake


namespace std {

/// Specialization of std::hash for TypeSpecificIdentifier<T>.
template <typename T>
struct hash<drake::maliput::api::TypeSpecificIdentifier<T>>
    : public drake::DefaultHash {};

/// Specialization of std::less for TypeSpecificIdentifier<T> providing a
/// strict ordering over TypeSpecificIdentifier<T> suitable for use with ordered
/// containers.
template <typename T>
struct less<drake::maliput::api::TypeSpecificIdentifier<T>> {
  bool operator()(const drake::maliput::api::TypeSpecificIdentifier<T>& lhs,
                  const drake::maliput::api::TypeSpecificIdentifier<T>& rhs)
      const {
    return lhs.string() < rhs.string();
  }
};

}  // namespace std
