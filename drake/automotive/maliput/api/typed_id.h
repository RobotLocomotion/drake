#pragma once

#include <functional>
#include <ostream>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {

/// TypeidId<T> represents an identifier specifically identifying an entity
/// of type `T`.
///
/// A new TypedId is constructed from a non-empty string; TypedId's constructed
/// from equal strings are considered to be equal.  There is currently no other
/// semantic value attributed to the contents of the string.
///
/// Construction from empty strings are not allowed; there is no notion of an
/// "unassigned" value for a TypedId.  To represent a possibly-unassigned
/// TypedId, use optional<TypedId<T>>.
///
/// TypedId is EqualityComparable (and provides == and != operators), but it is
/// not LessThanComparable; there is no particular ordering ascribed to TypedId
/// instances.  However, TypedId does provide a strict weak ordering via a
/// specialization of std::less for use in ordered containers such as std::set
/// and std::map.  This ordering may change in future implementations of
/// TypedId.
///
/// TypedId also provides a specialization of std::hash to make it easy to use
/// with std::unordered_set and std::unordered_map.
template <typename T>
class TypedId {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypedId);

  /// The type whose instances are identified by this TypedId.
  typedef T identified_type;

  /// Constructs a TypedId from the given `string`.
  ///
  /// @throws std::runtime_error if `string` is empty.
  explicit TypedId(const std::string& string) : string_(string) {
    DRAKE_THROW_UNLESS(!string.empty());
  }

  /// Returns the string representation of the TypedId.
  const std::string& string() const { return string_; }

  /// Tests for equality with another TypedId.
  bool operator==(const TypedId<T>& rhs) const {
    return string_ == rhs.string_;
  }

  /// Tests for inequality with another TypedId, specifically returning
  /// the opposite of operator==().
  bool operator!=(const TypedId<T>& rhs) const {
    return !(*this == rhs);
  }

 private:
  std::string string_;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake


namespace std {

/// Specialization of std::hash for TypedId<T>
template <typename T>
struct hash<drake::maliput::api::TypedId<T>> {
  typedef std::size_t result_type;
  typedef drake::maliput::api::TypedId<T> argument_type;

  result_type operator()(const argument_type& id) const {
    return hash<string>{}(id.string());
  }
};

/// Specialization of std::less for TypedId<T> providing a strict weak ordering
/// over TypedId<T> suitable for use with ordered containers.
template <typename T>
struct less<drake::maliput::api::TypedId<T>> {
  typedef std::size_t result_type;
  typedef drake::maliput::api::TypedId<T> first_argument_type;
  typedef drake::maliput::api::TypedId<T> second_argument_type;

  result_type operator()(const first_argument_type& lhs,
                         const second_argument_type& rhs) const {
    return lhs.string() < rhs.string();
  }
};

}  // namespace std
