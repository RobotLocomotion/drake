#pragma once

#include <functional>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

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

 private:
  std::string string_;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake


namespace std {

/// Specialization of std::hash for TypeSpecificIdentifier<T>
template <typename T>
struct hash<drake::maliput::api::TypeSpecificIdentifier<T>> {
  typedef std::size_t result_type;
  typedef drake::maliput::api::TypeSpecificIdentifier<T> argument_type;

  result_type operator()(const argument_type& id) const {
    // NB: Per the GSG, our current style guide strictly prohibits
    // creating new specializations of std::hash on the grounds that
    // it is in general difficult to do that correctly.  However,
    // since this implementation is merely a wrapper around
    // std::string with stricter type checking and since it merely
    // invokes a C++ standard hash approved by the style guide, it has
    // been granted an exception.  If this implementation changes, the
    // exception must be reevaluated.  Conversely, if the (arguably
    // maladaptive) prohibition is removed from our style guide, this
    // notice can go away.
    return hash<string>{}(id.string());
  }
};

/// Specialization of std::less for TypeSpecificIdentifier<T> providing a
/// strict weak ordering over TypeSpecificIdentifier<T> suitable for use with
/// ordered containers.
template <typename T>
struct less<drake::maliput::api::TypeSpecificIdentifier<T>> {
  typedef std::size_t result_type;
  typedef drake::maliput::api::TypeSpecificIdentifier<T> first_argument_type;
  typedef drake::maliput::api::TypeSpecificIdentifier<T> second_argument_type;

  result_type operator()(const first_argument_type& lhs,
                         const second_argument_type& rhs) const {
    return lhs.string() < rhs.string();
  }
};

}  // namespace std
