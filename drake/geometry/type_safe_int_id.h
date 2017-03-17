#pragma once

#include <cstdint>
#include <functional>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/**
 A type-safe integer-based identifier class.

 This can be used to define identifier values that are uniquely typed and *not*
 interconvertible. The underlying representation of the identifier is a 64-bit
 int.

 The TypeSafeIntId can be used as a unique identifier or "handle" for other
 objects (decoupling the concept of a unique identifier and location in memory).
 Identifiers of the same type can be compared against each other for strict
 equality. Identifier values are immutable once created. The id provides an
 interface to the underlying value.

 There is no such thing as an "invalid" identifier. Operations which return an
 identifier, but may fail, should communicate this in the function interface. If
 an identifier exists, it should be considered valid.

 It is the designed intent of this class, that ids derived from this class can
 be passed and returned by value. Passing ids by const reference should be
 considered a misuse.

 To create an int-based identifier for class Foo, do the following:
 @code{.cpp}
 using FooId = TypeSafeIntId<class FooTag>;
 @endcode

 @tparam Tag              The name of the tag that uniquely segregates one
                          instantiation from another.
 */
template <class Tag>
class TypeSafeIntId {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypeSafeIntId)

  /** Instantiates an identifier from the underlying representation type. */
  explicit TypeSafeIntId(int64_t val) : value_(val) {}

  /** Extracts the underlying representation from the identifier. */
  int64_t value() const { return value_; }

  /** Compares one identifier with another of the same type for equality. */
  friend bool operator==(TypeSafeIntId a, TypeSafeIntId b) {
    return a.value_ == b.value_;
  }

  /** Compares one identifier with another of the same type for inequality.*/
  friend bool operator!=(TypeSafeIntId a, TypeSafeIntId b) {
    return a.value_ != b.value_;
  }

  /** Enables use of the identifier to serve as a key in STL containers. */
  friend struct std::hash<TypeSafeIntId<Tag>>;

 private:
  // The underlying value.
  int64_t value_;
};
}  // namespace geometry
}  // namespace drake

namespace std {
template <typename Tag>
struct hash<drake::geometry::TypeSafeIntId<Tag>> {
  size_t operator()(const drake::geometry::TypeSafeIntId<Tag>& id) const {
    return static_cast<size_t>(id.value_);//hash(id.value());
  }
};
}  // namespace std
