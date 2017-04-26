#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace geometry {

/**
 A simple identifier class.

 This class serves as an upgrade to the standard practice of passing `int`s
 around as unique identifiers (or, as in this case, `int64_t`s). In the common
 practice, a method that takes identifiers to different types of objects would
 have an interface like:

 @code
 void foo(int64_t bar_id, int64_t thing_id);
 @endcode

 It is possible for a programmer to accidentally switch the two ids in an
 invocation. This mistake would still be _syntactically_ correct; it will
 successfully compile but  lead to inscrutable run-time errors. This identifier
 class provides the same speed and efficiency of passing `int64_t`s, but
 enforces unique types and limits the valid operations, providing compile-time
 checking. The function would now look like:

 @code
 void foo(BarId bar_id, ThingId thing_id)
 @endcode

 and the compiler will catch instances where the order is reversed.

 The identifier is a _stripped down_ 64-bit int. Each uniquely declared
 identifier type has the following properties:

   - The identifier's default constructor produces _invalid_ identifiers.
   - Valid identifiers must be constructed via the copy constructor or through
     Identifier::get_new_id().
   - The identifier is immutable.
   - The identifier can only be tested for equality/inequality with other
     identifiers of the _same_ type.
   - Identifiers of different types are _not_ interconvertible.
   - The identifier can be queried for its underlying `int64_t` value.
   - The identifier can be written to an output stream; its underlying `int64_t`
     value gets written.
   - Identifiers are not guaranteed to possess _meaningful_ ordering. I.e.,
     identifiers for two objects created sequentially may not have sequential
     identifier values.
   - Identifiers can only be generated from the static method get_new_id().

 While there _is_ the concept of an invalid identifier, this only exists to
 facilitate use with STL containers that require default constructors. Using an
 invalid identifier in any operation is considered an error. In Debug build,
 attempts to compare, get the value of, hash, or write an invalid identifier to
 a stream will cause program failure.

 Functions that query for identifiers should not return invalid identifiers. We
 prefer the practice of returning std::optional<Identifier> instead.

 It is the designed intent of this class, that ids derived from this class can
 be passed and returned by value. Passing ids by const reference should be
 considered a misuse.

 The following alias will create a unique identifier type for class `Foo`:
 @code{.cpp}
 using FooId = Identifier<class FooTag>;
 @endcode

 __Examples of valid and invalid operations__

 The Identifier guarantees that id instances of different types can't be
 compared or combined. Efforts to do so will cause a compile-time failure.
 For example:

 @code
    using AId = Identifier<class ATag>;
    using BId = Identifier<class BTag>;
    AId a1;                              // Compiler error. There is no default constructor.
    AId a2 = AId::get_new_id();          // Ok.
    AId a3(a2);                          // Ok.
    AId a4 = AId::get_new_id();          // Ok.
    BId b = BId::get_new_id();           // Ok.
    if ( a2 == 1 ) { ... }               // Compiler error.
    if ( a2 == a4 ) { ... }              // Ok, evaluates to false.
    if ( a2 == a3 ) { ... }              // Ok, evaluates to true.
    if ( a2 == b ) { ... }               // Compiler error.
    a4 = a2;                             // Ok.
    a3 = 7;                              // Compiler error.
 @endcode

 __Type-safe Index vs Identifier__

 In principle, the *identifier* is related to the TypeSafeIndex. In
 some sense, both are "type-safe" `int`s. They differ in their semantics. We can
 consider `ints`, indexes, and identifiers as a list of `int` types with
 _decreasing_ functionality.

   - The int, obviously, has the full range of C++ ints.
   - The TypeSafeIndex can be implicitly cast *to* an int, but there are a
     limited number of operations _on_ the index that produce other instances
     of the index (e.g., increment, in-place addition, etc.) They can be
     compared with `int` and other indexes of the same type. This behavior
     arises from the intention of having them serve as an _index_ in an
     ordered set (e.g., `std::vector`).
   - The Identifier is the most restricted. They exist solely to serve as a
     unique identifier. They are immutable when created. Very few operations
     exist on them (comparison for _equality_ with other identifiers of the same
     type, hashing, writing to output stream). These *cannot* be used as
     indices.

 Ultimately, indexes _can_ serve as identifiers (within the scope of the object
 they index into). Although, their mutability could make this a dangerous
 practice for a public API. Identifiers are more general in that they don't
 reflect an object's position in memory (hence the inability to transform to or
 compare with an `int`). This decouples details of implementation from the idea
 of the object. Combined with its immutability, it would serve well as a element
 of a public API.

 @sa TypeSafeIndex

 @tparam Tag              The name of the tag that uniquely segregates one
                          instantiation from another.
 */
template <class Tag>
class Identifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Identifier)

  /** Default constructor; the result is an _invalid_ identifier. This only
   exists to satisfy demands of working with various container classes. */
  Identifier() : value_(0) {}

  /** Extracts the underlying representation from the identifier. This is
   considered invalid for invalid ids and is strictly enforced in Debug builds.
   */
  int64_t get_value() const {
    DRAKE_ASSERT(is_valid());
    return value_; }

  /** Reports if the id is valid. */
  bool is_valid() const { return value_ > 0; }

  /** Compares one identifier with another of the same type for equality. This
   is considered invalid for invalid ids and is strictly enforced in Debug
   builds. */
  bool operator==(Identifier other) const {
    DRAKE_ASSERT(is_valid() && other.is_valid());
    return value_ == other.value_;
  }

  /** Compares one identifier with another of the same type for inequality. This
   is considered invalid for invalid ids and is strictly enforced in Debug
   builds. */
  bool operator!=(Identifier other) const {
    DRAKE_ASSERT(is_valid() && other.is_valid());
    return value_ != other.value_ && is_valid() && other.is_valid();
  }

  /** Generates a new identifier for this id type. This new identifier will be
   different from all previous identifiers created. This method does _not_
   make any guarantees about the values of ids from successive invocations.
   This method is guaranteed to be thread safe.
   */
  static Identifier get_new_id() {
    // Note that id 0 is reserved for uninitialized variable which is created
    // by the default constructor. As a result, we have an invariant that
    // get_new_id() > 0.
    static never_destroyed<std::atomic<int64_t>> next_index(1);
    return Identifier(next_index.access()++);
  }

 private:
  // Instantiates an identifier from the underlying representation type.
  explicit Identifier(int64_t val) : value_(val) {}

  // The underlying value.
  int64_t value_;
};

/** Streaming output operator.   This is considered invalid for invalid ids and
 is strictly enforced in Debug builds.
 @relates Identifier
 */
template <typename Tag>
inline std::ostream& operator<<(std::ostream& out,
                                const Identifier<Tag>& id) {
  out << id.get_value();
  return out;
}

/** Enables use of identifiers with to_string. It requires ADL to work. So,
 it should be invoked as: `to_string(id);` and should be preceded by
 `using std::to_string`.*/
template <typename Tag> inline
std::string to_string(const drake::geometry::Identifier<Tag>& id) {
  return std::to_string(id.get_value());
}

}  // namespace geometry

}  // namespace drake

namespace std {
/** Enables use of the identifier to serve as a key in STL containers.
 @relates Identifier
 */
template <typename Tag>
struct hash<drake::geometry::Identifier<Tag>> {
  size_t operator()(const drake::geometry::Identifier<Tag>& id) const {
    return hash<int64_t>()(id.get_value());
  }
};

}  // namespace std
