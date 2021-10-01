#pragma once

#include <cstdint>
#include <ostream>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/hash.h"

namespace drake {

namespace internal {
int64_t get_new_identifier();
}  // namespace internal

/**
 A simple identifier class.

 @note This is *purposely* a separate class from @ref TypeSafeIndex. For more
 explanatation, see @ref TypeSafeIndexVsIndentifier "this section".

 This class serves as an upgrade to the standard practice of passing `int`s
 around as unique identifiers (or, as in this case, `int64_t`s). In the common
 practice, a method that takes identifiers to different types of objects would
 have an interface like:

 @code
 void foo(int64_t bar_id, int64_t thing_id);
 @endcode

 It is possible for a programmer to accidentally switch the two ids in an
 invocation. This mistake would still be _syntactically_ correct; it will
 successfully compile but lead to inscrutable run-time errors. This identifier
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
 invalid identifier is generally considered to be an error. In Debug build,
 attempts to compare, get the value of, or write an invalid identifier to a
 stream will throw an exception.

 Functions that query for identifiers should not return invalid identifiers. We
 prefer the practice of returning std::optional<Identifier> instead.

 It is the designed intent of this class, that ids derived from this class can
 be passed and returned by value. (Drake's typical calling convention requires
 passing input arguments by const reference, or by value when moved from. That
 convention does not apply to this class.)

 The following alias will create a unique identifier type for class `Foo`:
 @code{.cpp}
 using FooId = Identifier<class FooTag>;
 @endcode

 __Examples of valid and invalid operations__

 The %Identifier guarantees that id instances of different types can't be
 compared or combined. Efforts to do so will cause a compile-time failure.
 For example:

 @code
    using AId = Identifier<class ATag>;
    using BId = Identifier<class BTag>;
    AId a1;                              // Compiler error; there is no
                                         //   default constructor.
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

 @anchor TypeSafeIndexVsIndentifier
 __TypeSafeIndex vs Identifier__

 In principle, the *identifier* is related to the @ref TypeSafeIndex. In
 some sense, both are "type-safe" `int`s. They differ in their semantics. We can
 consider `ints`, indices, and identifiers as a list of `int` types with
 _decreasing_ functionality.

   - The int, obviously, has the full range of C++ ints.
   - The @ref TypeSafeIndex can be implicitly cast *to* an int, but there are a
     limited number of operations _on_ the index that produce other instances
     of the index (e.g., increment, in-place addition, etc.) They can be
     compared with `int` and other indices of the same type. This behavior
     arises from the intention of having them serve as an _index_ in an
     ordered set (e.g., `std::vector`).
   - The %Identifier is the most restricted. They exist solely to serve as a
     unique identifier. They are immutable when created. Very few operations
     exist on them (comparison for _equality_ with other identifiers of the same
     type, hashing, writing to output stream). These *cannot* be used as
     indices.

 Ultimately, indices _can_ serve as identifiers (within the scope of the object
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
    if (kDrakeAssertIsArmed) {
      DRAKE_THROW_UNLESS(this->is_valid());
    }
    return value_;
  }

  /** Reports if the id is valid. */
  bool is_valid() const { return value_ > 0; }

  /** Compares one identifier with another of the same type for equality. This
   is considered invalid for invalid ids and is strictly enforced in Debug
   builds. */
  bool operator==(Identifier other) const {
    return this->get_value() == other.get_value();
  }

  /** Compares one identifier with another of the same type for inequality. This
   is considered invalid for invalid ids and is strictly enforced in Debug
   builds. */
  bool operator!=(Identifier other) const {
    return this->get_value() != other.get_value();
  }

  /** Compare two identifiers in order to define a total ordering among
   identifiers. This makes identifiers compatible with data structures which
   require total ordering (e.g., std::set).  */
  bool operator<(Identifier other) const {
    return this->get_value() < other.get_value();
  }

  /** Generates a new identifier for this id type. This new identifier will be
   different from all previous identifiers created. This method does _not_
   make any guarantees about the values of ids from successive invocations.
   This method is guaranteed to be thread safe.  */
  static Identifier get_new_id() {
    return Identifier(internal::get_new_identifier());
  }

  /** Implements the @ref hash_append concept. And invalid id will successfully
   hash (in order to satisfy STL requirements), and it is up to the user to
   confirm it is valid before using it as a key (or other hashing application).
   */
  template <typename HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const Identifier& i) noexcept {
    using drake::hash_append;
    hash_append(hasher, i.value_);
  }

  /** (Internal use only) Compares this possibly-invalid Identifier with one
  that is known to be valid and returns `false` if they don't match. It is an
  error if `valid_id` is not actually valid, and that is strictly enforced in
  Debug builds. However, it is not an error if `this` id is invalid; that
  results in a `false` return. This method can be faster than testing
  separately for validity and equality. */
  bool is_same_as_valid_id(Identifier valid_id) const {
    return value_ == valid_id.get_value();
  }

 protected:
  // Instantiates an identifier from the underlying representation type.
  explicit Identifier(int64_t val) : value_(val) {}

 private:
  // The underlying value.
  int64_t value_{};
};

/** Streaming output operator.   This is considered invalid for invalid ids and
 is strictly enforced in Debug builds.
 @relates Identifier
 */
template <typename Tag>
std::ostream& operator<<(std::ostream& out, const Identifier<Tag>& id) {
  out << id.get_value();
  return out;
}

/** Enables use of identifiers with to_string. It requires ADL to work. So,
 it should be invoked as: `to_string(id);` and should be preceded by
 `using std::to_string`.*/
template <typename Tag>
std::string to_string(const drake::Identifier<Tag>& id) {
  return std::to_string(id.get_value());
}

}  // namespace drake

namespace std {

/** Enables use of the identifier to serve as a key in STL containers.
 @relates Identifier
 */
template <typename Tag>
struct hash<drake::Identifier<Tag>> : public drake::DefaultHash {};

}  // namespace std
