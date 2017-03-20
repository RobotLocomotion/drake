#pragma once

#include <cstdint>
#include <functional>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/**
 A type-safe integer-based identifier class.

 This class serves as an upgrade to the standard practice of passing `int`s
 around as unique identifiers. In the common practice, a method that takes
 identifiers to different types of objects would have an interface like:

 @code
 void foo(int bar_id, int thing_id);
 @endcode

 It is possible for a programmer to accidentally switch the two ids in an
 invocation. This mistake would still be _syntactically_ correct; it will
 successfully compile but  lead to inscrutable run-time errors. The type-safe
 identifier provides the same speed and efficiency of passing `int`s, but
 provides compile-time checking. The function would now look like:

 @code
 void foo(BarId bar_id, ThingId thing_id)
 @endcode

 and the compiler will catch instances where the order is reversed.

 The type-safe identifier is a _stripped down_ 64-bit int. Each uniquely
 declared identifier type has the following properties:

   - The identifier has no public constructors (see note below).
   - The identifier is immutable.
   - The identifier can only be tested for equality/inequality with other
     identifiers of the _same_ type.
   - Identifiers of different types are _not_ interconvertible.
   - The identifier can be queried for its underlying `int` value.
   - The identifier can be written to an output stream. Its underlying `int`
     value gets written.
   - Identifiers are not guaranteed to possess _meaningful_ ordering. I.e.,
     identifiers for two objects created sequentially may not have sequential
     identifier values.
   - Identifiers can only be generated from the static method get_new_id().

 There is no such thing as an "invalid" identifier; there is no sentinel
 value which indicates uninitialized or undefined. Operations which return
 an identifier, but can fail, should communicate this in the function interface
 (e.g. through a std::optional<IdType> return value). If an identifier
 exists, it should be considered valid.

 It is the designed intent of this class, that ids derived from this class can
 be passed and returned by value. Passing ids by const reference should be
 considered a misuse.

 The following alias will create a unique identifier type for class `Foo`:
 @code{.cpp}
 using FooId = TypeSafeIntId<Foo>;
 @endcode

 __Examples of valid and invalid operations__

 The TypeSafeIntId guarantees that id instances of different types can't be
 compared or combined. Efforts to do so will cause a compile-time failure.
 For example:

 @code
    using AId = TypeSafeIntId<class A>;
    using BId = TypeSafeIntId<class B>;
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

 __Working in the absence of public constructors__

 As noted, the TypeSafeIntId has no public constructor. This has several
 implications:

   1. Working with STL containers. Generally, the lack of a default constructor,
      is not a problem. Using `emplace_back()` and methods of this type work
      fine. However, you can't use methods that require a default constructor
      (e.g., `std::vector::resize()`). This will produce a compilation error.
   2. Member of a class. With no default constructor and no public constructor
      at all, ids _can_ be included as members, but they _can't_ be initialized
      at declaration. The only way to initialize them is in the constructors
      initialization list. This supports the idea that if an identifier exists
      it is valid.  The class which owns the identifier must responsible for
      initializing it (either from the factory `get_new_id()` method or via
      the copy constructor on a supplied identifier.

 __Type-safe Index vs Identifier__

 In principle, the type-safe *identifier* is related to the TypeSafeIndex. In
 some sense, both are "type-safe `int`s". They differ in their semantics. We can
 consider `ints`, indexes, and identifiers as a list of `int` types with
 _decreasing_ functionality.

   - The int, obviously, has the full range of C++ ints.
   - The TypeSafeIndex can be implicitly cast *to* an int, but there are a
     limited number of operations _on_ the index that produces other instances
     of the index (e.g., increment, in-place addition, etc.) They can be
     compared with `int` and other indexes of the same type. This behavior
     arises from the intention of having them serve as an _index_ in an
     ordered set (e.g., `std::vector`.)
   - The TypeSafeIntId is the most restricted. They exist solely to serve as a
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

 @tparam Tag              The name of the tag that uniquely segregates one
                          instantiation from another.
 */
template <class Tag>
class TypeSafeIntId {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypeSafeIntId)

  /** Extracts the underlying representation from the identifier. */
  int64_t get_value() const { return value_; }

  /** Compares one identifier with another of the same type for equality. */
  bool operator==(TypeSafeIntId other) const {
    return value_ == other.value_;
  }

  /** Compares one identifier with another of the same type for inequality. */
  bool operator!=(TypeSafeIntId other) const {
    return value_ != other.value_;
  }

  /** Generates a new identifier for this id type. This new identifier will be
   different from all previous identifiers created. This method does _not_
   make any guarantees about the values of ids from successive invocations.
   This class is _not_ thread safe. If thread safety is required, the caller is
   responsible for executing this in a thread-safe context.
   */
  static TypeSafeIntId get_new_id() {
    return TypeSafeIntId(next_index_++);
  }

 private:
  // Instantiates an identifier from the underlying representation type.
  explicit TypeSafeIntId(int64_t val) : value_(val) {}

  // The underlying value.
  int64_t value_;

  //
  static int64_t next_index_;
};

template <typename Tag>
int64_t TypeSafeIntId<Tag>::next_index_ = 0;

/** Streaming output operator.
 @relates TypeSafeIntId
 */
template <typename Tag>
inline std::ostream& operator<<(std::ostream& out,
                                const TypeSafeIntId<Tag>& id) {
  out << id.get_value();
  return out;
}

}  // namespace geometry
}  // namespace drake

namespace std {
/** Enables use of the identifier to serve as a key in STL containers.
 @relates TypeSafeIntId
 */
template <typename Tag>
struct hash<drake::geometry::TypeSafeIntId<Tag>> {
  size_t operator()(const drake::geometry::TypeSafeIntId<Tag>& id) const {
    // There is no hash function for int64_t. However, given that the ids
    // are intended to be unique, they can serve as their own hash value.
    return static_cast<size_t>(id.get_value());
  }
};

}  // namespace std
