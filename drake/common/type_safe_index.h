#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"

namespace drake {

/// A type-safe non-negative index class.
///
/// This class serves as an upgrade to the standard practice of passing `int`s
/// around as indices. In the common practice, a method that takes indices into
/// multiple collections would have an interface like:
///
/// @code
/// void foo(int bar_index, int thing_index);
/// @endcode
///
/// It is possible for a programmer to accidentally switch the two index values
/// in an invocation.  This mistake would still be _syntactically_ correct; it
/// will successfully compile but lead to inscrutable run-time errors. The
/// type-safe index provides the same speed and efficiency of passing `int`s,
/// but provides compile-time checking. The function would now look like:
///
/// @code
/// void foo(BarIndex bar_index, ThingIndex thing_index);
/// @endcode
///
/// and the compiler will catch instances where the order is reversed.
///
/// The type-safe index is a _stripped down_ `int`. Each uniquely declared
/// index type has the following properties:
///
///   - Index values are _explicitly_ constructed from `int` values.
///   - The index is implicitly convertible to an `int` (to serve as an index).
///   - The index supports increment, decrement, and in-place addition and
///     subtraction to support standard index-like operations.
///   - An index _cannot_ be constructed or compared to an index of another
///     type.
///   - In general, indices of different types are _not_ interconvertible.
///
/// There is no such thing as an "invalid" index; there is no sentinel
/// value which indicates uninitialized or undefined. Operations which return
/// an index, but can fail, should communicate this in the function interface
/// (e.g. through the std::optional<IndexType> return value). If an index
/// exists, it should be considered valid.
///
/// It is the designed intent of this class, that indices derived from this
/// class can be passed and returned by value. Passing indices by const
/// reference should be considered a misuse.
///
/// This is the recommended method to create a unique index type associated with
/// class `Foo`:
///
/// @code
/// using FooIndex = TypeSafeIndex<class FooTag>;
/// @endcode
///
/// This references a non-existent, and ultimately anonymous, class `FooTag`.
/// This is sufficient to create a unique index type. It is certainly possible
/// to use an existing class (e.g., `Foo`). But this provides no functional
/// benefit.
///
/// __Examples of valid and invalid operations__
///
/// The TypeSafeIndex guarantees that index instances of different types can't
/// be compared or combined.  Efforts to do so will cause a compile-time
/// failure.  However, comparisons or operations on _other_ types that are
/// convertible to an int will succeed.  For example:
/// @code
///    using AIndex = TypeSafeIndex<class A>;
///    using BIndex = TypeSafeIndex<class B>;
///    AIndex a(1);
///    BIndex b(1);
///    if (a == 2) { ... }      // Ok.
///    size_t sz = 7;
///    if (a == sz) { ... }     // Ok.
///    if (a == b) { ... }      // <-- Compiler error.
/// @endcode
///
/// As previously stated, the intent of this class is to seamlessly serve as an
/// index into indexed objects (e.g., vector, array, etc.). At the same time, we
/// want to avoid implicit conversions _from_ int to an index.  These two design
/// constraints combined lead to a limitation in how TypeSafeIndex instances
/// can be used.  Specifically, we've lost a common index pattern:
///
/// @code
///    for (MyIndex a = 0; a < N; ++a) { ... }
/// @endcode
///
/// This pattern no longer works because it requires implicit conversion of int
/// to TypeSafeIndex. Instead, the following pattern needs to be used:
///
/// @code
///    for (MyIndex a(0); a < N; ++a) { ... }
/// @endcode
///
/// __Type-safe Index vs Identifier__
///
/// In principle, the TypeSafeIndex is related to the Identifier. In
/// some sense, both are "type-safe `int`s". They differ in their semantics. We
/// can consider `ints`, indexes, and identifiers as a list of `int` types with
/// _decreasing_ functionality.
///
///   - The int, obviously, has the full range of C++ ints.
///   - The TypeSafeIndex can be implicitly cast *to* an int, but there are a
///     limited number of operations _on_ the index that produces other
///     instances of the index (e.g., increment, in-place addition, etc.) They
///     can be compared with `int` and other indexes of the same type. This
///     behavior arises from the intention of having them serve as an _index_ in
///     an ordered set (e.g., `std::vector`.)
///   - The Identifier is the most restricted. They exist solely to serve as
///     a unique identifier. They are immutable when created. Very few
///     operations exist on them (comparison for _equality_ with other
///     identifiers of the same type, hashing, writing to output stream).
///
/// Ultimately, indexes _can_ serve as identifiers (within the scope of the
/// object they index into). Although, their mutability could make this a
/// dangerous practice for a public API. Identifiers are more general in that
/// they don't reflect an object's position in memory (hence the inability to
/// transform to or compare with an `int`). This decouples details of
/// implementation from the idea of the object. Combined with its immutability,
/// it would serve well as a element of a public API.
///
/// @sa Identifier
///
/// @tparam Tag The name of the tag associated with a class type. The class
///             need not be a defined class.
template <class Tag>
class TypeSafeIndex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypeSafeIndex)

  /// Default constructor is disabled to force users to initialize their indexes
  /// at creation.
  TypeSafeIndex() = delete;

  /// Construction from a non-negative `int` value.
  /// Constructor only promises to enforce non-negativity in Debug build.
  explicit TypeSafeIndex(int index) : index_(index) {
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  /// Disallow construction from another index type.
  template <typename U>
  TypeSafeIndex( const TypeSafeIndex<U>& idx) = delete;

  /// Assign the index a value from a non-negative int.
  /// In Debug builds, this method asserts that the input index is non-negative.
  TypeSafeIndex& operator=(int idx) {
    index_ = idx;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Implicit conversion-to-int operator.
  operator int() const { return index_; }

  /// @name Arithmetic operators.
  ///@{

  /// Prefix increment operator.
  const TypeSafeIndex& operator++() {
    ++index_;
    return *this;
  }

  /// Postfix increment operator.
  TypeSafeIndex operator++(int) {
    ++index_;
    return TypeSafeIndex(index_ - 1);
  }

  /// Prefix decrement operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  const TypeSafeIndex& operator--() {
    --index_;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex operator--(int) {
    --index_;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return TypeSafeIndex(index_ + 1);
  }
  ///@}

  /// @name Compound assignment operators.
  ///@{

  /// Addition assignment operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex& operator+=(int i) {
    index_ += i;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Subtraction assignment operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex& operator-=(int i) {
    index_ -= i;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }
  ///@}

  /// @name Exclusionary operators
  ///
  /// In order to prevent indices _of different type_ being added together or
  /// compared against each other, we apply a whitelist/blacklist approach to
  /// explicitly include indices of this type, but exclude indices of all other
  /// types.  This implicitly allows all _other_ objects that can be converted
  /// to int types.
  ///@{

  /// Whitelist equality test with indices of this tag.
  bool operator==(const TypeSafeIndex<Tag>& other) {
    return index_ == other.index_;
  }

  /// Blacklist equality tests with indices of other tags.
  template <typename U>
  bool operator==(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist inequality test with indices of this tag.
  bool operator!=(const TypeSafeIndex<Tag>& other) {
    return index_ != other.index_;
  }

  /// Blacklist inequality test with indices of other tags.
  template <typename U>
  bool operator!=(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist less than test with indices of this tag.
  bool operator<(const TypeSafeIndex<Tag>& other) {
    return index_ < other.index_;
  }

  /// Blacklist less than test with indices of other tags.
  template <typename U>
  bool operator<(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist less than or equals test with indices of this tag.
  bool operator<=(const TypeSafeIndex<Tag>& other) {
    return index_ <= other.index_;
  }

  /// Blacklist less than or equals test with indices of other tags.
  template <typename U>
  bool operator<=(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist greater than test with indices of this tag.
  bool operator>(const TypeSafeIndex<Tag>& other) {
    return index_ > other.index_;
  }

  /// Blacklist greater than test with indices of other tags.
  template <typename U>
  bool operator>(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist greater than or equals test with indices of this tag.
  bool operator>=(const TypeSafeIndex<Tag>& other) {
    return index_ >= other.index_;
  }

  /// Blacklist greater than or equals test with indices of other tags.
  template <typename U>
  bool operator>=(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist addition for indices with the same tag.
  TypeSafeIndex<Tag>& operator+=(const TypeSafeIndex<Tag>& other) {
    return *this += other.index_;
  }

  /// Blacklist addition for indices of different tags.
  template <typename U>
  TypeSafeIndex<U>& operator+=(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist subtraction for indices with the same tag.
  TypeSafeIndex<Tag>& operator-=(const TypeSafeIndex<Tag>& other) {
    return *this -= other.index_;
  }

  /// Blacklist subtraction for indices of different tags.
  template <typename U>
  TypeSafeIndex<U>& operator-=(const TypeSafeIndex<U>& u) = delete;

  ///@}

 private:
  // Checks if this index is negative and if so it throws an exception.
  void CheckInvariants() const {
    if (index_ < 0) {
      throw std::runtime_error(
          "This index, of type \"" +
              drake::NiceTypeName::Get<TypeSafeIndex<Tag>>() +
              "\", has the negative value = " + std::to_string(index_) +
              ". Negative indexes are not allowed.");
    }
  }

  int index_{0};
};

}  // namespace drake
