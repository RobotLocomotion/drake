#pragma once

#include <limits>
#include <string>

#include "drake/common/drake_assert.h"
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
///   - Valid index values are _explicitly_ constructed from `int` values.
///   - The index is implicitly convertible to an `int` (to serve as an index).
///   - The index supports increment, decrement, and in-place addition and
///     subtraction to support standard index-like operations.
///   - An index _cannot_ be constructed or compared to an index of another
///     type.
///   - In general, indices of different types are _not_ interconvertible.
///   - Binary integer operators (e.g., +, -, |, *, etc.) _always_ produce `int`
///     return values. One can even use operands of different index types in
///     such a binary expression. It is the _programmer's_ responsibility to
///     confirm that the resultant `int` value has meaning.
///
/// While there _is_ the concept of an "invalid" index, this only exists to
/// support default construction _where appropriate_ (e.g., using indices in
/// STL containers). Using an invalid index in _any_ operation is considered
/// an error. In Debug build, attempts to compare, increment, decrement, etc. an
/// invalid index will throw an exception.
///
/// A function that returns %TypeSafeIndex values which need to communicate
/// failure should _not_ use an invalid index. It should return an
/// `std::optional<Index>` instead.
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
///    AIndex invalid;          // Creates an invalid index.
///    ++invalid;               // Runtime error in Debug build.
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
/// In principle, the TypeSafeIndex is related to the
/// @ref drake::geometry::Identifier "Identifier". In
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
///   - The @ref drake::geometry::Identifier "Identifier" is the most
///     restricted. They exist solely to serve as a unique identifier. They are
///     immutable when created. Very few operations exist on them (comparison
///     for _equality_ with other identifiers of the same type, hashing, writing
///     to output stream).
///
/// Ultimately, indexes _can_ serve as identifiers (within the scope of the
/// object they index into). Although, their mutability could make this a
/// dangerous practice for a public API. Identifiers are more general in that
/// they don't reflect an object's position in memory (hence the inability to
/// transform to or compare with an `int`). This decouples details of
/// implementation from the idea of the object. Combined with its immutability,
/// it would serve well as a element of a public API.
///
/// @sa drake::geometry::Identifier
///
/// @tparam Tag The name of the tag associated with a class type. The class
///             need not be a defined class.
template <class Tag>
class TypeSafeIndex {
 public:
  /// @name           Constructors
  ///@{

  /// Default constructor; the result is an _invalid_ index. This only
  /// exists to serve applications which require a default constructor.
  TypeSafeIndex() {}

  /// Construction from a non-negative `int` value.
  /// Constructor only promises to enforce non-negativity in Debug build.
  explicit TypeSafeIndex(int index) : index_(index) {
    DRAKE_ASSERT_VOID(
        AssertValid("Explicitly constructing an invalid index."));
  }

  /// Disallow construction from another index type.
  template <typename U>
  TypeSafeIndex( const TypeSafeIndex<U>& idx) = delete;

  TypeSafeIndex(const TypeSafeIndex&) = default;

  TypeSafeIndex(TypeSafeIndex&& other) noexcept : index_(other.index_) {
    other.index_ = kDefaultInvalid;
  }
  ///@}

  /// @name       Assignment
  ///@{

  TypeSafeIndex& operator=(const TypeSafeIndex&) = default;

  TypeSafeIndex& operator=(TypeSafeIndex&& other) noexcept {
    index_ = other.index_;
    other.index_ = kDefaultInvalid;
    return *this;
  }

  /// Assign the index a value from a non-negative int.
  /// In Debug builds, this method asserts that the input index is non-negative.
  TypeSafeIndex& operator=(int idx) {
    index_ = idx;
    DRAKE_ASSERT_VOID(AssertValid("Assigning an invalid int."));
    return *this;
  }

  ///@}

  /// @name     Utility methods
  ///@{

  /// Implicit conversion-to-int operator.
  operator int() const {
    DRAKE_ASSERT_VOID(AssertValid("Converting to an int."));
    return index_;
  }

  /// Reports if the index is valid--the only operation on an invalid index
  /// that doesn't throw an exception in Debug builds.
  bool is_valid() const {
    // All other error testing, with assert armed, indirectly enforces the
    // invariant that the only way to get an invalid index is via the default
    // constructor. This assertion will catch any crack in that effort.
    DRAKE_ASSERT((index_ >= 0) || (index_ == kDefaultInvalid));
    return index_ >= 0;
  }

  ///@}

  /// @name Arithmetic operators
  ///@{

  /// Prefix increment operator.
  const TypeSafeIndex& operator++() {
    DRAKE_ASSERT_VOID(
        AssertValid("Pre-incrementing an invalid index."));
    DRAKE_ASSERT_VOID(
        AssertNoOverflow(1, "Pre-incrementing produced an invalid index."));
    ++index_;
    return *this;
  }

  /// Postfix increment operator.
  TypeSafeIndex operator++(int) {
    DRAKE_ASSERT_VOID(
        AssertValid("Post-incrementing an invalid index."));
    DRAKE_ASSERT_VOID(
        AssertNoOverflow(1, "Post-incrementing produced an invalid index."));
    ++index_;
    return TypeSafeIndex(index_ - 1);
  }

  /// Prefix decrement operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  const TypeSafeIndex& operator--() {
    DRAKE_ASSERT_VOID(
        AssertValid("Pre-decrementing an invalid index."));
    --index_;
    DRAKE_ASSERT_VOID(
        AssertValid("Pre-decrementing produced an invalid index."));
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex operator--(int) {
    DRAKE_ASSERT_VOID(
        AssertValid("Post-decrementing an invalid index."));
    --index_;
    DRAKE_ASSERT_VOID(AssertValid(
        "Post-decrementing produced an invalid index."));
    return TypeSafeIndex(index_ + 1);
  }
  ///@}

  /// @name Compound assignment operators
  ///@{

  /// Addition assignment operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex& operator+=(int i) {
    DRAKE_ASSERT_VOID(
        AssertValid("In-place addition with an int on an invalid index."));
    DRAKE_ASSERT_VOID(AssertNoOverflow(
        i, "In-place addition with an int produced an invalid index."));
    index_ += i;
    DRAKE_ASSERT_VOID(AssertValid(
        "In-place addition with an int produced an invalid index."));
    return *this;
  }

  /// Whitelist addition for indices with the same tag.
  TypeSafeIndex<Tag>& operator+=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid(
        "In-place addition with another index invalid LHS."));
    DRAKE_ASSERT_VOID(other.AssertValid(
        "In-place addition with another index invalid RHS."));
    DRAKE_ASSERT_VOID(AssertNoOverflow(
        other.index_,
        "In-place addition with another index produced an invalid index."));
    index_ += other.index_;
    DRAKE_ASSERT_VOID(AssertValid(
        "In-place addition with another index produced an invalid index."));
    return *this;
  }

  /// Blacklist addition for indices of different tags.
  template <typename U>
  TypeSafeIndex<U>& operator+=(const TypeSafeIndex<U>& u) = delete;

  /// Subtraction assignment operator.
  /// In Debug builds, this method asserts that the resulting index is
  /// non-negative.
  TypeSafeIndex& operator-=(int i) {
    DRAKE_ASSERT_VOID(
        AssertValid("In-place subtraction with an int on an invalid index."));
    DRAKE_ASSERT_VOID(AssertNoOverflow(
        -i, "In-place subtraction with an int produced an invalid index."));
    index_ -= i;
    DRAKE_ASSERT_VOID(AssertValid(
        "In-place subtraction with an int produced an invalid index."));
    return *this;
  }

  /// Whitelist subtraction for indices with the same tag.
  TypeSafeIndex<Tag>& operator-=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid(
        "In-place subtraction with another index invalid LHS."));
    DRAKE_ASSERT_VOID(other.AssertValid(
        "In-place subtraction with another index invalid RHS."));
    // No test for overflow; it would only be necessary if other had a negative
    // index value. In that case, it would be invalid and that would be caught
    // by the previous assertion.
    index_ -= other.index_;
    DRAKE_ASSERT_VOID(AssertValid(
        "In-place subtraction with another index produced an invalid index."));
    return *this;
  }

  /// Blacklist subtraction for indices of different tags.
  template <typename U>
  TypeSafeIndex<U>& operator-=(const TypeSafeIndex<U>& u) = delete;

  ///@}

  /// @name Exclusive comparison operators
  ///
  /// In order to prevent indices _of different type_ being added together or
  /// compared against each other, we apply a whitelist/blacklist approach to
  /// explicitly include indices of this type, but exclude indices of all other
  /// types.  This implicitly allows all _other_ objects that can be converted
  /// to int types.
  ///@{

  /// Whitelist equality test with indices of this tag.
  bool operator==(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid("Testing == with invalid LHS."));
    DRAKE_ASSERT_VOID(
        other.AssertValid("Testing == with invalid RHS."));
    return index_ == other.index_;
  }

  /// Blacklist equality tests with indices of other tags.
  template <typename U>
  bool operator==(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist inequality test with indices of this tag.
  bool operator!=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid("Testing != with invalid LHS."));
    DRAKE_ASSERT_VOID(
        other.AssertValid("Testing != with invalid RHS."));
    return index_ != other.index_;
  }

  /// Blacklist inequality test with indices of other tags.
  template <typename U>
  bool operator!=(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist less than test with indices of this tag.
  bool operator<(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid("Testing < with invalid LHS."));
    DRAKE_ASSERT_VOID(
        other.AssertValid("Testing < with invalid RHS."));
    return index_ < other.index_;
  }

  /// Blacklist less than test with indices of other tags.
  template <typename U>
  bool operator<(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist less than or equals test with indices of this tag.
  bool operator<=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid("Testing <= with invalid LHS."));
    DRAKE_ASSERT_VOID(
        other.AssertValid("Testing <= with invalid RHS."));
    return index_ <= other.index_;
  }

  /// Blacklist less than or equals test with indices of other tags.
  template <typename U>
  bool operator<=(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist greater than test with indices of this tag.
  bool operator>(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid("Testing > with invalid LHS."));
    DRAKE_ASSERT_VOID(
        other.AssertValid("Testing > with invalid RHS."));
    return index_ > other.index_;
  }

  /// Blacklist greater than test with indices of other tags.
  template <typename U>
  bool operator>(const TypeSafeIndex<U>& u) = delete;

  /// Whitelist greater than or equals test with indices of this tag.
  bool operator>=(const TypeSafeIndex<Tag>& other) {
    DRAKE_ASSERT_VOID(AssertValid("Testing >= with invalid LHS."));
    DRAKE_ASSERT_VOID(
        other.AssertValid("Testing >= with invalid RHS."));
    return index_ >= other.index_;
  }

  /// Blacklist greater than or equals test with indices of other tags.
  template <typename U>
  bool operator>=(const TypeSafeIndex<U>& u) = delete;

  ///@}

 private:
  // Checks if this index is negative and if so it throws an exception.
  // Invocations provide a string explaining the origin of the bad value.
  void AssertValid(const char* source) const {
    if (index_ < 0) {
      throw std::runtime_error(
          std::string(source) + " Type \"" +
              drake::NiceTypeName::Get<TypeSafeIndex<Tag>>() +
              "\", has the negative value = " + std::to_string(index_) +
              ". Negative indexes are not allowed.");
    }
  }

  // This tests for overflow conditions based on adding the given delta into
  // the current index value.
  void AssertNoOverflow(int delta, const char* source) const {
    if (delta > 0 && index_ > std::numeric_limits<int>::max() - delta) {
      throw std::runtime_error(
          std::string(source) + " Type \"" +
          drake::NiceTypeName::Get<TypeSafeIndex<Tag>>() +
          "\", has overflowed.");
    }
  }

  // This value helps distinguish indices that are invalid through construction
  // or moves versus user manipulation.
  enum {
    kDefaultInvalid = -1234567
  };

  int index_{kDefaultInvalid};
};

}  // namespace drake
