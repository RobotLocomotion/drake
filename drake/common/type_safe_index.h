#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"

namespace drake {

/// A type-safe non-negative index that can be associated to a tag name.
/// Different instantiations of TypeSafeIndex are not interconvertible.
/// TypeSafeIndex allows for instantiations from an `int` as well as it allows
/// to convert back to an `int` via its conversion operator. Negative index
/// values are not allowed.
/// TypeSafeIndex can be used in places where an index is needed and therefore
/// basic operations such as prefix and postfix increment/decrement are
/// implemented. As an example, consider the creation of an index associated
/// with class `Foo`. This can be done in code as: <pre>
///   using FooIndex = TypeSafeIndex<class FooTag>;
/// </pre>
/// where the class `FooTag` above should simply be a dummy argument, a
/// never-defined type.
///
/// The TypeSafeIndex guarantees that index instances of different types can't
/// be compared or combined.  Efforts to do so will cause a compile-time
/// failure.  However, comparisons or operations on _other_ types that are
/// convertible to an int will succeed.  For example:
/// @code{.cpp}
///    using AIndex = TypeSafeIndex<class A>;
///    using BIndex = TypeSafeIndex<class B>;
///    AIndex a(1);
///    BIndex b(1);
///    if (a == 2) { ... }   // Ok.
///    size_t sz = 7;
///    if (a == sz) { ... }  // Ok.
///    if (a == b) { ... }   // <-- Compiler error.
/// @endcode
///
/// The intent of these classes is to seamlessly serve as indices into typical
/// indexed objects (e.g., vector, array, etc.). At the same time, we want to
/// avoid implicit conversions _from_ int to an index.  These two design
/// constraints combined lead to a limitation in how TypeSafeIndex instances
/// can be used.  Specifically, we've lost a common index pattern:
/// @code{.cpp}
///    for (MyIndex a = 0; a < N; ++a) { ... }
/// @endcode
/// This pattern no longer works because it requires implicit conversion of int
/// to TypeSafeIndex. Instead, the following pattern needs to be used:
/// @code{.cpp}
///    for (MyIndex a(0); a < N; ++a) { ... }
/// @endcode
///
/// @tparam Tag The name of the tag associated with a class type.
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
