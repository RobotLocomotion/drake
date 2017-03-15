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

  /// Implicit conversion to int operator.
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
