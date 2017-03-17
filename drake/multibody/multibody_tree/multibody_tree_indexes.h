#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace drake {
namespace multibody {

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

  /// Default constructor is disabled to force users to intialize their indexes
  /// at creation.
  TypeSafeIndex() = delete;

  /// Construction from an `int` value.
  /// For Debug builds this constructor throws if the provided input `int`
  /// `index` is negative. There is no check for Release builds.
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
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  const TypeSafeIndex& operator--() {
    --index_;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
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
  TypeSafeIndex& operator+=(int i) {
    index_ += i;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Subtraction assignment operator.
  /// In Debug builds this method asserts that the resulting index is
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
              ". Negative indexes are not allowed");
    }
  }

  int index_{0};
};

/// Type used to identify frames by index in a multibody tree system.
using FrameIndex = TypeSafeIndex<class FrameTag>;

/// Type used to identify bodies by index in a multibody tree system.
using BodyIndex = TypeSafeIndex<class BodyTag>;

/// Type used to identify mobilizers by index in a multibody tree system.
using MobilizerIndex = TypeSafeIndex<class MobilizerTag>;

/// For every MultibodyTree the **world** body _always_ has this unique index
/// and it is always zero.
// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables
inline BodyIndex world_index() { return BodyIndex(0); }

}  // namespace multibody
}  // namespace drake
