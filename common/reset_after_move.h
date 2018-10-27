#pragma once

#include <utility>

namespace drake {

// TODO(sherm1) Upgrade this to match reset_on_copy (e.g. noexcept).
/// Type wrapper that performs value-initialization on the wrapped type, and
/// guarantees that when moving from this type that the donor object is reset
/// to its value-initialized value.
///
/// Background:
///
/// For performance reasons, we often like to provide overloaded move functions
/// on our types, instead of relying on the copy functions.  When doing so, it
/// is more robust to rely on the compiler's `= default` implementation using
/// member-wise move, instead of writing out the operations manually.  In
/// general, move functions should reset the donor object of the move to its
/// default-constructed (empty) resource state.  Inductively, the member
/// fields' existing move implementations do this already, except in the case
/// of non-class (primitive) members, where the donor object's primitives will
/// not be zeroed.  By wrapping primitive members fields with this type, they
/// are guaranteed to be zeroed on construction and after being moved from.
///
/// Example:
///
/// <pre>
/// class Foo {
///  public:
///   DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo)
///   Foo() = default;
///
///  private:
///   std::vector<int> items_;
///   reset_after_move<int> sum_;
/// };
/// </pre>
///
/// When moving from `Foo`, the donor object will reset to its default state:
/// `items_` will be empty and `sum_` will be zero.  If `Foo` had not used the
/// `reset_after_move` wrapper, the `sum_` would remain intact (be copied)
/// while moving, even though `items_` was cleared.
///
/// @tparam T must support CopyConstructible, CopyAssignable, MoveConstructible,
/// and MoveAssignable and must not throw exceptions during construction or
/// assignment.
/// @see reset_on_copy
template <typename T>
class reset_after_move {
 public:
  /// Constructs a reset_after_move<T> with a value-initialized wrapped value.
  /// See http://en.cppreference.com/w/cpp/language/value_initialization.
  reset_after_move() {}

  /// Constructs a reset_after_move<T> with the given wrapped value.  This is
  /// an implicit conversion, so that reset_after_move<T> behaves more like
  /// the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reset_after_move(const T& value) : value_(value) {}

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable.
  //@{
  reset_after_move(const reset_after_move&) = default;
  reset_after_move& operator=(const reset_after_move&) = default;
  reset_after_move(reset_after_move&& other) {
    value_ = std::move(other.value_);
    other.value_ = T{};
  }
  reset_after_move& operator=(reset_after_move&& other) {
    if (this != &other) {
      value_ = std::move(other.value_);
      other.value_ = T{};
    }
    return *this;
  }
  //@}

  /// @name Implicit conversion operators to make reset_after_move<T> act
  /// as the wrapped type.
  //@{
  operator T&() { return value_; }
  operator const T&() const { return value_; }
  //@}

  /// @name Dereference operators if T is a pointer type.
  /// If type T is a pointer, these exist and return the pointed-to object.
  /// For non-pointer types these methods are not instantiated.
  //@{
  template <typename T1 = T>
  std::enable_if_t<std::is_pointer<T1>::value, T> operator->() const {
    return value_;
  }
  template <typename T1 = T>
  std::enable_if_t<std::is_pointer<T1>::value,
                   std::add_lvalue_reference_t<std::remove_pointer_t<T>>>
  operator*() const {
    return *value_;
  }
  //@}

 private:
  T value_{};
};

}  // namespace drake
