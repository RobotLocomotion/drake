#pragma once

#include <utility>

namespace drake {

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
///   reinit_after_move<int> sum_;
/// };
/// </pre>
///
/// When moving from `Foo`, the donor object will reset to its default state:
/// `items_` will be empty and `sum_` will be zero.  If `Foo` had not used the
/// `reinit_after_move` wrapper, the `sum_` would remain intact (be copied)
/// while moving, even though `items_` was cleared.
///
/// @tparam T must support CopyConstructible, CopyAssignable, MoveConstructible,
/// and MoveAssignable and must not throw exceptions during construction or
/// assignment.
template <typename T>
class reinit_after_move {
 public:
  /// Constructs a reinit_after_move<T> with a value-initialized wrapped value.
  /// See http://en.cppreference.com/w/cpp/language/value_initialization.
  reinit_after_move() {}

  /// Constructs a reinit_after_move<T> with the given wrapped value.  This is
  /// an implicit conversion, so that reinit_after_move<T> behaves more like
  /// the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reinit_after_move(const T& value) : value_(value) {}

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable.
  /** @{ */
  reinit_after_move(const reinit_after_move&) = default;
  reinit_after_move& operator=(const reinit_after_move&) = default;
  reinit_after_move(reinit_after_move&& other) {
    value_ = std::move(other.value_);
    other.value_ = T{};
  }
  reinit_after_move& operator=(reinit_after_move&& other) {
    if (this != &other) {
      value_ = std::move(other.value_);
      other.value_ = T{};
    }
    return *this;
  }
  /** @} */

  /// @name Implicit conversion operators to make reinit_after_move<T> to act
  /// as the wrapped type.
  /** @{ */
  operator T&() { return value_; }
  operator const T&() const { return value_; }
  /** @} */

 private:
  T value_{};
};

}  // namespace drake
