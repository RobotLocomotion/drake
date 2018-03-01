#pragma once

#include <type_traits>
#include <utility>

namespace drake {

/// Type wrapper that performs value-initialization on the wrapped type, and
/// guarantees that when copying from this type the copied object is reset
/// to its value-initialized value. Move assignment and construction are
/// unaffected. Note that value initialization means the initialization
/// performed when a variable is constructed with an empty initializer. For
/// example, numeric types are set to zero and pointer types are set to nullptr.
///
/// Background:
///
/// It is preferable to use default copy construction for classes whenever
/// possible because it avoids difficult-to-maintain enumeration of member
/// fields in bespoke copy constructors. The presence of fields that must be
/// reset to zero in the copy (counters, for example) prevents use of default
/// copy construction. Similarly, pointers that would be invalid in the copy
/// need to be set to null to avoid stale references. By wrapping those
/// problematic data members in this adapter, default copy construction can
/// continue to be used, with all data members copied properly except the
/// designated ones, which are value-initialized instead.
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
///   reset_on_copy<int> use_count_;
/// };
/// </pre>
///
/// When copying from `Foo`, the new object will contain a copy of `items_`
/// but `use_count_` will be zero.  If `Foo` had not used the
/// `reset_on_copy` wrapper, `use_count_` would have been copied also,
/// which we're assuming is not the desired behavior here.
///
/// @note This is most useful for numeric and integer types but works for
/// any type T for which `T{}` provides the behavior you want. Be aware that for
/// class types T, the implementation of `T{}` (the default constructor, which
/// may have been user-supplied) won't necessarily reset T's members to zero,
/// nor even necessarily value-initialize T's members.
///
/// @tparam T must support CopyConstructible, CopyAssignable, MoveConstructible,
/// and MoveAssignable.
template <typename T>
class reset_on_copy {
 public:
  /// Constructs a reset_on_copy<T> with a value-initialized wrapped value.
  /// See http://en.cppreference.com/w/cpp/language/value_initialization.
  reset_on_copy() noexcept(std::is_nothrow_default_constructible<T>::value) {}

  /// Constructs a reset_on_copy<T> with a copy of the given value. This is
  /// an implicit conversion, so that reset_on_copy<T> behaves more like
  /// the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reset_on_copy(const T& value) noexcept(
      std::is_nothrow_copy_constructible<T>::value)
      : value_(value) {}

  /// Constructs a reset_on_copy<T> with the given wrapped value, by move
  /// construction if possible. This is an implicit conversion, so that
  /// reset_on_copy<T> behaves more like the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reset_on_copy(T&& value) noexcept(
      std::is_nothrow_move_constructible<T>::value)
      : value_(std::move(value)) {}

  /// @name Implements copy/move construction/assignment.
  /// These make %reset_on_copy objects CopyConstructible, CopyAssignable,
  /// MoveConstructible, and MoveAssignable. These methods will be noexcept
  /// if the required methods of type T are noexcept.
  //@{
  /// Copy constructor just value-initializes instead; the source is ignored.
  reset_on_copy(const reset_on_copy&) noexcept(
      std::is_nothrow_default_constructible<T>::value) {}

  /// Copy assignment just destructs the contained value and then
  /// value-initializes it, _except_ for self-assignment which does nothing.
  /// The source argument is otherwise ignored.
  reset_on_copy& operator=(const reset_on_copy& source) noexcept(
  std::is_nothrow_destructible<T>::value &&
      std::is_nothrow_default_constructible<T>::value) {
    if (this != &source)
      destruct_and_reset_value();
    return *this;
  }

  /// Move construction uses T's move constructor, then destructs and
  /// value initializes the source.
  reset_on_copy(reset_on_copy&& source) noexcept(
      std::is_nothrow_move_constructible<T>::value &&
          std::is_nothrow_destructible<T>::value &&
          std::is_nothrow_default_constructible<T>::value)
      : value_(std::move(source.value_)) {
    source.destruct_and_reset_value();
  }

  /// Move assignment uses T's move assignment, then destructs and
  /// value initializes the source.
  reset_on_copy& operator=(reset_on_copy&& source) noexcept(
      std::is_nothrow_move_assignable<T>::value &&
          std::is_nothrow_destructible<T>::value &&
          std::is_nothrow_default_constructible<T>::value) {
    if (this != &source) {
      value_ = std::move(source);
      source.destruct_and_reset_value();
    }
    return *this;
  }
  //@}

  /// @name Implicit conversion operators to make reset_on_copy<T> act
  /// as the wrapped type.
  //@{
  operator T&() noexcept { return value_; }
  operator const T&() const noexcept { return value_; }
  //@}

  /// @name Dereference operators available for access to T.
  /// If type T is a pointer, these return the pointed-to object. Otherwise
  /// they provide access to the contained object.
  //@{
  template <typename T1 = T>
  std::enable_if_t<std::is_pointer<T1>::value, T> operator->() const {
    return value_;
  }
  template <typename T1 = T>
  std::enable_if_t<!std::is_pointer<T1>::value, T*> operator->() {
    return &value_;
  }
  template <typename T1 = T>
  std::enable_if_t<std::is_pointer<T1>::value,
                   std::add_lvalue_reference_t<std::remove_pointer_t<T>>>
  operator*() const {
    return *value_;
  }
  template <typename T1 = T>
  std::enable_if_t<!std::is_pointer<T1>::value, std::add_lvalue_reference_t<T>>
  operator*() {
    return value_;
  }
  //@}

 private:
  // Invokes T's destructor if there is one, then value-initializes.
  void destruct_and_reset_value() { value_.~T(); new (&value_) T{}; }

  T value_{};
};

}  // namespace drake
