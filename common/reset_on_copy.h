#pragma once

#include <type_traits>
#include <utility>

namespace drake {

// NOTE(sherm1) to future implementers: if you decide to extend this adapter for
// use with class types, be sure to think carefully about the semantics of copy
// and move and how to explain that to users. Be aware that for class types T,
// the implementation of `T{}` (the default constructor, which may have been
// user-supplied) won't necessarily reset T's members to zero, nor even
// necessarily value-initialize T's members. Also, the "noexcept" reasoning
// below is more than we need with the std::is_scalar<T> restriction, but is
// strictly necessary for class types if you want std::vector to choose move
// construction (content-preserving) over copy construction (resetting).
/// Type wrapper that performs value-initialization on copy construction or
/// assignment.
///
/// Rather than copying the source supplied for copy construction or copy
/// assignment, this wrapper instead value-initializes the destination object.
/// Move assignment and construction preserve contents in the destination as
/// usual, but reset the source to its value-initialized value.
///
/// Only types T that satisfy `std::is_scalar<T>` are currently
/// permitted: integral and floating point types, enums, and pointers.
/// Value initialization means the initialization performed when a variable is
/// constructed with an empty initializer `{}`. For the restricted set of types
/// we support, that just means that numeric types are set to zero and pointer
/// types are set to nullptr. Also, all the methods here are noexcept due to the
/// `std::is_scalar<T>` restriction.
/// See http://en.cppreference.com/w/cpp/language/value_initialization.
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
/// designated ones, which are value-initialized instead. The resetting of the
/// source on move doesn't change semantics since the condition of the source
/// after a move is generally undefined. It is instead opportunistic good
/// hygiene for early detection of bugs, taking advantage of the fact that we
/// know type T can be value-initialized. See reset_after_move for more
/// discussion.
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
/// @warning Even if you initialize a %reset_on_copy member to a non-zero value
/// using an initializer like `reset_on_copy<int> some_member_{5}` it will be
/// _reset_ to zero, not _reinitialized_ to 5 when copied.
///
/// @note Enum types T are permitted, but be aware that they will be reset to
/// zero, regardless of whether 0 is one of the specified enumeration values.
///
/// @tparam T must satisfy `std::is_scalar<T>`.
/// @see reset_after_move
template <typename T>
class reset_on_copy {
 public:
  static_assert(std::is_scalar<T>::value,
                "reset_on_copy<T> is permitted only for integral, "
                "floating point, and pointer types T.");

  /// Constructs a reset_on_copy<T> with a value-initialized wrapped value.
  reset_on_copy() noexcept(std::is_nothrow_default_constructible<T>::value) {}

  /// Constructs a %reset_on_copy<T> with a copy of the given value. This is
  /// an implicit conversion, so that %reset_on_copy<T> behaves more like
  /// the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reset_on_copy(const T& value) noexcept(
      std::is_nothrow_copy_constructible<T>::value)
      : value_(value) {}

  /// Constructs a %reset_on_copy<T> with the given wrapped value, by move
  /// construction if possible. This is an implicit conversion, so that
  /// %reset_on_copy<T> behaves more like the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reset_on_copy(T&& value) noexcept(
      std::is_nothrow_move_constructible<T>::value)
      : value_(std::move(value)) {}

  /// @name Implements copy/move construction and assignment.
  /// These make %reset_on_copy objects CopyConstructible, CopyAssignable,
  /// MoveConstructible, and MoveAssignable.
  //@{

  /// Copy constructor just value-initializes instead; the source is ignored.
  reset_on_copy(const reset_on_copy&) noexcept(
      std::is_nothrow_default_constructible<T>::value) {}

  /// Copy assignment just destructs the contained value and then
  /// value-initializes it, _except_ for self-assignment which does nothing.
  /// The source argument is otherwise ignored.
  reset_on_copy& operator=(const reset_on_copy& source) noexcept(
      std::is_nothrow_destructible<T>::value&&
          std::is_nothrow_default_constructible<T>::value) {
    if (this != &source) destruct_and_reset_value();
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

  /// Move assignment uses T's move assignment, then destructs and value
  /// initializes the source, _except_ for self-assignment which does nothing.
  /// The source argument is otherwise ignored.
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

  /// @name Dereference operators if T is a pointer type.
  /// If type T is a pointer, these exist and return the pointed-to object.
  /// For non-pointer types these methods are not instantiated.
  //@{
  template <typename T1 = T>
  std::enable_if_t<std::is_pointer<T1>::value, T> operator->() const noexcept {
    return value_;
  }

  template <typename T1 = T>
  std::enable_if_t<std::is_pointer<T1>::value,
                   std::add_lvalue_reference_t<std::remove_pointer_t<T>>>
  operator*() const noexcept {
    return *value_;
  }
  //@}

 private:
  // Invokes T's destructor if there is one, then value-initializes. Note that
  // the noexcept code above assumes exactly this implementation. Don't change
  // this to `value_ = T{}` which would introduce an additional dependence on
  // the behavior of T's copy assignment operator.
  void destruct_and_reset_value() {
    value_.~T();  // Invokes destructor if there is one.
    new (&value_) T{};  // Placement new; no heap activity.
  }

  T value_{};
};

}  // namespace drake
