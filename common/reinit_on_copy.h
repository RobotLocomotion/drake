#pragma once

#include <type_traits>

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
///   reinit_on_copy<int> use_count_;
/// };
/// </pre>
///
/// When copying from `Foo`, the new object will contain a copy of `items_`
/// but `use_count_` will be zero.  If `Foo` had not used the
/// `reinit_on_copy` wrapper, `use_count_` would have been copied also,
/// which we're assuming is not the desired behavior here.
///
/// @tparam T must support CopyConstructible, CopyAssignable, MoveConstructible,
/// and MoveAssignable.
template <typename T>
class reinit_on_copy {
 public:
  /// Constructs a reinit_on_copy<T> with a value-initialized wrapped value.
  /// See http://en.cppreference.com/w/cpp/language/value_initialization.
  reinit_on_copy() {}

  /// Constructs a reinit_on_copy<T> with the given wrapped value.  This is
  /// an implicit conversion, so that reinit_on_copy<T> behaves more like
  /// the unwrapped type.
  // NOLINTNEXTLINE(runtime/explicit)
  reinit_on_copy(const T& value) : value_(value) {}

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable.
  //@{
  /// Copy constructor just value-initializes instead.
  reinit_on_copy(const reinit_on_copy&) {}

  /// Copy assignment value-initializes _except_ for self-assignment.
  reinit_on_copy& operator=(const reinit_on_copy& source) {
    if (this != &source)
      new (&value_) T{};  // Use placement new to reconstruct.
    return *this;
  }

  reinit_on_copy(reinit_on_copy&& other) = default;

  reinit_on_copy& operator=(reinit_on_copy&& other) = default;
  //@}

  /// @name Implicit conversion operators to make reinit_on_copy<T> act
  /// as the wrapped type.
  //@{
  operator T&() { return value_; }
  operator const T&() const { return value_; }
  //@}

  /// @name Dereference operators available if type T is a pointer.
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
