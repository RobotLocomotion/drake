/// @file
/// Defines convenience utilities to wrap pybind11 methods and classes.

#pragma once

#include <string>
#include <utility>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/util/wrap_function.h"

namespace drake {
namespace pydrake {

/// Defines a function in object `a` and mirrors `def` calls to object `b`.
///
/// @tparam A Type of object `a`
/// @tparam B Type of object `b`
template <typename A, typename B>
class MirrorDef {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MirrorDef);

  MirrorDef(A* a, B* b)
      : a_(a), b_(b) {}

  /// Calls `def` for both `a` and `b`.
  template <typename... Args>
  MirrorDef& def(const char* name, Args&&... args) {
    a_->def(name, std::forward<Args>(args)...);
    b_->def(name, std::forward<Args>(args)...);
    return *this;
  }

 private:
  A* const a_{};
  B* const b_{};
};

namespace detail {

template <typename T, typename = void>
struct wrap_ref_ptr : public wrap_arg_default<T> {};

template <typename T>
using is_generic_pybind =
  std::is_base_of<py::detail::type_caster_generic, py::detail::make_caster<T>>;

// TODO(eric.cousineau): Only use this on non-primitive types (e.g. types whose
// `type_caster`s are generic).
template <typename T>
struct wrap_ref_ptr<T&, std::enable_if_t<is_generic_pybind<T>::value>> {
  // NOLINTNEXTLINE[runtime/references]: Intentional.
  static T* wrap(T& arg) { return &arg; }
  static T& unwrap(T* arg_wrapped) { return *arg_wrapped; }
};

template <typename T, typename = void>
struct wrap_callback : public wrap_arg_default<T> {};

template <typename Signature>
struct wrap_callback<const std::function<Signature>&>
    : public wrap_arg_function<wrap_ref_ptr, Signature> {};

template <typename Signature>
struct wrap_callback<std::function<Signature>>
    : public wrap_callback<const std::function<Signature>&> {};

}  // namespace detail

/// Ensures that any `std::function<>` arguments are wrapped such that any `T&`
/// (which can infer for `T = const U`) is wrapped as `U*` (and conversely
/// unwrapped when returned).
/// Arguments with const or mutable references to objects that may only exist
/// in C++, and not yet in Python, may cause `pybind11` to trigger a copy, when
/// it's only valid to use a reference.
/// For more information, see: https://github.com/pybind/pybind11/issues/1241
template <typename Func>
auto WrapCallbacks(Func&& func) {
  return WrapFunction<detail::wrap_callback, false>(std::forward<Func>(func));
}

}  // namespace pydrake
}  // namespace drake
