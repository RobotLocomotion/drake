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
/// Use this when you have a callback in C++ that has a lvalue reference (const
/// or mutable) to a C++ argument or return value.
/// Otherwise, `pybind11` may try and copy the object, will be bad if either
/// the type is a non-copyable or if you are trying to mutate the object; in
/// this case, the copy is mutated, but not the original you care about.
/// For more information, see: https://github.com/pybind/pybind11/issues/1241
template <typename Func>
auto WrapCallbacks(Func&& func) {
  return WrapFunction<detail::wrap_callback, false>(std::forward<Func>(func));
}

/// Mirror ufunc loop definitions from NumPy to `math`
template <typename PyClass>
class UfuncMirrorDef {
 public:
  UfuncMirrorDef(PyClass* cls, py::module math)
    : cls_(cls), math_(math) {}

  template <typename Func>
  UfuncMirrorDef& def_loop(
      const char* cls_name, const char* math_name, const Func& func) {
    cls_->def_loop(cls_name, func);
    math_.def(math_name, func);
    return *this;
  }

  template <typename Func>
  UfuncMirrorDef& def_loop(
      const char* name, const Func& func) {
    return def_loop(name, name, func);
  }

 private:
  PyClass* const cls_{};
  py::module math_;
};

}  // namespace pydrake
}  // namespace drake
