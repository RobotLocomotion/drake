/// @file
/// Defines convenience utilities to wrap pybind11 methods and classes.

#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace pydrake {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

// Determines if a type will go through pybind11's generic caster. This
// implies that the type has been declared using `py::class_`, and can have
// a reference passed through. Otherwise, the type uses type-conversion:
// https://pybind11.readthedocs.io/en/stable/advanced/cast/index.html
template <typename T>
constexpr inline bool is_generic_pybind_v =
    std::is_base_of_v<py::detail::type_caster_generic,
        py::detail::make_caster<T>>;

template <typename T, typename = void>
struct wrap_ref_ptr : public wrap_arg_default<T> {};

template <typename T>
struct wrap_ref_ptr<T&, std::enable_if_t<is_generic_pybind_v<T>>> {
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

// Implements a `py::detail::type_caster<>` specialization used to convert
// types using a specific wrapping policy.
// @tparam Wrapper
//  Struct which must provide `Type`, `WrappedType`, `unwrap`, `wrap`,
// `wrapped_name`, and `original_name`.
// Fails-fast at runtime if there is an attempt to pass by reference.
template <typename Wrapper>
struct type_caster_wrapped {
  using Type = typename Wrapper::Type;
  using WrappedType = typename Wrapper::WrappedType;
  using WrappedTypeCaster = py::detail::type_caster<WrappedType>;

  // Python to C++.
  bool load(py::handle src, bool converter) {
    WrappedTypeCaster caster;
    if (!caster.load(src, converter)) {
      return false;
    }
    value_ = Wrapper::unwrap(caster.operator WrappedType&());
    loaded_ = true;
    return true;
  }

  // See `pybind11/eigen.h`, `type_caster<>` implementations.
  // N.B. Do not use `PYBIND11_TYPE_CASTER(...)` so we can avoid casting
  // garbage values.
  operator Type&() {
    if (!loaded_) {
      throw py::cast_error("Internal error: value not loaded?");
    }
    return value_;
  }

  template <typename T>
  using cast_op_type = py::detail::movable_cast_op_type<T>;

  static constexpr auto name = Wrapper::wrapped_name;

  // C++ to Python.
  template <typename TType>
  static py::handle cast(
      TType&& src, py::return_value_policy policy, py::handle parent) {
    if (policy == py::return_value_policy::reference ||
        policy == py::return_value_policy::reference_internal) {
      // N.B. We must declare a local `static constexpr` here to prevent
      // linking errors. This does not appear achievable with
      // `constexpr char[]`, so we use `py::detail::descr`.
      // See `pybind11/pybind11.h`, `cpp_function::initialize(...)` for an
      // example.
      static constexpr auto original_name = Wrapper::original_name;
      throw py::cast_error(
          std::string("Can only pass ") + original_name.text + " by value.");
    }
    return WrappedTypeCaster::cast(
        Wrapper::wrap(std::forward<TType>(src)), policy, parent);
  }

 private:
  bool loaded_{false};
  Type value_;
};

}  // namespace internal
#endif  // DRAKE_DOXYGEN_CXX

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
  return WrapFunction<internal::wrap_callback, false>(std::forward<Func>(func));
}

/// Idempotent to pybind11's `def_readwrite()`, with the exception that the
/// setter is protected with keep_alive on a `member` variable that is a bare
/// pointer.  Should not be used for unique_ptr members.
///
/// @tparam PyClass the python class.
/// @tparam Class the C++ class.
/// @tparam T type for the member we wish to apply keep alive semantics.
template <typename PyClass, typename Class, typename T>
void DefReadWriteKeepAlive(
    PyClass* cls, const char* name, T Class::*member, const char* doc = "") {
  auto getter = [member](const Class* obj) { return obj->*member; };
  auto setter = [member](Class* obj, const T& value) { obj->*member = value; };
  cls->def_property(name,  // BR
      py::cpp_function(getter),
      py::cpp_function(setter,
          // Keep alive, reference: `self` keeps `value` alive.
          py::keep_alive<1, 2>()),
      doc);
}

/// Idempotent to pybind11's `def_readonly()`, which works for unique_ptr
/// elements; the getter is protected with keep_alive on a `member` variable
/// that is a unique_ptr.
///
/// @tparam PyClass the python class.
/// @tparam Class the C++ class.
/// @tparam T type for the member we wish to apply keep alive semantics.
template <typename PyClass, typename Class, typename T>
void DefReadUniquePtr(PyClass* cls, const char* name,
    const std::unique_ptr<T> Class::*member, const char* doc = "") {
  auto getter = py::cpp_function(
      [member](const Class* obj) { return (obj->*member).get(); },
      py_rvp::reference_internal);
  cls->def_property_readonly(name, getter, doc);
}

// Variant of DefReadUniquePtr() for copyable_unique_ptr.
template <typename PyClass, typename Class, typename T>
void DefReadUniquePtr(PyClass* cls, const char* name,
    const copyable_unique_ptr<T> Class::*member, const char* doc = "") {
  auto getter = py::cpp_function(
      [member](const Class* obj) { return (obj->*member).get(); },
      py_rvp::reference_internal);
  cls->def_property_readonly(name, getter, doc);
}

}  // namespace pydrake
}  // namespace drake
