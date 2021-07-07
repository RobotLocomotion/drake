#pragma once

/// @file
/// Provides a mechanism to map C++ types to canonical Python types.

#include <string>
#include <typeinfo>
#include <vector>

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/// Provides a publicly visible, but minimal, re-implementation of `py::object`
/// so that a public type can be used with `drake::Value<T>`, while still
/// maintaining the revelant semantics with its generic implementation (#13207).
/// This should *only* be used in place of `py::object` for public APIs that
/// rely on RTTI (e.g. `typeid`).
/// See implementation of `class object` in pybind11/include/pybind11/pytypes.h.
class Object {
 public:
  Object() {}

  /// Decrements reference count (if pointing to a real object).
  ~Object();

  /// Constructs from raw pointer, incrementing the reference count.
  /// @note This does not implement any of the `py::reinterpret_borrow<>`
  /// semantics.
  explicit Object(::PyObject* ptr);

  /// Constructs from another Object, incrementing the reference count.
  explicit Object(const Object& other);

  /// Steals object (and reference count) from another Object.
  Object(Object&& other);

  /// Copies object reference and increments reference count.
  Object& operator=(const Object& other);

  /// Steals object (and reference count) from another Object.
  Object& operator=(Object&& other);

  /// Accesses raw PyObject pointer (no reference counting).
  ::PyObject* ptr() const { return ptr_; }

  /// Converts to a pybind11 Python type, using py::reinterpret_borrow.
  template <typename T>
  T to_pyobject() const {
    return py::reinterpret_borrow<T>(ptr());
  }

  /// Converts from a pybind11 Python type, using py::reinterpret_borrow.
  template <typename T>
  static Object from_pyobject(const T& h) {
    return Object(h.ptr());
  }

 private:
  // Increments reference count. See `py::handle::inc_ref()` for more details.
  void inc_ref();

  // Decrements reference count. See `py::handle::dec_ref()` for more details.
  void dec_ref();

  ::PyObject* ptr_{};
};

namespace internal {

// Wrapper for Object.
struct wrapper_pydrake_object {
  using Type = Object;
  static constexpr auto original_name = py::detail::_("pydrake::Object");
  using WrappedType = py::object;
  static constexpr auto wrapped_name = py::detail::_("object");

  static Object unwrap(const py::object& obj) {
    return Object::from_pyobject(obj);
  }
  static py::object wrap(const Object& obj) {
    return obj.to_pyobject<py::object>();
  }
};

// Gets singleton for type aliases from `cpp_param`.
py::object GetParamAliases();

// Gets Python type object given `std::type_info`.
// @throws std::exception if type is neither aliased nor registered in
// `pybind11`.
py::object GetPyParamScalarImpl(const std::type_info& tinfo);

// Gets Python type for a C++ type (base case).
template <typename T>
inline py::object GetPyParamScalarImpl(type_pack<T> = {}) {
  static_assert(!py::detail::is_pyobject<T>::value,
      "You cannot use `pybind11` types (e.g. `py::object`). Use a publicly "
      "visible replacement type instead (e.g. `drake::pydrake::Object`).");
  return GetPyParamScalarImpl(typeid(T));
}

// Gets Python literal for a C++ literal (specialization).
template <typename T, T Value>
inline py::object GetPyParamScalarImpl(
    type_pack<std::integral_constant<T, Value>> = {}) {
  return py::cast(Value);
}

// Gets Python type for a C++ vector that is not registered using
// PYBIND11_MAKE_OPAQUE.
template <typename T>
inline py::object GetPyParamScalarImpl(type_pack<std::vector<T>> = {}) {
  // Get inner type for validation.
  py::object py_T = GetPyParamScalarImpl(type_pack<T>{});
  if constexpr (!internal::is_generic_pybind_v<std::vector<T>>) {
    return py::module::import("pydrake.common.cpp_param").attr("List")[py_T];
  } else {
    return GetPyParamScalarImpl(typeid(std::vector<T>));
  }
}

}  // namespace internal

/// Gets the canonical Python parameters for each C++ type.
/// @returns Python tuple of canonical parameters.
/// @throws std::exception on the first type it encounters that is neither
/// aliased nor registered in `pybind11`.
/// @tparam Ts The types to get C++ types for.
/// @pre Ts must be public symbols.
/// @pre Ts cannot be a `py::` symbol (e.g. `py::object`). On Mac, this may
/// cause failure depending on import order (e.g. trying to use
/// `Value<py::object>` between different modules). See #8704 and #13207 for
/// more details.
template <typename... Ts>
inline py::tuple GetPyParam(type_pack<Ts...> = {}) {
  return py::make_tuple(internal::GetPyParamScalarImpl(type_pack<Ts>{})...);
}

}  // namespace pydrake
}  // namespace drake

namespace pybind11 {
namespace detail {

template <>
struct type_caster<drake::pydrake::Object>
    : public drake::pydrake::internal::type_caster_wrapped<
          drake::pydrake::internal::wrapper_pydrake_object> {};

}  // namespace detail
}  // namespace pybind11
