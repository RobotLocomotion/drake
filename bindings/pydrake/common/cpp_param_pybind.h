#pragma once

/// @file
/// Provides a mechanism to map C++ types to canonical Python types.

#include <string>
#include <typeinfo>
#include <vector>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

// Make a publicly visible symbol (#13207). This mirrors some of `py::object`
// API, in that it is a refcount-incrementing container for PyObject.
// See implementation of `class object` in pybind11/include/pybind11/pytypes.h.
// N.B. This is a bare implementation so that a public type can be used with
// `drake::Value<T>`, while still maintain the revelant semantics with
// its generic implementation.
class Object {
 public:
  Object() {}
  ~Object();

  // N.B. This does not implement any of the `py::reinterpret_borrow<>`
  // semantics.
  explicit Object(::PyObject* ptr);
  explicit Object(const Object& other);
  Object(Object&& other);
  Object& operator=(const Object& other);
  Object& operator=(Object&& other);

  void inc_ref();
  void dec_ref();
  ::PyObject* ptr() const { return ptr_; }

  template <typename T>
  T to_pyobject() const {
    return py::reinterpret_borrow<T>(ptr());
  }

  template <typename T>
  static Object from_pyobject(const T& h) {
    return Object(h.ptr());
  }

 private:
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
// @throws std::runtime_error if type is neither aliased nor registered in
// `pybind11`.
py::object GetPyParamScalarImpl(const std::type_info& tinfo);

// Gets Python type for a C++ type (base case).
template <typename T>
inline py::object GetPyParamScalarImpl(type_pack<T> = {}) {
  static_assert(!py::detail::is_pyobject<T>::value,
      "You cannot use `pybind11` types (e.g. `py::object`). Use a publicly "
      "visible replacement type instead, please.");
  return GetPyParamScalarImpl(typeid(T));
}

// Gets Python literal for a C++ literal (specialization).
template <typename T, T Value>
inline py::object GetPyParamScalarImpl(
    type_pack<std::integral_constant<T, Value>> = {}) {
  return py::cast(Value);
}

}  // namespace internal

/// Gets the canonical Python parameters for each C++ type.
/// @returns Python tuple of canonical parameters.
/// @throws std::runtime_error on the first type it encounters that is neither
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

namespace pybind11::detail {

// N.B. Since this is used with
template <>
struct type_caster<drake::pydrake::Object>
    : public drake::pydrake::internal::type_caster_wrapped<
          drake::pydrake::internal::wrapper_pydrake_object> {};

}  // namespace pybind11::detail
