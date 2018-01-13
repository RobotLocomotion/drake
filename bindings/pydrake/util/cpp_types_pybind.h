#pragma once

/// @file
/// Provides a mechanism to map C++ types to canonical Python types.

#include <string>
#include <typeinfo>
#include <vector>

#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/util/type_pack.h"

namespace py = pybind11;

namespace drake {
namespace pydrake {
namespace internal {

// Gets singleton for type aliases from `cpp_types`.
py::object GetTypeAliases();

// Gets Python type object given `std::type_info`.
py::object GetPyTypeImpl(const std::type_info& tinfo);

// Gets Python type for a C++ type (base case).
template <typename T>
inline py::object GetPyTypeImpl(type_pack<T> = {}) {
  return GetPyTypeImpl(typeid(T));
}

// Gets Python literal for a C++ literal (specialization).
template <typename T, T Value>
inline py::object GetPyTypeImpl(
    type_pack<std::integral_constant<T, Value>> = {}) {
  return py::cast(Value);
}

}  // namespace internal

/// Gets the canonical Python type for a given C++ type.
template <typename T>
inline py::object GetPyType(type_pack<T> tag = {}) {
  // Explicitly provide `tag` so that inference can handle the different
  // cases.
  return internal::GetPyTypeImpl(tag);
}

/// Gets the canonical Python types for each C++ type.
template <typename ... Ts>
inline py::tuple GetPyTypes(type_pack<Ts...> = {}) {
  return py::make_tuple(GetPyType<Ts>()...);
}

}  // namespace pydrake
}  // namespace drake
