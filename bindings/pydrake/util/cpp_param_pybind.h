#pragma once

/// @file
/// Provides a mechanism to map C++ types to canonical Python types.

#include <string>
#include <typeinfo>
#include <vector>

#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/util/type_pack.h"

namespace drake {
namespace pydrake {
namespace internal {

// Gets singleton for type aliases from `cpp_param`.
pybind11::object GetParamAliases();

// Gets Python type object given `std::type_info`.
// @throws std::runtime_error if type is neither aliased nor registered in
// `pybind11`.
pybind11::object GetPyParamScalarImpl(const std::type_info& tinfo);

// Gets Python type for a C++ type (base case).
template <typename T>
inline pybind11::object GetPyParamScalarImpl(type_pack<T> = {}) {
  return GetPyParamScalarImpl(typeid(T));
}

// Gets Python literal for a C++ literal (specialization).
template <typename T, T Value>
inline pybind11::object GetPyParamScalarImpl(
    type_pack<std::integral_constant<T, Value>> = {}) {
  return pybind11::cast(Value);
}

}  // namespace internal

/// Gets the canonical Python parameters for each C++ type.
/// @returns Python tuple of canonical parameters.
/// @throws std::runtime_error on the first type it encounters that is neither
/// aliased nor registered in `pybind11`.
template <typename ... Ts>
inline pybind11::tuple GetPyParam(type_pack<Ts...> = {}) {
  return pybind11::make_tuple(
      internal::GetPyParamScalarImpl(type_pack<Ts>{})...);
}

}  // namespace pydrake
}  // namespace drake
