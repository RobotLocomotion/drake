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

py::object GetTypeRegistry();

void RegisterTypeImpl(const std::string& py_type_str, size_t cpp_type);

template <typename T>
void RegisterType(const std::string& py_type_str) {
  RegisterTypeImpl(py_type_str, typeid(T).hash_code());
}

py::object GetPyTypeImpl(const std::type_info& tinfo);

template <typename T, typename = void>
struct get_py_type_impl {
  static py::object run() {
    return GetPyTypeImpl(typeid(T));
  }
};

template <typename T, T Value>
struct get_py_type_impl<std::integral_constant<T, Value>> {
  static py::object run() {
    return py::cast(Value);
  }
};

}  // namespace internal

/// Gets the canonical Python type for a given C++ type.
template <typename T>
inline py::object GetPyType(type_pack<T> = {}) {
  return internal::get_py_type_impl<T>::run();
}

/// Gets the canonical Python types for each C++ type.
template <typename ... Ts>
inline py::tuple GetPyTypes(type_pack<Ts...> = {}) {
  return py::make_tuple(GetPyType<Ts>()...);
}

}  // namespace pydrake
}  // namespace drake
