#pragma once

/// @file
/// Helpers for defining C++ LCM type serializers.

#include <string>
#include <utility>

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

/// Provides a Python binding of C++ LCM type serializers. This registers the
/// binding using the Python LCM type in `pydrake.systems.lcm.CppSerializer`,
/// so that the type is easily indexed.
/// @tparam CppType C++ message type.
/// @param lcm_package The package that the LCM type belongs to. The C++
///   message type name is used to retrieve the relevant Python LCM type.
/// @see The `use_cpp_serializer` parameter in the factory methods in
/// `_lcm_extra.py`.
template <typename CppType>
py::object BindCppSerializer(const std::string& lcm_package) {
  using systems::lcm::Serializer;
  using systems::lcm::SerializerInterface;
  // Retrieve Python type using C++ name.
  // N.B. Since the LCM type does not supply the package, we need it supplied.
  py::object py_type =
      py::module::import(lcm_package.c_str()).attr(CppType::getTypeName());
  py::module lcm_py = py::module::import("pydrake.systems.lcm");
  auto py_cls =
      DefineTemplateClassWithDefault<Serializer<CppType>, SerializerInterface>(
          lcm_py, "_Serializer", py::make_tuple(py_type));
  py_cls.def(py::init());
  // We use move here because the type of py_class differs from our declared
  // return type.
  return std::move(py_cls);
}

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
