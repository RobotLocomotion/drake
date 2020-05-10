#pragma once

/// @file
/// Helpers for defining C++ LCM type serializers.

#include <string>
#include <utility>

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

/**
Registers:
- Python binding of the C++ LCM serializer in
`pydrake.systems.lcm._cpp_types.Serializer_`.
- Python binding of the C++ LCM type, with default construction.
- Python binding of the C++ Value<> instantiation.
- Correspondence between the Python LCM type and the C++ LCM type, registered
under `pydrake.systems.lcm._cpp_types.{py_to_cpp,cpp_to_py}.`.

@tparam CppType C++ message type.
@param lcm_package The package that the LCM type belongs to. The C++
  message type name is used to retrieve the relevant Python LCM type.
@see The `use_cpp_serializer` parameter in the factory methods in
`_lcm_extra.py`.
*/
template <typename CppType>
py::object BindCppSerializer(const std::string& lcm_package) {
  using systems::lcm::Serializer;
  using systems::lcm::SerializerInterface;
  py::module m_cpp_types = py::module::import("pydrake.systems.lcm._cpp_types");
  // Retrieve Python type using C++ name.
  // N.B. Since the LCM type does not supply the package, we need it supplied.
  py::object py_cls =
      py::module::import(lcm_package.c_str()).attr(CppType::getTypeName());
  auto serializer_cls =
      DefineTemplateClassWithDefault<Serializer<CppType>, SerializerInterface>(
          m_cpp_types, "Serializer", py::make_tuple(py_cls));
  serializer_cls.def(py::init());
  // Register types for direct conversion.
  // TODO(eric.cousineau): Hoist this to `pydrake.lcm`.
  py::class_<CppType> cpp_cls(m_cpp_types, CppType::getTypeName());
  cpp_cls.def(py::init());
  m_cpp_types.attr("py_to_cpp")[py_cls] = cpp_cls;
  m_cpp_types.attr("cpp_to_py")[cpp_cls] = py_cls;
  // Register Value[] instantiation.
  AddValueInstantiation<CppType>(m_cpp_types);
  // We use move here because the type of py_class differs from our declared
  // return type.
  return std::move(serializer_cls);
}

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
