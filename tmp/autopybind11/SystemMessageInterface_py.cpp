#include "drake/systems/framework/framework_common.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_SystemMessageInterface_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::systems::internal;

  py::class_<SystemMessageInterface> PySystemMessageInterface(
      m, "SystemMessageInterface");

  PySystemMessageInterface
      .def(
          "GetSystemName",
          static_cast<::std::string const &(SystemMessageInterface::*)() const>(
              &SystemMessageInterface::GetSystemName))
      .def("GetSystemPathname",
           static_cast<::std::string (SystemMessageInterface::*)() const>(
               &SystemMessageInterface::GetSystemPathname))
      .def("GetSystemType",
           static_cast<::std::string (SystemMessageInterface::*)() const>(
               &SystemMessageInterface::GetSystemType))
      .def("ValidateContext",
           static_cast<void (SystemMessageInterface::*)(
               ::drake::systems::ContextBase const &) const>(
               &SystemMessageInterface::ValidateContext),
           py::arg("context"))
      .def_static("no_name", static_cast<::std::string const &(*)()>(
                                 &SystemMessageInterface::no_name))
      .def_static("path_separator",
                  static_cast<::std::string const &(*)()>(
                      &SystemMessageInterface::path_separator))

      ;
}
