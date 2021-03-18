#include "drake/common/constants.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_drake_py_register(py::module &m) {
  using namespace drake;

  py::module Pydrake = m.def_submodule("drake", "");
  py::enum_<ToleranceType>(Pydrake, "ToleranceType", py::arithmetic())
      .value("kAbsolute", ToleranceType::kAbsolute, "")
      .value("kRelative", ToleranceType::kRelative, "");
  return Pydrake;
}
