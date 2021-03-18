#include "drake/perception/point_cloud_flags.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_pc_flags_py_register(py::module &m) {
  using namespace drake::perception::pc_flags;

  py::module Pypc_flags = m.def_submodule("pc_flags", "");
  py::enum_<BaseField>(Pypc_flags, "BaseField", py::arithmetic())
      .value("kInherit", BaseField::kInherit, "")
      .value("kNone", BaseField::kNone, "")
      .value("kNormals", BaseField::kNormals, "")
      .value("kRGBs", BaseField::kRGBs, "")
      .value("kXYZs", BaseField::kXYZs, "");
  return Pypc_flags;
}
