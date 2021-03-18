#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_internal_py_register(py::module &m) {

  py::module Pyinternal = m.def_submodule("internal", "");

  return Pyinternal;
}
