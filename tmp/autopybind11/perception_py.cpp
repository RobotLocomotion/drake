#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_perception_py_register(py::module &m) {

  py::module Pyperception = m.def_submodule("perception", "");

  return Pyperception;
}
