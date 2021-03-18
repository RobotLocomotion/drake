#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_math_py_register(py::module &m) {

  py::module Pymath = m.def_submodule("math", "");

  return Pymath;
}
