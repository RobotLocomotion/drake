#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_symbolic_py_register(py::module &m) {

  py::module Pysymbolic = m.def_submodule("symbolic", "");

  return Pysymbolic;
}
