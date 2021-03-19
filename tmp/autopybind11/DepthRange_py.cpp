#include "drake/geometry/render/render_engine.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::geometry::render;

namespace py = pybind11;
void apb11_pydrake_DepthRange_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<DepthRange> PyDepthRange(m, "DepthRange");

  PyDepthRange.def(py::init<DepthRange const &>(), py::arg("arg0"))
      .def(py::init<double, double>(), py::arg("min_in"), py::arg("max_in"))
      .def("max_depth",
           static_cast<double (DepthRange::*)() const>(&DepthRange::max_depth))
      .def("min_depth",
           static_cast<double (DepthRange::*)() const>(&DepthRange::min_depth))

      ;
}
