#include "drake/geometry/render/render_engine.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_ClippingRange_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::geometry::render;

  py::class_<ClippingRange> PyClippingRange(
      m, "ClippingRange",
      R"""(/** Defines the near and far clipping planes for frustum-based (e.g. OpenGL) 
 RenderEngine cameras.  */)""");

  PyClippingRange.def(py::init<ClippingRange const &>(), py::arg("arg0"))
      .def(py::init<double, double>(), py::arg("near"), py::arg("far"))
      .def_static("DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE",
                  static_cast<void (*)()>(
                      &ClippingRange::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
      .def("far",
           static_cast<double (ClippingRange::*)() const>(&ClippingRange::far))
      .def("near",
           static_cast<double (ClippingRange::*)() const>(&ClippingRange::near))

      ;
}
