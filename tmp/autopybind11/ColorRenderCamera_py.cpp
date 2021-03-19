#include "drake/geometry/render/render_engine.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::geometry::render;

namespace py = pybind11;
void apb11_pydrake_ColorRenderCamera_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<ColorRenderCamera> PyColorRenderCamera(m, "ColorRenderCamera");

  PyColorRenderCamera
      .def(py::init<ColorRenderCamera const &>(), py::arg("arg0"))
      .def(py::init<CameraProperties const &, bool,
                    ::drake::math::RigidTransformd>(),
           py::arg("camera"), py::arg("show_window") = bool(false),
           py::arg("X_BC") = ::drake::math::RigidTransformd({}))
      .def(py::init<RenderCameraCore, bool>(), py::arg("core"),
           py::arg("show_window") = bool(false))
      .def("core",
           static_cast<RenderCameraCore const &(ColorRenderCamera::*)() const>(
               &ColorRenderCamera::core))
      .def("show_window", static_cast<bool (ColorRenderCamera::*)() const>(
                              &ColorRenderCamera::show_window))

      ;
}
