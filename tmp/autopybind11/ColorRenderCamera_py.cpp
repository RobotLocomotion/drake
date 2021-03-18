#include "drake/geometry/render/render_engine.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_ColorRenderCamera_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::geometry::render;

  py::class_<ColorRenderCamera> PyColorRenderCamera(
      m, "ColorRenderCamera",
      R"""(/** Collection of camera properties for cameras to be used with color/label 
 images.  */)""");

  PyColorRenderCamera
      .def(py::init<ColorRenderCamera const &>(), py::arg("arg0"))
      .def(py::init<CameraProperties const &, bool,
                    ::drake::math::RigidTransformd>(),
           py::arg("camera"), py::arg("show_window") = bool(false),
           py::arg("X_BC") = ::drake::math::RigidTransformd({}))
      .def(py::init<RenderCameraCore, bool>(), py::arg("core"),
           py::arg("show_window") = bool(false))
      .def_static(
          "DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE",
          static_cast<void (*)()>(
              &ColorRenderCamera::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
      .def("core",
           static_cast<RenderCameraCore const &(ColorRenderCamera::*)() const>(
               &ColorRenderCamera::core),
           R"""(/** This camera's core render properties.  */)""")
      .def(
          "show_window",
          static_cast<bool (ColorRenderCamera::*)() const>(
              &ColorRenderCamera::show_window),
          R"""(/** If true, requests that the RenderEngine display the rendered image.  */)""")

      ;
}
