#include "drake/geometry/render/render_engine.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_DepthRenderCamera_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::geometry::render;

  py::class_<DepthRenderCamera> PyDepthRenderCamera(
      m, "DepthRenderCamera",
      R"""(/** Collection of camera properties for cameras to be used with depth images. 
 */)""");

  PyDepthRenderCamera
      .def(py::init<DepthRenderCamera const &>(), py::arg("arg0"))
      .def(py::init<DepthCameraProperties const &,
                    ::drake::math::RigidTransformd>(),
           py::arg("camera"),
           py::arg("X_BD") = ::drake::math::RigidTransformd({}))
      .def(py::init<RenderCameraCore, DepthRange>(), py::arg("core"),
           py::arg("depth_range"))
      .def_static(
          "DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE",
          static_cast<void (*)()>(
              &DepthRenderCamera::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
      .def("core",
           static_cast<RenderCameraCore const &(DepthRenderCamera::*)() const>(
               &DepthRenderCamera::core),
           R"""(/** This camera's core render properties.  */)""")
      .def("depth_range",
           static_cast<DepthRange const &(DepthRenderCamera::*)() const>(
               &DepthRenderCamera::depth_range),
           R"""(/** The range of valid values for the depth camera.  */)""")

      ;
}
