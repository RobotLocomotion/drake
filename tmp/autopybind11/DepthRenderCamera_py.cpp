#include "drake/geometry/render/render_engine.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::geometry::render;

namespace py = pybind11;
void apb11_pydrake_DepthRenderCamera_py_register(py::module &m) {
  py::class_<DepthRenderCamera> PyDepthRenderCamera(m, "DepthRenderCamera");

  PyDepthRenderCamera
      .def(py::init<DepthRenderCamera const &>(), py::arg("arg0"))
      .def(py::init<DepthCameraProperties const &,
                    ::drake::math::RigidTransformd>(),
           py::arg("camera"),
           py::arg("X_BD") = ::drake::math::RigidTransformd({}))
      .def(py::init<RenderCameraCore, DepthRange>(), py::arg("core"),
           py::arg("depth_range"))
      .def("core",
           static_cast<RenderCameraCore const &(DepthRenderCamera::*)() const>(
               &DepthRenderCamera::core))
      .def("depth_range",
           static_cast<DepthRange const &(DepthRenderCamera::*)() const>(
               &DepthRenderCamera::depth_range))

      ;
}
