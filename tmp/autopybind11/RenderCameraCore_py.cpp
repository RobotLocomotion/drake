#include "drake/geometry/render/render_engine.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::geometry::render;

namespace py = pybind11;

void apb11_pydrake_RenderCameraCore_py_register(py::module &m) {
  py::class_<RenderCameraCore> PyRenderCameraCore(m, "RenderCameraCore");

  PyRenderCameraCore.def(py::init<RenderCameraCore const &>(), py::arg("arg0"))
      .def(py::init<CameraProperties const &, double,
                    ::drake::math::RigidTransformd>(),
           py::arg("camera"), py::arg("clipping_far"),
           py::arg("X_BS") = ::drake::math::RigidTransformd({}))
      .def(py::init<::std::string, ::drake::systems::sensors::CameraInfo,
                    ClippingRange, ::drake::math::RigidTransformd>(),
           py::arg("renderer_name"), py::arg("intrinsics"), py::arg("clipping"),
           py::arg("X_BS"))
      .def("CalcProjectionMatrix",
           static_cast<::Eigen::Matrix4d (RenderCameraCore::*)() const>(
               &RenderCameraCore::CalcProjectionMatrix))
      .def("clipping",
           static_cast<ClippingRange const &(RenderCameraCore::*)() const>(
               &RenderCameraCore::clipping))
      .def("intrinsics",
           static_cast<::drake::systems::sensors::CameraInfo const &(
               RenderCameraCore::*)() const>(&RenderCameraCore::intrinsics))
      .def("renderer_name",
           static_cast<::std::string const &(RenderCameraCore::*)() const>(
               &RenderCameraCore::renderer_name))
      .def("sensor_pose_in_camera_body",
           static_cast<::drake::math::RigidTransformd const &(
               RenderCameraCore::*)() const>(
               &RenderCameraCore::sensor_pose_in_camera_body))

      ;
}
