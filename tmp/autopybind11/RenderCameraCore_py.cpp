#include "drake/geometry/render/render_engine.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_RenderCameraCore_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::geometry::render;

  py::class_<RenderCameraCore> PyRenderCameraCore(
      m, "RenderCameraCore",
      R"""(/** Collection of core parameters for modeling a pinhole-model camera in a 
 RenderEngine. These parameters are applicable to both depth and color/label 
 renderings. Parameters specific to those output image types can be found 
 below. 
 
 While these parameters are generally applicable to all RenderEngine 
 implementations, this is not guaranteed to be true. For example, the clipping 
 range property only applies to frustum-based RenderEngine implementations. 
 I.e., it wouldn't apply to a ray-tracing based implementation.  */)""");

  PyRenderCameraCore.def(py::init<RenderCameraCore const &>(), py::arg("arg0"))
      .def(py::init<CameraProperties const &, double,
                    ::drake::math::RigidTransformd>(),
           py::arg("camera"), py::arg("clipping_far"),
           py::arg("X_BS") = ::drake::math::RigidTransformd({}))
      .def(py::init<::std::string, ::drake::systems::sensors::CameraInfo,
                    ClippingRange, ::drake::math::RigidTransformd>(),
           py::arg("renderer_name"), py::arg("intrinsics"), py::arg("clipping"),
           py::arg("X_BS"))
      .def(
          "CalcProjectionMatrix",
          static_cast<::Eigen::Matrix4d (RenderCameraCore::*)() const>(
              &RenderCameraCore::CalcProjectionMatrix),
          R"""(/** Expresses `this` camera's pinhole camera properties as the projective 
 transform T_DC which transforms points in a camera's frame C to a 2D, 
 normalized device frame D. The transform is represented by a 4x4 matrix 
 (i.e., a 
 <a href="https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/"> 
 classic OpenGl projection matrix</a>).  */)""")
      .def_static(
          "DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE",
          static_cast<void (*)()>(
              &RenderCameraCore::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
      .def(
          "clipping",
          static_cast<ClippingRange const &(RenderCameraCore::*)() const>(
              &RenderCameraCore::clipping),
          R"""(/** The near and far clipping planes for this camera. This property is ignored 
 by RenderEngine implementations that don't use a clipping frustum.  */)""")
      .def(
          "intrinsics",
          static_cast<::drake::systems::sensors::CameraInfo const &(
              RenderCameraCore::*)() const>(&RenderCameraCore::intrinsics),
          R"""(/** The camera's intrinsic properties (e.g., focal length, sensor size, etc.) 
 See systems::sensors::CameraInfo for details.  */)""")
      .def(
          "renderer_name",
          static_cast<::std::string const &(RenderCameraCore::*)() const>(
              &RenderCameraCore::renderer_name),
          R"""(/** The name of the render engine this camera should be used with.  */)""")
      .def(
          "sensor_pose_in_camera_body",
          static_cast<::drake::math::RigidTransformd const &(
              RenderCameraCore::*)() const>(
              &RenderCameraCore::sensor_pose_in_camera_body),
          R"""(/** The pose of the sensor frame (S) in the camera's body frame (B). This is 
 the "imager" referred to in systems::sensors::CameraInfo's documentation.  */)""")

      ;
}
