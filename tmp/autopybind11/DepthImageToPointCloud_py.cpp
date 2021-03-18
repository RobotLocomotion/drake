#include "drake/perception/depth_image_to_point_cloud.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_DepthImageToPointCloud_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::perception;

  py::class_<DepthImageToPointCloud, ::drake::systems::LeafSystem<double>>
      PyDepthImageToPointCloud(m, "DepthImageToPointCloud");

  PyDepthImageToPointCloud
      .def(py::init<::drake::systems::sensors::CameraInfo const &,
                    ::drake::systems::sensors::PixelType, float,
                    pc_flags::BaseFieldT>(),
           py::arg("camera_info"),
           py::arg("depth_pixel_type") = ::drake::systems::sensors::PixelType(
               ::drake::systems::sensors::PixelType::kDepth32F),
           py::arg("scale") = float(1.),
           py::arg("fields") = pc_flags::BaseFieldT(pc_flags::kXYZs))
      .def_static(
          "Convert",
          static_cast<void (*)(
              ::drake::systems::sensors::CameraInfo const &,
              ::std::optional<drake::math::RigidTransform<double>> const &,
              ::drake::systems::sensors::ImageDepth32F const &,
              ::std::optional<drake::systems::sensors::Image<
                  drake::systems::sensors::PixelType::kRgba8U>> const &,
              ::std::optional<float> const &, PointCloud *)>(
              &DepthImageToPointCloud::Convert),
          py::arg("camera_info"), py::arg("camera_pose"),
          py::arg("depth_image"), py::arg("color_image"), py::arg("scale"),
          py::arg("cloud"))
      .def("camera_pose_input_port",
           static_cast<::drake::systems::InputPort<double> const &(
               DepthImageToPointCloud::*)() const>(
               &DepthImageToPointCloud::camera_pose_input_port))
      .def("color_image_input_port",
           static_cast<::drake::systems::InputPort<double> const &(
               DepthImageToPointCloud::*)() const>(
               &DepthImageToPointCloud::color_image_input_port))
      .def("depth_image_input_port",
           static_cast<::drake::systems::InputPort<double> const &(
               DepthImageToPointCloud::*)() const>(
               &DepthImageToPointCloud::depth_image_input_port))
      .def("point_cloud_output_port",
           static_cast<::drake::systems::OutputPort<double> const &(
               DepthImageToPointCloud::*)() const>(
               &DepthImageToPointCloud::point_cloud_output_port))

      ;
}
