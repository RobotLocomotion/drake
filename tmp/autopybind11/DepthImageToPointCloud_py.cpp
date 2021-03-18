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
      PyDepthImageToPointCloud(
          m, "DepthImageToPointCloud",
          R"""(/// Converts a depth image to a point cloud. 
/// 
/// @system 
/// name: DepthImageToPointCloud 
/// input_ports: 
/// - depth_image 
/// - color_image (optional) 
/// - camera_pose (optional) 
/// output_ports: 
/// - point_cloud 
/// @endsystem 
/// 
/// The system has an input port that takes a depth image, an optional input 
/// port that takes a color image, and an additional optional input port that 
/// takes the camera_pose, X_PC.  If the camera_pose input is connected, then 
/// the point cloud is represented in the parent frame (e.g., if camera_pose is 
/// the pose of the camera in the world frame, then the point_cloud output will 
/// be a PointCloud in the world frame).  If the camera_pose input is not 
/// connected, the PointCloud will be represented in the camera frame.  Note 
/// that if a color image is provided, it must be in the same frame as the depth 
/// image. 
/// 
/// If a pixel is NaN, the converted point will be (NaN, NaN, NaN).  If a pixel 
/// is kTooClose or kTooFar (as defined by ImageTraits), the converted point 
/// will be (+Inf, +Inf, +Inf). Note that this matches the convention used by 
/// the Point Cloud Library (PCL). 
/// 
/// @ingroup perception_systems)""");

  PyDepthImageToPointCloud
      .def(py::init<::drake::systems::sensors::CameraInfo const &,
                    ::drake::systems::sensors::PixelType, float,
                    pc_flags::BaseFieldT>(),
           py::arg("camera_info"),
           py::arg("depth_pixel_type") = ::drake::systems::sensors::PixelType(
               ::drake::systems::sensors::PixelType::kDepth32F),
           py::arg("scale") = float(1.),
           py::arg("fields") =
               pc_flags::BaseFieldT(drake::perception::pc_flags::kXYZs))
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
          py::arg("cloud"),
          R"""(/// Converts a depth image to a point cloud using direct arguments instead of 
/// System input and output ports.  The semantics are the same as documented 
/// in the class overview and constructor. 
/// 
/// @param[in,out] cloud Destination for point data; must not be nullptr. 
/// The `cloud` will be resized to match the size of the depth image.  The 
/// `cloud` must have the XYZ channel enabled.)""")
      .def(
          "camera_pose_input_port",
          static_cast<::drake::systems::InputPort<double> const &(
              DepthImageToPointCloud::*)() const>(
              &DepthImageToPointCloud::camera_pose_input_port),
          R"""(/// Returns the abstract valued input port that expects X_PC as a 
/// RigidTransformd.  (This input port does not necessarily need to be 
/// connected; refer to the class overview for details.))""")
      .def(
          "color_image_input_port",
          static_cast<::drake::systems::InputPort<double> const &(
              DepthImageToPointCloud::*)() const>(
              &DepthImageToPointCloud::color_image_input_port),
          R"""(/// Returns the abstract valued input port that expects an ImageRgba8U.)""")
      .def(
          "depth_image_input_port",
          static_cast<::drake::systems::InputPort<double> const &(
              DepthImageToPointCloud::*)() const>(
              &DepthImageToPointCloud::depth_image_input_port),
          R"""(/// Returns the abstract valued input port that expects either an 
/// ImageDepth16U or ImageDepth32F (depending on the constructor argument).)""")
      .def(
          "point_cloud_output_port",
          static_cast<::drake::systems::OutputPort<double> const &(
              DepthImageToPointCloud::*)() const>(
              &DepthImageToPointCloud::point_cloud_output_port),
          R"""(/// Returns the abstract valued output port that provides a PointCloud. 
/// Only the channels passed into the constructor argument "fields" are 
/// present.)""")

      ;
}
