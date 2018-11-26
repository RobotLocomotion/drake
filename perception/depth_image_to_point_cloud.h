#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace perception {

/// Converts a depth image to a point cloud.
///
/// @system{ DepthImageToPointCloud,
///          @input_port{depth_image}
///          @input_port{camera_pose (optional)},
///          @output_port{point_cloud}
/// }
///
/// The system has an input port that takes a
/// systems::sensors::ImageDepth32F and an additional optional input port
/// that takes a math::RigidTransform for the camera_pose.  If this port is
/// connected, then the point cloud is represented in the parent frame (e.g.
/// if camera_pose is the pose of the camera in the world frame, then the
/// point_cloud output will be a PointCloud in the world frame).  If
/// the camera_pose input is not connected, the PointCloud will be represented
/// in the camera frame.
///
/// @ingroup perception_systems
class DepthImageToPointCloud final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthImageToPointCloud)

  /// Constructs the converter.
  ///
  /// @param[in] camera_info The camera info.
  explicit DepthImageToPointCloud(
      const systems::sensors::CameraInfo& camera_info);

  /// Returns the abstract valued input port that contains an ImageDepth32F.
  const systems::InputPort<double>& depth_image_input_port() const {
    return this->get_input_port(depth_image_input_port_);
  }

  /// Returns the abstract valued input port that contains an Isometry3d.
  const systems::InputPort<double>& camera_pose_input_port() const {
    return this->get_input_port(camera_pose_input_port_);
  }

  /// Returns the abstract valued output port that contains a PointCloud.
  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

  /// Converts a depth image obtained from RgbdCamera to a point cloud.  If a
  /// pixel in the depth image has NaN depth value, all the `(x, y, z)` values
  /// in the converted point will be NaN.
  /// Similarly, if a pixel has either InvalidDepth::kTooFar or
  /// InvalidDepth::kTooClose, the converted point will be
  /// InvalidDepth::kTooFar. Note that this matches the convention used by the
  /// Point Cloud Library (PCL).
  ///
  /// @param[in] depth_image The input depth image obtained from RgbdCamera.
  ///
  /// @param[in] camera_info The input camera info which is used for conversion.
  ///
  /// @param[out] point_cloud A pointer to a valid, non nullptr, point
  /// cloud of Eigen Matrix3Xf type.
  // TODO(kunimatsu-tri) Use drake::perception::PointCloud instead of
  // Eigen::Matrix3Xf and create new constants there instead of reusing
  // InvalidDepth.
  static void Convert(const systems::sensors::ImageDepth32F& depth_image,
                      const systems::sensors::CameraInfo& camera_info,
                      Eigen::Matrix3Xf* point_cloud);

 private:
  /// Returns an empty point cloud.
  PointCloud MakeOutputPointCloud() const;

  /// Converts the input depth image to a point cloud.
  void ConvertDepthImageToPointCloud(const systems::Context<double>& context,
                                     PointCloud* output) const;

  systems::InputPortIndex depth_image_input_port_{};
  systems::InputPortIndex camera_pose_input_port_{};

  const systems::sensors::CameraInfo& camera_info_;
};

}  // namespace perception
}  // namespace drake
