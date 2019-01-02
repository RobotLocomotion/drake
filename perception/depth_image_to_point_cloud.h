#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

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
/// The system has an input port that takes a depth image and an additional
/// optional input port that takes the camera_pose, X_PC.  If the camera_pose
/// input is connected, then the point cloud is represented in the parent frame
/// (e.g., if camera_pose is the pose of the camera in the world frame, then
/// the point_cloud output will be a PointCloud in the world frame).  If the
/// camera_pose input is not connected, the PointCloud will be represented in
/// the camera frame.
///
/// If a pixel is NaN, the converted point will be (NaN, NaN, NaN).  If a pixel
/// is kTooClose or kTooFar (as defined by ImageTraits), the converted point
/// will be (+Inf, +Inf, +Inf). Note that this matches the convention used by
/// the Point Cloud Library (PCL).
///
/// @ingroup perception_systems
class DepthImageToPointCloud final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthImageToPointCloud)

  /// Constructs the converter.
  ///
  /// @param[in] camera_info The camera info.
  /// @param[in] pixel_type The pixel type of the depth image input.
  ///   Only 16U and 32F are supported.
  /// @param[in] scale The depth image input is multiplied by this scale factor
  ///   before projecting to a point cloud.  (This is useful for converting mm
  ///   to meters, etc.)
  explicit DepthImageToPointCloud(
      const systems::sensors::CameraInfo& camera_info,
      systems::sensors::PixelType pixel_type =
          systems::sensors::PixelType::kDepth32F,
      float scale = 1.0);

  /// Returns the abstract valued input port that expects either an
  /// ImageDepth16U or ImageDepth32F (depending on the constructor argument).
  const systems::InputPort<double>& depth_image_input_port() const {
    return this->get_input_port(depth_image_input_port_);
  }

  /// Returns the abstract valued input port that expects X_PC as a
  /// RigidTransformd.  (This input port does not necessarily need to be
  /// connected; refer to the class overview for details.)
  const systems::InputPort<double>& camera_pose_input_port() const {
    return this->get_input_port(camera_pose_input_port_);
  }

  /// Returns the abstract valued output port that provides a PointCloud.
  /// Only the XYZ channel of the point cloud is present.
  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

  /// Converts a depth image to a point cloud using direct arguments instead of
  /// System input and output ports.  The semantics are the same as documented
  /// in the class overview and constructor.
  ///
  /// @param[in,out] cloud Destination for point data; must not be nullptr.
  /// The `cloud` will be resized to match the size of the depth image.  The
  /// `cloud` must have the XYZ channel enabled.  If other channels such as RGB
  /// are present, their existing data will only be disturbed insofar as we
  /// need to resize the cloud, obeying the the PointCloud::resize() semantics.
  static void Convert(
      const systems::sensors::CameraInfo& camera_info,
      const optional<math::RigidTransformd>& camera_pose,
      const systems::sensors::ImageDepth32F& depth_image,
      const optional<float>& scale,
      PointCloud* cloud);

  /// Converts a depth image to a point cloud using direct arguments instead of
  /// System input and output ports.  The semantics are the same as documented
  /// in the class overview and constructor.
  ///
  /// @param[in,out] cloud Destination for point data; must not be nullptr.
  /// The `cloud` will be resized to match the size of the depth image.  The
  /// `cloud` must have the XYZ channel enabled.  If other channels such as RGB
  /// are present, their existing data will only be disturbed insofar as we
  /// need to resize the cloud, obeying the the PointCloud::resize() semantics.
  static void Convert(
      const systems::sensors::CameraInfo& camera_info,
      const optional<math::RigidTransformd>& camera_pose,
      const systems::sensors::ImageDepth16U& depth_image,
      const optional<float>& scale,
      PointCloud* cloud);


 private:
  void CalcOutput16U(const systems::Context<double>&, PointCloud*) const;
  void CalcOutput32F(const systems::Context<double>&, PointCloud*) const;

  const systems::sensors::CameraInfo camera_info_;
  const systems::sensors::PixelType pixel_type_;
  const float scale_;

  systems::InputPortIndex depth_image_input_port_{};
  systems::InputPortIndex camera_pose_input_port_{};
};

}  // namespace perception
}  // namespace drake
