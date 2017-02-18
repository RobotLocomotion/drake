#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace systems {
namespace sensors {
/// The RGBD camera model provides both RGB and depth images. The image
/// resolution is fixed at VGA resolution (640x480) for both RGB and depth
/// cameras. The depth sensing range is 0.5 m to 5.0 m. The RGBD camera's base
/// coordinate system is defined to be x-forward, y-left, and z-up. The origin
/// of the RGB camera's optical coordinate system is +0.02 m offset in the base
/// coordinate system's y axis.  The depth camera's optical coordinate system is
/// the same as the RGB camera's optical coordinate system, so the user can
/// regard the depth image to be a "registered depth image" for the RGB image.
/// No disparity is considered.  For the camera optical coordinate systems's
/// orientation, refer to CameraInfo's documentation.
///
/// Output image format:
///   - The RGB image has four channels in the following order: blue, green
///     red, alpha. Each channel is represented by a uint8_t.
///   - The depth image has a depth channel represented by a float. The value
///     stored in the depth channel holds *the Z value in the camera optical
///     coordinate system.*  Note that this is different from the range data
///     used by laser range finders in which the depth value represents the
///     distance from the sensor origin to the object surface.
///
// TODO(kunimatsu-tri) Add support for the image publish capability.
class RgbdCamera : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCamera)

  /// Set of constants used to represent invalid depth values.
  class InvalidDepth {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InvalidDepth)
    /// The depth value when the depth is not measureable.
    static constexpr float kError{std::numeric_limits<float>::quiet_NaN()};

    /// The depth value when the max sensing range is exceeded.
    static constexpr float kTooFar{std::numeric_limits<float>::infinity()};

    /// The depth value when the min sensing range is violated because the
    /// object being sensed is too close. Note that this
    /// <a href="http://www.ros.org/reps/rep-0117.html">differs from ROS</a>,
    /// which uses negative infinity in this scenario. Drake uses zero because
    /// it results in less devastating bugs when users fail to check for the
    /// lower limit being hit and using negative infinity does not prevent users
    /// from writing bad code.
    static constexpr float kTooClose{0.f};
  };

  /// A constructor for %RgbdCamera
  ///
  /// @param name The name of the RGBD camera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  /// @param tree The RigidBodyTree containing the geometric configuration of
  /// the world.  This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  /// @param position 3D position of RgbdCamera in the world coordinate system.
  /// @param orientation 3D orientation of RgbdCamera by roll-pitch-yaw in the
  /// world coordinate system.
  /// @param fov_y The vertical field of view for RgbdCamera.
  /// @param show_window The flag to show visible window.  If this is false,
  /// offscreen rendering is executed.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             double fov_y,
             bool show_window);

  /// A constructor for %RgbdCamera
  ///
  /// @param name The name of the RGBD camera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  /// @param tree The RigidBodyTree containing the geometric configuration of
  /// the world.  This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  /// @param frame The frame in RigidBodyTree to which this camera is attached.
  /// @param fov_y The vertical field of view for RgbdCamera.
  /// @param show_window The flag to show visible window.  If this is false,
  /// offscreen rendering is executed.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             double fov_y,
             bool show_window);

  ~RgbdCamera();

  /// Reterns the color camera info.
  const CameraInfo& color_camera_info() const;

  /// Reterns the depth camera info.
  const CameraInfo& depth_camera_info() const;

  /// Returns the camera base pose in the world coordinate system.
  const Eigen::Isometry3d& base_pose() const;

  /// Returns the pose of color camera's optical coordinate system in the world
  /// coordinate system.
  const Eigen::Isometry3d& color_camera_optical_pose() const;

  /// Returns the pose of depth camera's optical coordinate system in the world
  /// coordinate system.
  const Eigen::Isometry3d& depth_camera_optical_pose() const;

  /// Returns the RigidBodyTree within the RigidBodyPlant that this sensor is
  /// sensing.
  const RigidBodyTree<double>& tree() const;

  /// Allocates the output vector. See this class's description for details of
  /// this output vector.
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
    const Context<double>& context) const override;

 protected:
  /// Updates all the model frames for the renderer and outputs the rendered
  /// images.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;

  int input_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
