#pragma once

#include <limits>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace systems {
namespace sensors {
/// The rgbd camera model to provide both rgb and depth images. The image
/// resolution is fixed at VGA (640x480) for both rgb and depth cameras.
/// The depth sensing range is also fixed to be from 0.5m at minimum to 5.0m at
/// maximum. The rgbd camera's base coordinate system is defined to be x-forward
/// , y-left, z-up and the origin of the rgb camera's coordinate system is
/// +0.02m offset in the base coordinate system's y axis.  The depth camera's
/// coordinate system is the same as the one of the rgb camera coordinate
/// system, so user can regard the depth image as "registered depth image" to
/// the rgb image.  No disparity is considered.
///
/// Output image format:
///   - The rgb image has four channel in the order of blue, green, red and
///     alpha, and each channel is represented with uint8_t.
///   - The depth image has a depth channel represented with float.  The value
///     stored in the depth channel holds *the Z value in the camera coordinate
///     system.*  Note that this is different from range data used by laser
///     range finder in which the depth value represents the distance from the
///     sensor origin to the object surface.
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
  /// @param name The name of the rgbd camera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  /// @param tree The RigidBodyTree containing the geometric configuration of
  /// the world.  This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  /// @param position 3D position of RgbdCamera in the world coordinate system.
  /// @param orientation 3D orientation of RgbdCamera by roll-pitch-yaw in the
  /// world coordinate system.
  /// @param frame_rate The frame rate of the RgbdCamera in Hz.
  /// @param fov_y The vertical field of view for RgbdCamera.
  /// @param show_window The flag to show visible window.  If this is false,
  /// offscreen rendering is executed.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             double frame_rate,
             double fov_y,
             bool show_window);

  /// A constructor for %RgbdCamera
  ///
  /// @param name The name of the rgbd camera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  /// @param tree The RigidBodyTree containing the geometric configuration of
  /// the world.  This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  /// @param frame The frame in RigidBodyTree to which this camera is attached.
  /// @param frame_rate The frame rate of the RgbdCamera in Hz.
  /// @param fov_y The vertical field of view for RgbdCamera.
  /// @param show_window The flag to show visible window.  If this is false,
  /// offscreen rendering is executed.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             double frame_rate,
             double fov_y,
             bool show_window);

  ~RgbdCamera();

  /// Reterns the color camera info.
  const CameraInfo& get_color_camera_info() const;

  /// Reterns the depth camera info.
  const CameraInfo& get_depth_camera_info() const;

  /// Returns the camera base pose in the world coordinate system.
  const Eigen::Isometry3d& get_base_pose() const;

  /// Returns the color camera pose in the camera base coordinate system.
  const Eigen::Isometry3d& get_color_camera_pose() const;

  /// Returns the depth camera pose in the camera base coordinate system.
  const Eigen::Isometry3d& get_depth_camera_pose() const;

  /// Returns the frame rate in Hz.
  double get_frame_rate() const { return 1. / frame_interval_; };

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const;

  /// Allocates the output vector. See this class' description for details of
  /// this output vector.
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
    const Context<double>& context) const override;

 protected:
  /// Update all the model frames for renderer and outputs the rendered images.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;

  int input_port_index_{};
  const double frame_interval_{};
  // For the time step calculation in const member function
  mutable double previous_output_time_{0.};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
