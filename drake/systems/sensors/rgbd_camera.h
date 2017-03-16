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
/// The RgbdCamera provides both RGB and depth images. Its image resolution is
/// fixed at VGA (640 x 480 pixels) for both the RGB and depth measurements. The
/// depth sensing range is 0.5 m to 5.0 m.
///
/// Let `W` be the world coordinate system. In addition to `W`, there are three
/// more coordinate systems that are associated with an RgbdCamera. They are
/// defined as follows:
///
///   * `B` - The camera's base coordinate system: X-forward, Y-left, and Z-up.
///
///   * `C` - the camera's color sensor's optical coordinate system: `X-right`,
///           `Y-down` and `Z-forward`.
///
///   * `D` - the camera's depth sensor's optical coordinate system: `X-right`,
///           `Y-down` and `Z-forward`.
///
/// The origins of `C` and `D` (i.e., `Co` and `Do`, respectively) are offset
/// from `B`'s origin (`Bo`) by 0 m in `B`'s X-axis, +0.02 m in `B`'s Y-axis,
/// and 0 m in `B`'s Z axis.  Since `C` and `D` are coincident, the depth image
/// is a "registered depth image" for the RGB image. No disparity between the
/// RGB and depth images are modeled. For more details about the poses of `C`
/// and `D`, see the class documentation of CameraInfo.
///
/// Output image format:
///   - The RGB image has four channels in the following order: blue, green
///     red, alpha. Each channel is represented by a uint8_t.
///
///   - The depth image has a depth channel represented by a float. The value
///     stored in the depth channel holds *the Z value in `D`.*  Note that this
///     is different from the range data used by laser range finders (like that
///     provided by DepthSensor) in which the depth value represents the
///     distance from the sensor origin to the object's surface.
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

  /// A constructor for %RgbdCamera that defines `B` using Euler angles.
  /// The pose of %RgbdCamera will be fixed to the world coordinate system
  /// throughout the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance.
  ///
  /// @param position The x-y-z position of `B` in `W`. This defines the
  /// translation component of `X_WB`.
  ///
  /// @param orientation The roll-pitch-yaw orientation of `B` in `W`. This
  /// defines the orientation component of `X_WB`.
  ///
  /// @param fov_y The RgdbCamera's vertical field of view.
  ///
  /// @param show_window A flag for showing a visible window.  If this is false,
  /// offscreen rendering is executed. This is useful for debugging purposes.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             double fov_y,
             bool show_window);

  /// A constructor for %RgbdCamera that defines `B` using a RigidBodyFrame.
  /// The pose of %RgbdCamera is fixed to a user-defined frame and will be
  /// updated during the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance.
  ///
  /// @param frame The frame in @tree to which this camera is attached.
  ///
  /// @param fov_y The RgdbCamera's vertical field of view.
  ///
  /// @param show_window A flag for showing a visible window.  If this is false,
  /// offscreen rendering is executed. This is useful for debugging purposes.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             double fov_y,
             bool show_window);

  ~RgbdCamera();

  /// Reterns the color sensor's info.
  const CameraInfo& color_camera_info() const;

  /// Reterns the depth sensor's info.
  const CameraInfo& depth_camera_info() const;

  /// Returns `X_BC`.
  const Eigen::Isometry3d& color_camera_optical_pose() const;

  /// Returns `X_BD`.
  const Eigen::Isometry3d& depth_camera_optical_pose() const;

  /// Returns the RigidBodyFrame to which this RgbdCamera is attached.
  const RigidBodyFrame<double>& frame() const;

  /// Returns the RigidBodyTree to which this RgbdCamera is attached.
  const RigidBodyTree<double>& tree() const;

  /// Returns a descriptor of the vector valued input port that takes a vector
  /// of `q, v` corresponding to the positions and velocities associated with
  /// the RigidBodyTree.
  const InputPortDescriptor<double>& state_input_port() const;

  /// Returns a descriptor of the abstract valued output port that contains an
  /// Image<uint_8>.
  const OutputPortDescriptor<double>& color_image_output_port() const;

  /// Returns a descriptor of the abstract valued output port that contains an
  /// Image<float>.
  const OutputPortDescriptor<double>& depth_image_output_port() const;

  /// Returns a descriptor of the vector valued output port that contains an
  /// PoseVector.
  const OutputPortDescriptor<double>& camera_base_pose_output_port() const;

 protected:
  /// Allocates the outputs.  See class description.
  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<double>& descriptor) const override;

  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const OutputPortDescriptor<double>& descriptor) const override;

  /// Updates all the model frames for the renderer and outputs the rendered
  /// images.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  void Init(const std::string& name);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
