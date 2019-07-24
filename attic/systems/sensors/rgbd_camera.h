#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {
/// An RGB-D camera system that provides RGB, depth and label images using
/// visual elements of RigidBodyTree.
///
/// @warning This is an old interface, with potentially ambiguous and outdated
/// terms. Please see `RgbdSensor` and `CameraInfo` for more modern
/// terminology.
///
/// Let `W` be the world coordinate system. In addition to `W`, there are three
/// more coordinate systems that are associated with an RgbdCamera. They are
/// defined as follows:
///
///   * `B` - the camera's base coordinate system: X-forward, Y-left, and Z-up.
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
///
/// - The RGB image has four channels in the following order: red, green
///   blue, alpha. Each channel is represented by a uint8_t.
///
/// - The depth image has a depth channel represented by a float. The value
///   stored in the depth channel holds *the Z value in `D`.*  Note that this
///   is different from the range data used by laser range finders (like that
///   provided by DepthSensor) in which the depth value represents the
///   distance from the sensor origin to the object's surface.
///
/// - The label image has single channel represented by a int16_t. The value
///   stored in the channel holds a model ID which corresponds to an object
///   in the scene. For the pixels corresponding to no body, namely the sky
///   and the flat terrain, we assign Label::kNoBody and Label::kFlatTerrain,
///   respectively.
///
/// @ingroup sensor_systems
class RgbdCamera final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCamera)

  /// A constructor that uses the default RgbdRendererVTK renderer and
  /// defines `B` using Euler angles.
  /// The pose of %RgbdCamera will be fixed to the world coordinate system
  /// throughout the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance. The maximum number of bodies in the `tree` must be less than
  /// 1536 based on the number of the colors used for the label image, otherwise
  /// an exception will be thrown.
  ///
  /// @param position The x-y-z position of `B` in `W`. This defines the
  /// translation component of `X_WB`.
  ///
  /// @param orientation The roll-pitch-yaw orientation of `B` in `W`. This
  /// defines the orientation component of `X_WB`.
  ///
  /// @param z_near The minimum depth distance RgbdCamera can measure.
  ///
  /// @param z_far The maximum depth distance RgbdCamera can measure.
  ///
  /// @param fov_y The RgbdCamera's vertical field of view.
  ///
  /// @param show_window A flag for showing a visible window.  If this is false,
  /// offscreen rendering is executed. This is useful for debugging purposes.
  ///
  /// @param width Width of output image.
  ///
  /// @param height Height of output image.
  ///
  /// @param flat_terrain A flag to add a flat terrain at z = 0 in the world
  /// coordinate system.
  ///
  /// @throws std::logic_error When the number of rigid bodies in the scene
  /// exceeds the maximum limit 1535.
  // TODO(kunimatsu-tri) Use the information from RigidBodyTree for rendering
  // the flat terrain.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             double z_near = 0.5,
             double z_far = 5.0,
             double fov_y = M_PI_4,
             bool show_window = RenderingConfig::kDefaultShowWindow,
             int width = RenderingConfig::kDefaultWidth,
             int height = RenderingConfig::kDefaultHeight,
             bool flat_terrain = true);

  /// A constructor that uses the default RgbdRendererVTK renderer and
  /// defines `B` using a RigidBodyFrame.
  /// The pose of %RgbdCamera is fixed to a user-defined frame and will be
  /// updated during the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance. The maximum number of bodies in the `tree` must be less than
  /// 1536 based on the number of the colors used for the label image, otherwise
  /// an exception will be thrown.
  ///
  /// @param frame The frame in @tree to which this camera is attached.
  ///
  /// @param z_near The minimum depth distance RgbdCamera can measure.
  ///
  /// @param z_far The maximum depth distance RgbdCamera can measure.
  ///
  /// @param fov_y The RgbdCamera's vertical field of view.
  ///
  /// @param show_window A flag for showing a visible window.  If this is false,
  /// offscreen rendering is executed. This is useful for debugging purposes.
  ///
  /// @param width Width of output image.
  ///
  /// @param height Height of output image.
  ///
  /// @param flat_terrain A flag to add a flat terrain at z = 0 in the world
  /// coordinate system.
  ///
  /// @throws std::logic_error When the number of rigid bodies in the scene
  /// exceeds the maximum limit 1535.
  // TODO(kunimatsu-tri) Use the information from RigidBodyTree for rendering
  // the flat terrain.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             double z_near = 0.5,
             double z_far = 5.0,
             double fov_y = M_PI_4,
             bool show_window = RenderingConfig::kDefaultShowWindow,
             int width = RenderingConfig::kDefaultWidth,
             int height = RenderingConfig::kDefaultHeight,
             bool flat_terrain = true);

  /// A constructor that takes a renderer and defines `B` using Euler angles.
  /// The pose of %RgbdCamera will be fixed to the world coordinate system
  /// throughout the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance. The maximum number of bodies in the `tree` must be less than
  /// 1536 based on the number of the colors used for the label image, otherwise
  /// an exception will be thrown.
  ///
  /// @param position The x-y-z position of `B` in `W`. This defines the
  /// translation component of `X_WB`.
  ///
  /// @param orientation The roll-pitch-yaw orientation of `B` in `W`. This
  /// defines the orientation component of `X_WB`.
  ///
  /// @param renderer The rendering backend to render images for the camera.
  /// Camera configuration parameters are obtained from the RenderingConfig
  /// settings in the renderer. The renderer will be owned by this camera.
  ///
  /// @param flat_terrain A flag to add a flat terrain at z = 0 in the world
  /// coordinate system.
  ///
  /// @throws std::logic_error When the number of rigid bodies in the scene
  /// exceeds the maximum limit 1535.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             std::unique_ptr<RgbdRenderer> renderer,
             bool flat_terrain = true);

  /// A constructor that takes a renderer and defines `B` using a
  /// RigidBodyFrame.
  /// The pose of %RgbdCamera is fixed to a user-defined frame and will be
  /// updated during the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance. The maximum number of bodies in the `tree` must be less than
  /// 1536 based on the number of the colors used for the label image, otherwise
  /// an exception will be thrown.
  ///
  /// @param frame The frame in @tree to which this camera is attached.
  ///
  /// @param renderer The rendering backend to render images for the camera.
  /// Camera configuration parameters are obtained from the RenderingConfig
  /// settings in the renderer. The renderer will be owned by this camera.
  ///
  /// @param flat_terrain A flag to add a flat terrain at z = 0 in the world
  /// coordinate system.
  ///
  /// @throws std::logic_error When the number of rigid bodies in the scene
  /// exceeds the maximum limit 1535.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             std::unique_ptr<RgbdRenderer> renderer,
             bool flat_terrain = true);

  ~RgbdCamera() = default;

  /// Reterns mutable renderer.
  RgbdRenderer& mutable_renderer() { return *renderer_; }

  /// Reterns the color sensor's info.
  const CameraInfo& color_camera_info() const { return color_camera_info_; }

  /// Reterns the depth sensor's info.
  const CameraInfo& depth_camera_info() const { return depth_camera_info_; }

  /// Returns `X_BC`.
  const Eigen::Isometry3d& color_camera_optical_pose() const { return X_BC_; }

  /// Returns `X_BD`.
  const Eigen::Isometry3d& depth_camera_optical_pose() const { return X_BD_; }

  /// Returns the RigidBodyFrame to which this RgbdCamera is attached.
  ///
  /// @throws std::logic_error When RgbdCamera is instantiated as a fixed
  /// camera.
  const RigidBodyFrame<double>& frame() const {
    if (camera_fixed_)
      throw std::logic_error(
          "RgbdCamera::frame(): "
          "A fixed camera does not have an associated frame.");
    return frame_;
  }

  /// Returns the RigidBodyTree to which this RgbdCamera is attached.
  const RigidBodyTree<double>& tree() const { return tree_; }

  /// Returns the vector valued input port that takes a vector of `q, v`
  /// corresponding to the positions and velocities associated with the
  /// RigidBodyTree.
  const InputPort<double>& state_input_port() const;

  /// Returns the abstract valued output port that contains a RGBA image of the
  /// type ImageRgba8U.
  const OutputPort<double>& color_image_output_port() const;

  /// Returns the abstract valued output port that contains an ImageDepth32F.
  const OutputPort<double>& depth_image_output_port() const;

  /// Returns the abstract valued output port that contains an label image of
  /// the type ImageLabel16I.
  const OutputPort<double>& label_image_output_port() const;

  /// Returns the vector valued output port that contains a PoseVector.
  const OutputPort<double>& camera_base_pose_output_port() const;

 private:
  void InitPorts(const std::string& name);

  void InitRenderer();

  // These are the calculator methods for the four output ports.
  void OutputColorImage(const Context<double>& context,
                        ImageRgba8U* color_image) const;
  void OutputDepthImage(const Context<double>& context,
                        ImageDepth32F* depth_image) const;
  void OutputLabelImage(const Context<double>& context,
                        ImageLabel16I* label_image) const;
  void OutputPoseVector(const Context<double>& context,
                        rendering::PoseVector<double>* pose_vector) const;

  // TODO(sherm1) This should be the calculator for a cache entry containing
  // the VTK update that must be valid before outputting any image info. For
  // now it has to be repeated before each image output port calculation.
  void UpdateModelPoses(const BasicVector<double>& input_vector) const;

  const InputPort<double>* state_input_port_{};
  const OutputPort<double>* color_image_port_{};
  const OutputPort<double>* depth_image_port_{};
  const OutputPort<double>* label_image_port_{};
  const OutputPort<double>* camera_base_pose_port_{};

  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double> frame_;
  using VisualIndex = RgbdRenderer::VisualIndex;
  std::map<int, std::vector<VisualIndex>> body_visual_indices_map_;
  const bool camera_fixed_;
  const bool flat_terrain_;
  const CameraInfo color_camera_info_;
  const CameraInfo depth_camera_info_;
  const Eigen::Isometry3d X_WB_initial_;
  // The color sensor's origin (`Co`) is offset by 0.02 m on the Y axis of
  // the RgbdCamera's base coordinate system (`B`).
  // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
  // TODO(kunimatsu-tri) Change the X_BD_ to be different from X_BC_ when
  // it's needed.
  const Eigen::Isometry3d X_BC_{Eigen::Translation3d(0., 0.02, 0.) *
        (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))};
  // The depth sensor's origin (`Do`) is offset by 0.02 m on the Y axis of
  // the RgbdCamera's base coordinate system (`B`).
  const Eigen::Isometry3d X_BD_{Eigen::Translation3d(0., 0.02, 0.) *
        (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))};

  std::unique_ptr<RgbdRenderer> renderer_;
};

/**
 * Wraps a continuous RgbdCamera with zero order holds to have it function as
 * a discrete sensor.
 */
class RgbdCameraDiscrete final : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCameraDiscrete);

  static constexpr double kDefaultPeriod = 1. / 30;

  /// Constructs a diagram containing a (non-registered) RgbdCamera that will
  /// update at a given rate.
  /// @param period
  ///   Update period (sec).
  /// @param render_label_image
  ///   If true, renders label image (which requires additional overhead). If
  ///   false, `label_image_output_port` will raise an error if called.
  RgbdCameraDiscrete(std::unique_ptr<RgbdCamera> camera,
                     double period = kDefaultPeriod,
                     bool render_label_image = true);

  /// Returns reference to RgbdCamera intsance.
  const RgbdCamera& camera() const { return *camera_; }

  /// Returns reference to RgbdCamera intsance.
  RgbdCamera& mutable_camera() { return *camera_; }

  /// Returns update period for discrete camera.
  double period() const { return period_; }

  /// @see RgbdCamera::state_input_port().
  const InputPort<double>& state_input_port() const {
    return get_input_port(input_port_state_);
  }

  /// @see RgbdCamera::color_image_output_port().
  const systems::OutputPort<double>& color_image_output_port() const {
    return get_output_port(output_port_color_image_);
  }

  /// @see RgbdCamera::depth_image_output_port().
  const systems::OutputPort<double>& depth_image_output_port() const {
    return get_output_port(output_port_depth_image_);
  }

  /// @see RgbdCamera::label_image_output_port().
  const systems::OutputPort<double>& label_image_output_port() const {
    return get_output_port(output_port_label_image_);
  }

  /// @see RgbdCamera::camera_base_pose_output_port().
  const systems::OutputPort<double>& camera_base_pose_output_port() const {
    return get_output_port(output_port_pose_);
  }

 private:
  RgbdCamera* const camera_{};
  const double period_{};

  int input_port_state_{-1};
  int output_port_color_image_{-1};
  int output_port_depth_image_{-1};
  int output_port_label_image_{-1};
  int output_port_pose_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
