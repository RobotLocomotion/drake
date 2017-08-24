#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Defines frames for the Xtion mounted on the WSG gripper on the IIWA.
 *
 * Frames:
 *  B - RgbdCamera frame
 *  C - RgbdCamera color frame
 *  D - RgbdCamera depth frame
 *  X - Xtion frame (X-left, Y-up, Z-forward)
 *     To visualize base pose, run `meshlab asus_xtion.obj`, go to
 *     Render > Show Axis.
 *  G - WSG Gripper frame.
 *  Wb - Base-world frame (X-forward, Y-left, Z-up).
 */
// TODO(eric.cousineau): Encode these in an URDF somewhere, where they are
// easily accessible? (and easily interpreted?)
class IiwaCameraFrames {
 public:
  /** Transform from Xtion to Gripper */
  static Eigen::Isometry3d X_GX();

  /** Orientation of Xtion in the base-world frame */
  static Eigen::Matrix3d R_WbX();

  /** Pose of RgbdCamera frame w.r.t. Xtion frame. */
  static Eigen::Isometry3d X_XB();
};

/**
 * Attaches an Asus Xtion PRO camera to a given frame on a RigidBodyTree
 * scene, and provides the ability to add the camera as a system in a diagram
 * builder.
 *
 * Convenience provided for the IIWA + WSG setup.
 *
 * Example:
 * @code{.cc}
 *  // Add to tree.
 *  auto* camera = new IiwaCamera(&tree_builder, wsg_id);
 *  // Build diagram.
 *  camera->Build(&lcm, true, true);
 *  // Add to system.
 *  builder.AddSystem(std::unique_ptr<IiwaCamera>(camera));
 *  builder.Connect(plant->get_output_port_state(),
 *                  camera->get_input_port_state());
 * @endcode
 */
class IiwaCamera : public systems::Diagram<double> {
 public:
  typedef manipulation::util::WorldSimTreeBuilder<double> TreeBuilder;

  /// Attach camera to a given body, with a given transform.
  IiwaCamera(TreeBuilder* tree_builder,
             RigidBody<double>* parent,
             const Eigen::Isometry3d& X_PX);

  /// Convenience for WSG.
  IiwaCamera(TreeBuilder* tree_builder,
             int wsg_id = -1)
    : IiwaCamera(tree_builder,
                 tree_builder->mutable_tree().FindBody("body", "", wsg_id),
                 IiwaCameraFrames::X_GX()) {}

  // Convenience for world-frame.
  IiwaCamera(TreeBuilder* tree_builder,
             const Eigen::Isometry3d& X_WX)
    : IiwaCamera(tree_builder,
                 &tree_builder->mutable_tree().world(),
                 X_WX) {}

  void Build(lcm::DrakeLcm *lcm,
             bool do_lcm_publish = true,
             bool add_frame_visualizer = false);

  using Input = systems::InputPortDescriptor<double>;
  using Output = systems::OutputPort<double>;

  const Input& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const Output& get_output_port_color_image() const {
    return get_output_port(output_port_color_image_);
  }
  const Output& get_output_port_depth_image() const {
    return get_output_port(output_port_depth_image_);
  }
  const Output& get_output_port_label_image() const {
    return get_output_port(output_port_label_image_);
  }

  /// @return Period at which the camera captures images.
  double period() const { return period_; }
  void set_period(double period) { period_ = period; }

  /// @return If camera is attached to the world frame.
  bool is_movable() const;

  // TODO(eric.cousineau): Add option for noise.

 private:
  std::string name_{"xtion"};
  std::string lcm_channel_{"DRAKE_RGBD_CAMERA_IMAGES"};
  double period_{1. / 30};

  const RigidBodyTree<double>* tree_{};
  std::shared_ptr<RigidBodyFrame<double>> fixture_frame_;
  std::shared_ptr<RigidBodyFrame<double>> sensor_frame_;

  // Ports.
  int input_port_state_{-1};
  int output_port_color_image_{-1};
  int output_port_depth_image_{-1};
  int output_port_label_image_{-1};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
