#include "drake/examples/kuka_iiwa_arm/iiwa_camera.h"

#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using systems::DiagramBuilder;
using systems::FrameVisualizer;
using systems::PassThrough;
using systems::Value;
using systems::ZeroOrderHold;
using systems::lcm::LcmPublisherSystem;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageToLcmImageArrayT;
using systems::sensors::RgbdCamera;
using systems::sensors::RgbdCameraDiscrete;

Eigen::Isometry3d IiwaCameraFrames::X_GX() {
  // Attach Xtion (X) to wsg's end effector (G).
  Eigen::Isometry3d X_GX;
  X_GX.setIdentity();
  // Based on tri_exp:27a7d76:book_pulling.cc (Jiaji's code), but changed to
  // align with present model frames.
  // clang-format off
  X_GX.linear() <<
      0, 1, 0,
      1, 0, 0,
      0, 0, -1;
  // clang-format on
  X_GX.translation() << 0, -0.015, -0.025;
  return X_GX;
}

Eigen::Isometry3d IiwaCameraFrames::X_XB() {
  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB;
  X_XB.setIdentity();
  // TODO(eric.cousineau): Once relative transforms between B, C, D can be
  // defined, impose correct constraints on the frame offsets.
  const double kSpacing = 0.001;
  X_XB.translation() << 0.02 + kSpacing, 0, 0.0325;
  return X_XB;
}

bool IiwaCamera::is_movable() const {
  return &fixture_frame_->get_rigid_body() == &tree_->world();
}

IiwaCamera::IiwaCamera(TreeBuilder* tree_builder, RigidBody<double>* parent,
                       const Eigen::Isometry3d& X_PX)
    : tree_(&tree_builder->tree()) {
  std::string urdf_resource =
      "drake/examples/kuka_iiwa_arm/models/cameras/asus_xtion.urdf";
  tree_builder->StoreModel(
      name_, urdf_resource);

  fixture_frame_ =
      std::make_shared<RigidBodyFrame<double>>(
          name_ + "_fixture",
          parent, X_PX);

  tree_builder->mutable_tree().addFrame(fixture_frame_);
  int xtion_id = tree_builder->AddModelInstanceToFrame(
      name_, fixture_frame_,
      drake::multibody::joints::kFixed);

  RigidBody<double>* xtion_body =
      tree_builder->mutable_tree().FindBody("base_link", "", xtion_id);

  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB = IiwaCameraFrames::X_XB();
  // Compose frame to get proper orientation for RgbdCamera.
  sensor_frame_ =
      std::make_shared<RigidBodyFrame<double>>(
          name_ + "_sensor",
          xtion_body,
          X_XB);
  tree_builder->mutable_tree().addFrame(sensor_frame_);
}

void IiwaCamera::Build(lcm::DrakeLcm* lcm, bool do_lcm_publish,
                       bool add_frame_visualizer) {
  DRAKE_DEMAND(tree_);

  DiagramBuilder<double> builder;

  // Declare state input port.
  const int kStateSize =
      tree_->get_num_positions() + tree_->get_num_velocities();

  auto* camera = builder.AddSystem<RgbdCameraDiscrete>(
      std::make_unique<RgbdCamera>(name_, *tree_, *sensor_frame_, M_PI_4, true),
      period_);

  // TODO(eric.cousineau): See if there is a better way to connect an input
  // to other blocks.
  auto* state_pass_through = builder.AddSystem<PassThrough>(kStateSize);
  input_port_state_ = builder.ExportInput(state_pass_through->get_input_port());

  builder.Connect(state_pass_through->get_output_port(),
                  camera->state_input_port());

  output_port_color_image_ =
      builder.ExportOutput(camera->color_image_output_port());
  output_port_depth_image_ =
      builder.ExportOutput(camera->depth_image_output_port());
  output_port_label_image_ =
      builder.ExportOutput(camera->label_image_output_port());

  if (do_lcm_publish) {
    // Image to LCM.
    auto* image_to_lcm_message =
        builder.AddSystem<ImageToLcmImageArrayT>("color", "depth", "label");
    image_to_lcm_message->set_name(name_ + "_lcm_converter");

    builder.Connect(camera->color_image_output_port(),
                    image_to_lcm_message->color_image_input_port());

    builder.Connect(camera->depth_image_output_port(),
                    image_to_lcm_message->depth_image_input_port());

    builder.Connect(camera->label_image_output_port(),
                    image_to_lcm_message->label_image_input_port());

    // Camera image publisher.
    auto* image_lcm_pub = builder.AddSystem(
        LcmPublisherSystem::Make<robotlocomotion::image_array_t>(lcm_channel_,
                                                                 lcm));
    image_lcm_pub->set_name(name_ + "_lcm_publisher");
    image_lcm_pub->set_publish_period(period_);

    builder.Connect(image_to_lcm_message->image_array_t_msg_output_port(),
                    image_lcm_pub->get_input_port(0));
  }

  if (add_frame_visualizer) {
    // TODO(eric.cousineau): Consider refactoring FrameVisualizer to have a
    // name / prefix to allow multiple visualizers to be shown.
    std::vector<RigidBodyFrame<double>> frames =
        {*fixture_frame_, *sensor_frame_};
    auto* frame_viz =
        builder.AddSystem<FrameVisualizer>(
            tree_, frames, lcm);
    builder.Connect(
        state_pass_through->get_output_port(),
        frame_viz->get_input_port(0));
  }

  builder.BuildInto(this);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
