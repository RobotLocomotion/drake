#include "drake/examples/kuka_iiwa_arm/iiwa_camera.h"

#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_camera.h"

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

Eigen::Isometry3d IiwaCameraFrames::X_GX() {
  // Attach Xtion (X) to wsg's end effector (G).
  Eigen::Isometry3d X_GX;
  // Based on tri_exp:27a7d76:book_pulling.cc (Jiaji's code), but changed to
  // align with present model frames.
  // clang-format off
  X_GX.linear() <<
      1, 0, 0,
      0, 0, 1,
      0, -1, 0;
  // clang-format on
  X_GX.translation() << 0, -0.015, -0.025;
  return X_GX;
}

Eigen::Matrix3d IiwaCameraFrames::R_WbX() {
  Eigen::Matrix3d R_WbX;
  // clang-format off
  R_WbX <<
      0, 0, 1,
      1, 0, 0,
      0, 1, 0;
  // clang-format on
  return R_WbX;
}

Eigen::Isometry3d IiwaCameraFrames::X_XB() {
  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB;
  X_XB.setIdentity();
  // clang-format off
  X_XB.linear() <<
      0, 1, 0,
      0, 0, 1,
      1, 0, 0;
  // clang-format on
  // TODO(eric.cousineau): Once relative transforms between B, C, D can be
  // defined, impose correct constraints on the frame offsets.
  X_XB.translation() << 0.0, 0.0325, 0.021;
  return X_XB;
}

bool IiwaCamera::is_movable() const {
  return &fixture_frame_->get_rigid_body() == &tree_->world();
}

IiwaCamera::IiwaCamera(TreeBuilder* tree_builder, RigidBody<double>* parent,
                       const Eigen::Isometry3d& X_PX)
    : tree_(&tree_builder->tree()) {
  tree_builder->StoreModel(
      "xtion", "drake/examples/kuka_iiwa_arm/models/cameras/asus_xtion.urdf");

  fixture_frame_ =
      std::make_shared<RigidBodyFrame<double>>(
          name_ + "_fixture",
          parent, X_PX);

  tree_builder->mutable_tree().addFrame(fixture_frame_);
  tree_builder->AddModelInstanceToFrame(
      name_, fixture_frame_,
      drake::multibody::joints::kFixed);

  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB = IiwaCameraFrames::X_XB();
  // Compose frame to get proper orientation for RgbdCamera.
  // TODO(eric.cousineau): Add this frame directly to the object.
  sensor_frame_ =
      std::make_shared<RigidBodyFrame<double>>(
          name_ + "_sensor",
          parent,
          X_PX * X_XB);
  tree_builder->mutable_tree().addFrame(sensor_frame_);
}

void IiwaCamera::Build(lcm::DrakeLcm* lcm, bool do_lcm_publish,
                       bool add_frame_visualizer) {
  DRAKE_DEMAND(tree_);

  DiagramBuilder<double> builder;

  // Declare state input port.
  const int kStateSize =
      tree_->get_num_positions() + tree_->get_num_velocities();

  auto* rgbd_camera = builder.AddSystem<RgbdCamera>(
      name_, *tree_, *sensor_frame_, M_PI_4, true);

  // TODO(eric.cousineau): See if there is a better way to connect an input
  // to other blocks.
  auto* state_pass_through = builder.AddSystem<PassThrough>(kStateSize);
  input_port_state_ = builder.ExportInput(state_pass_through->get_input_port());

  builder.Connect(state_pass_through->get_output_port(),
                  rgbd_camera->state_input_port());

  const int kWidth = 640, kHeight = 480;

  Value<ImageRgba8U> image_rgb(kWidth, kHeight);
  auto* zoh_rgb = builder.template AddSystem<ZeroOrderHold>(period_, image_rgb);
  builder.Connect(rgbd_camera->color_image_output_port(),
                  zoh_rgb->get_input_port());
  output_port_color_image_ = builder.ExportOutput(zoh_rgb->get_output_port());

  Value<ImageDepth32F> image_depth(kWidth, kHeight);
  auto* zoh_depth =
      builder.template AddSystem<ZeroOrderHold>(period_, image_depth);
  builder.Connect(rgbd_camera->depth_image_output_port(),
                  zoh_depth->get_input_port());
  output_port_color_image_ = builder.ExportOutput(zoh_depth->get_output_port());

  Value<ImageLabel16I> image_label(kWidth, kHeight);
  auto* zoh_label =
      builder.template AddSystem<ZeroOrderHold>(period_, image_label);
  builder.Connect(rgbd_camera->label_image_output_port(),
                  zoh_label->get_input_port());
  output_port_label_image_ = builder.ExportOutput(zoh_label->get_output_port());

  if (do_lcm_publish) {
    // Image to LCM.
    auto* image_to_lcm_message =
        builder.AddSystem<ImageToLcmImageArrayT>("color", "depth", "label");
    image_to_lcm_message->set_name(name_ + "_lcm_converter");

    builder.Connect(zoh_rgb->get_output_port(),
                    image_to_lcm_message->color_image_input_port());

    builder.Connect(zoh_depth->get_output_port(),
                    image_to_lcm_message->depth_image_input_port());

    builder.Connect(zoh_label->get_output_port(),
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
