#include "drake/manipulation/sensors/xtion.h"

#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {

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

namespace manipulation {

using util::WorldSimTreeBuilder;

namespace sensors {

// TODO(eric.cousineau): Encode these in an URDF somewhere, where they are
// easily accessible? (and easily interpreted?)
namespace {

/* Transform from RgbdCamera to Xtion base. */
Eigen::Isometry3d get_X_XB() {
  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB;
  X_XB.setIdentity();
  // TODO(eric.cousineau): Once relative transforms between B, C, D can be
  // defined, impose correct constraints on the frame offsets.
  const double kSpacing = 0.001;
  X_XB.translation() << 0.02 + kSpacing, 0, 0.0325;
  return X_XB;
}

}  // namespace

Xtion::Xtion(WorldSimTreeBuilder<double>* tree_builder,
             RigidBody<double>* parent, const Eigen::Isometry3d& X_PX)
    : tree_(&tree_builder->tree()) {
  std::string urdf_resource =
      "drake/manipulation/models/xtion_description/urdf/xtion.urdf";
  tree_builder->StoreModel(name_, urdf_resource);

  fixture_frame_ = std::make_shared<RigidBodyFrame<double>>(name_ + "_fixture",
                                                            parent, X_PX);

  tree_builder->mutable_tree().addFrame(fixture_frame_);
  int xtion_id = tree_builder->AddModelInstanceToFrame(
      name_, fixture_frame_, drake::multibody::joints::kFixed);

  RigidBody<double>* xtion_body =
      tree_builder->mutable_tree().FindBody("base_link", "", xtion_id);

  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB = get_X_XB();
  // Compose frame to get proper orientation for RgbdCamera.
  sensor_frame_ = std::make_shared<RigidBodyFrame<double>>(name_ + "_sensor",
                                                           xtion_body, X_XB);
  tree_builder->mutable_tree().addFrame(sensor_frame_);
}

void Xtion::Build(lcm::DrakeLcm* lcm, bool add_lcm_publisher,
                  bool add_frame_visualizer) {
  DRAKE_DEMAND(tree_);

  DiagramBuilder<double> builder;

  // Declare state input port.
  const int kStateSize =
      tree_->get_num_positions() + tree_->get_num_velocities();

  // @ref https://www.asus.com/us/3D-Sensor/Xtion_PRO_LIVE/specifications/
  const double fov_y = M_PI_4;
  const double depth_range_near = 0.8;
  const double depth_range_far = 3.5;

  auto* camera = builder.AddSystem<RgbdCameraDiscrete>(
      std::make_unique<RgbdCamera>(
          name_, *tree_, *sensor_frame_,
          depth_range_near, depth_range_far,
          fov_y),
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

  if (add_lcm_publisher) {
    DRAKE_DEMAND(lcm != nullptr);
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
    DRAKE_DEMAND(lcm != nullptr);
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

}  // namespace sensors
}  // namespace manipulation
}  // namespace drake
