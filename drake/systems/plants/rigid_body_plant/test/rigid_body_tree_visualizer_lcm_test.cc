#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"
// NOLINT(whitespace/line_length)
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_visualizer_lcm.h"
#include "drake/systems/plants/shapes/Geometry.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;

using DrakeShapes::Sphere;

void VerifyLoadMessage(const RigidBodyTree& tree,
    const drake::lcmt_viewer_load_robot& load_message) {
  // Instantiates a `drake::lcmt_viewer_load_robot` message that contains the
  // expected state.
  drake::lcmt_viewer_load_robot expected_load_message;
  expected_load_message.num_links = tree.get_number_of_bodies();
  for (int i = 0; i < expected_load_message.num_links; ++i) {
    const RigidBody& body = tree.get_body(i);

    lcmt_viewer_link_data link_message;
    link_message.name = body.get_name();
    link_message.robot_num = body.get_model_instance_id();

    // Unlike the rest of the bodies in the RigidBodyTree, the world body has no
    // geometry. Thus, treat it as a special case.
    if (i == RigidBodyTree::kWorldBodyIndex) {
      link_message.num_geom = 0;
    } else {
      link_message.num_geom = 1;

      lcmt_viewer_geometry_data geom_message;
      geom_message.type = lcmt_viewer_geometry_data::SPHERE;
      geom_message.position[0] = 0;
      geom_message.position[1] = 0;
      geom_message.position[2] = 0;
      geom_message.quaternion[0] = 1;
      geom_message.quaternion[1] = 0;
      geom_message.quaternion[2] = 0;
      geom_message.quaternion[3] = 0;
      geom_message.color[0] = 0.3;
      geom_message.color[1] = 0.4;
      geom_message.color[2] = 0.5;
      geom_message.color[3] = 1.0;
      geom_message.num_float_data = 1;
      geom_message.float_data.push_back(0.54);

      link_message.geom.push_back(geom_message);
    }
    expected_load_message.link.push_back(link_message);
  }

  // Ensures the serialized versions of both the load message and expected load
  // message have the same length.
  EXPECT_EQ(expected_load_message.getEncodedSize(),
            load_message.getEncodedSize());
  int load_message_byte_count = expected_load_message.getEncodedSize();

  // Serializes both the load message and expected load message. Then verifies
  // that both byte arrays are identical.
  std::vector<uint8_t> expected_load_message_bytes;
  expected_load_message_bytes.resize(load_message_byte_count);
  expected_load_message.encode(expected_load_message_bytes.data(), 0,
                               load_message_byte_count);

  std::vector<uint8_t> load_message_bytes;
  load_message_bytes.resize(load_message_byte_count);
  load_message.encode(load_message_bytes.data(), 0,
                               load_message_byte_count);

  bool load_message_bytes_match = true;
  for (int i = 0; i < load_message_byte_count && load_message_bytes_match;
      ++i) {
    load_message_bytes_match =
        (load_message_bytes[i] == expected_load_message_bytes[i]);
  }

  EXPECT_TRUE(load_message_bytes_match);
}

void VerifyDrawMessage(const RigidBodyTree& tree,
                       const Context<double>& context,
                       const std::vector<uint8_t>& draw_message_bytes) {
  std::vector<float> quaternion_zero_rotation;
  quaternion_zero_rotation.resize(4);
  quaternion_zero_rotation[0] = 1;
  quaternion_zero_rotation[1] = 0;
  quaternion_zero_rotation[2] = 0;
  quaternion_zero_rotation[3] = 0;

  // Instantiates a `drake::lcmt_viewer_draw` message that contains the expected
  // state.
  drake::lcmt_viewer_draw expected_draw_message;
  expected_draw_message.timestamp = context.get_time();
  expected_draw_message.num_links = tree.get_number_of_bodies();
  for (int i = 0; i < expected_draw_message.num_links; ++i) {
    const RigidBody& body = tree.get_body(i);

    expected_draw_message.link_name.push_back(body.get_name());
    expected_draw_message.robot_num.push_back(body.get_model_instance_id());
    std::vector<float> position;
    position.resize(3);

    // Unlike the rest of the bodies in the RigidBodyTree, the world body's
    // position is not a function of `i`. Thus, treat it as a special case.
    if (i == RigidBodyTree::kWorldBodyIndex) {
      position[0] = 0;
      position[1] = 0;
      position[2] = 0;
    } else {
      position[0] = i - 1;
      position[1] = 0;
      position[2] = 0;
    }

    expected_draw_message.position.push_back(position);
    expected_draw_message.quaternion.push_back(quaternion_zero_rotation);
  }

  // Ensures the serialized versions of both the draw message and expected draw
  // message have the same length.
  EXPECT_EQ(expected_draw_message.getEncodedSize(), draw_message_bytes.size());
  int draw_message_byte_count = expected_draw_message.getEncodedSize();

  // Serializes expected draw message. Then verifies
  // that both byte arrays are identical.
  std::vector<uint8_t> expected_draw_message_bytes;
  expected_draw_message_bytes.resize(draw_message_byte_count);
  expected_draw_message.encode(expected_draw_message_bytes.data(), 0,
                               draw_message_byte_count);

  bool draw_message_bytes_match = true;
  for (int i = 0; i < draw_message_byte_count && draw_message_bytes_match;
      ++i) {
    draw_message_bytes_match =
        (draw_message_bytes[i] == expected_draw_message_bytes[i]);
  }

  EXPECT_TRUE(draw_message_bytes_match);
}

// Tests the basic functionality of the RigidBodyTreeVisualizerLcm.
GTEST_TEST(RigidBodyTreeVisualizerLcmTests, BasicTest) {
  // Define the number of rigid bodies in the RigidbodyTree.
  const int kNumBodies = 5;

  // Creates a RigidBodyTree with kNumBodies rigid bodies. Each rigid body is
  // a sphere and belongs to a different model instance. The X coordinate of
  // each sphere in the world frame is 1, 2, ...
  auto tree = make_unique<RigidBodyTree>();
  for (int i = 0; i < kNumBodies; ++i) {
    // Instantiates a new body to add to the tree.
    auto body = make_unique<RigidBody>();

    // Sets the body's name and model instance ID.
    body->set_name("body" + std::to_string(i));
    body->set_model_instance_id(tree->add_model_instance());

    // The inertia model must be set to prevent RigidBodyTree::compile() from
    // replacing the body's joint with a FixedJoint.
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    // Creates a sphere with a radius of 0.54 meters.
    Sphere sphere_shape(0.54);

    // Specifies the color of the sphere
    Eigen::Vector4d material;
    material << 0.3, 0.4, 0.5, 1.0;

    // Sets the sphere as the visual representation of the body.
    DrakeShapes::VisualElement visual_element(
        sphere_shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    // Connects the body to the world using a RPY floating joint.
    Eigen::Isometry3d joint_transform;
    {
      Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                      xyz = Eigen::Vector3d::Zero();
      xyz(0) = i;
      joint_transform.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
    }

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "Joint" + std::to_string(i), joint_transform);
    body->add_joint(&tree->world(), std::move(joint));

    // Adds the body to the tree.
    tree->bodies.push_back(std::move(body));
  }
  tree->compile();

  // Instantiates the LCM subsystem.
  ::lcm::LCM lcm;

  // Creates a RigidBodyTreeVisualizerLcm object using the previously created
  // RigidBodyTree. The name "dut" stands for "Device Under Test".
  RigidBodyTreeVisualizerLcm dut(*tree.get(), &lcm);

  // Verifies that the name of the system is correct.
  EXPECT_EQ(dut.get_name(), "rigid_body_tree_visualizer_lcm_1");

  auto context = dut.CreateDefaultContext();

  // Sets the time to be zero. This is necessary since the load robot message
  // is only transmitted when the time is zero.
  // TODO(liang.fok) Remove this assumption once System::Initialize() exists.
  context->set_time(0);

  // Verifies that the `dut` system has the correct number of input ports.
  EXPECT_EQ(context->get_num_input_ports(), 1);

  // Initializes the input vector to contain all zeros.
  int vector_size = tree->number_of_positions() + tree->number_of_velocities();
  auto input_data = make_unique<BasicVector<double>>(vector_size);

  for (int i = 0; i < vector_size; ++i) {
    input_data->SetAtIndex(i, 0);
  }

  // Saves the input vector into the context's input port.
  context->SetInputPort(0,
      std::make_unique<systems::FreestandingInputPort>(std::move(input_data)));

  // Make the `dut` system publish the `RigidBodyTree` visualization messages.
  dut.Publish(*context.get());

  // Verifies that the correct messages were actually transmitted.
  VerifyLoadMessage(*tree.get(), dut.get_load_message());
  VerifyDrawMessage(*tree.get(), *context.get(), dut.get_draw_message_bytes());
}

}  // namespace
}  // namespace systems
}  // namespace drake
