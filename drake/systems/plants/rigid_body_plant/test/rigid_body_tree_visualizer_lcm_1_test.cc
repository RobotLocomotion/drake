#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"
// NOLINT(whitespace/line_length)
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_visualizer_lcm_1.h"
#include "drake/systems/plants/shapes/Geometry.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;

using DrakeShapes::Sphere;

// Tests the basic functionality of the RigidBodyTreeVisualizerLcm1.
GTEST_TEST(RigidBodyTreeVisualizerLcm1Tests, BasicTest) {
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

    // Creates a sphere with a radius of 0.54 meters.
    Sphere sphere_shape(0.54);

    // Specifies the color of the sphere
    Eigen::Vector4d material;
    material << 0.3, 0.4, 0.5, 1.0;

    // Sets the sphere as the visual representation of the body.
    DrakeShapes::VisualElement visual_element(
        sphere_shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    // Connects the body to the world using a floating joint.
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

  // Creates an RigidBodyTreeVisualizerLcm1 object using the tree and LCM that
  // was just created. The name "dut" stands for "Device Under Test".
  RigidBodyTreeVisualizerLcm1 dut(*tree.get(), &lcm);

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

  // TODO(liang.fok): Verify that the correct messages were actually
  // transmitted. This will involve calling `dut.get_load_message()` and
  // `dut.get_draw_message_bytes()`, and verifying that the returned values
  // are correct.
}

}  // namespace
}  // namespace systems
}  // namespace drake
