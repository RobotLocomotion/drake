#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/shapes/geometry.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::unique_ptr;

using DrakeShapes::Box;
using DrakeShapes::Capsule;
using DrakeShapes::Cylinder;
using DrakeShapes::Mesh;
using DrakeShapes::Sphere;

// Verifies that @p message is correct.
void VerifyLoadMessage(const std::vector<uint8_t>& message_bytes) {
  // Instantiates the expected message.
  drake::lcmt_viewer_load_robot expected_message;
  expected_message.num_links = 6;

  // Adds the world body.
  {
    lcmt_viewer_link_data link_data;
    link_data.name = "world";
    link_data.robot_num = 0;
    link_data.num_geom = 0;
    expected_message.link.push_back(link_data);
  }

  // Adds the box body.
  {
    lcmt_viewer_geometry_data geometry_data;
    geometry_data.type = geometry_data.BOX;
    geometry_data.position[0] = 0;
    geometry_data.position[1] = 0;
    geometry_data.position[2] = 0;
    geometry_data.quaternion[0] = 1;
    geometry_data.quaternion[1] = 0;
    geometry_data.quaternion[2] = 0;
    geometry_data.quaternion[3] = 0;
    geometry_data.color[0] = 0.3;
    geometry_data.color[1] = 0.4;
    geometry_data.color[2] = 0.5;
    geometry_data.color[3] = 1.0;
    geometry_data.num_float_data = 3;
    geometry_data.float_data.push_back(0.5);
    geometry_data.float_data.push_back(0.5);
    geometry_data.float_data.push_back(0.5);

    lcmt_viewer_link_data link_data;
    link_data.name = "box_body";
    link_data.robot_num = 0;
    link_data.num_geom = 1;
    link_data.geom.push_back(geometry_data);

    expected_message.link.push_back(link_data);
  }

  // Adds the capsule body.
  {
    lcmt_viewer_geometry_data geometry_data;
    geometry_data.type = geometry_data.CAPSULE;
    geometry_data.position[0] = 0;
    geometry_data.position[1] = 0;
    geometry_data.position[2] = 0;
    geometry_data.quaternion[0] = 1;
    geometry_data.quaternion[1] = 0;
    geometry_data.quaternion[2] = 0;
    geometry_data.quaternion[3] = 0;
    geometry_data.color[0] = 0.7;
    geometry_data.color[1] = 0.7;
    geometry_data.color[2] = 0.7;
    geometry_data.color[3] = 1.0;
    geometry_data.num_float_data = 2;
    geometry_data.float_data.push_back(0.1);
    geometry_data.float_data.push_back(0.5);

    lcmt_viewer_link_data link_data;
    link_data.name = "capsule_body";
    link_data.robot_num = 1;
    link_data.num_geom = 1;
    link_data.geom.push_back(geometry_data);

    expected_message.link.push_back(link_data);
  }

  // Adds the cylinder body.
  {
    lcmt_viewer_geometry_data geometry_data;
    geometry_data.type = geometry_data.CYLINDER;
    geometry_data.position[0] = 0;
    geometry_data.position[1] = 0;
    geometry_data.position[2] = 0;
    geometry_data.quaternion[0] = 1;
    geometry_data.quaternion[1] = 0;
    geometry_data.quaternion[2] = 0;
    geometry_data.quaternion[3] = 0;
    geometry_data.color[0] = 0.9;
    geometry_data.color[1] = 0.0;
    geometry_data.color[2] = 0.7;
    geometry_data.color[3] = 1.0;
    geometry_data.num_float_data = 2;
    geometry_data.float_data.push_back(0.2);
    geometry_data.float_data.push_back(0.25);

    lcmt_viewer_link_data link_data;
    link_data.name = "cylinder_body";
    link_data.robot_num = 2;
    link_data.num_geom = 1;
    link_data.geom.push_back(geometry_data);

    expected_message.link.push_back(link_data);
  }

  // Adds the mesh body.
  {
    lcmt_viewer_geometry_data geometry_data;
    geometry_data.type = geometry_data.MESH;
    geometry_data.position[0] = 0;
    geometry_data.position[1] = 0;
    geometry_data.position[2] = 0;
    geometry_data.quaternion[0] = 1;
    geometry_data.quaternion[1] = 0;
    geometry_data.quaternion[2] = 0;
    geometry_data.quaternion[3] = 0;
    geometry_data.color[0] = 0.2;
    geometry_data.color[1] = 0.7;
    geometry_data.color[2] = 0.3;
    geometry_data.color[3] = 1.0;
    geometry_data.string_data = FindResourceOrThrow(
        "drake/multibody/collision/test/spherical_cap.obj");
    geometry_data.num_float_data = 3;
    geometry_data.float_data.push_back(1);
    geometry_data.float_data.push_back(1);
    geometry_data.float_data.push_back(1);

    lcmt_viewer_link_data link_data;
    link_data.name = "mesh_body";
    link_data.robot_num = 3;
    link_data.num_geom = 1;
    link_data.geom.push_back(geometry_data);

    expected_message.link.push_back(link_data);
  }

  // Adds the sphere body.
  {
    lcmt_viewer_geometry_data geometry_data;
    geometry_data.type = geometry_data.SPHERE;
    geometry_data.position[0] = 0;
    geometry_data.position[1] = 0;
    geometry_data.position[2] = 0;
    geometry_data.quaternion[0] = 1;
    geometry_data.quaternion[1] = 0;
    geometry_data.quaternion[2] = 0;
    geometry_data.quaternion[3] = 0;
    geometry_data.color[0] = 0.8;
    geometry_data.color[1] = 0.7;
    geometry_data.color[2] = 0.6;
    geometry_data.color[3] = 1.0;
    geometry_data.num_float_data = 1;
    geometry_data.float_data.push_back(0.54);

    lcmt_viewer_link_data link_data;
    link_data.name = "sphere_body";
    link_data.robot_num = 4;
    link_data.num_geom = 1;
    link_data.geom.push_back(geometry_data);

    expected_message.link.push_back(link_data);
  }

  // Ensures both messages have the same length.
  EXPECT_EQ(expected_message.getEncodedSize(),
      static_cast<int>(message_bytes.size()));
  const int byte_count = expected_message.getEncodedSize();

  // Serialize the expected message.
  std::vector<uint8_t> expected_message_bytes(byte_count);
  expected_message.encode(expected_message_bytes.data(), 0, byte_count);

  // Verifies that the messages are equal.
  EXPECT_EQ(expected_message_bytes, message_bytes);
}

// Verifies that @p message_bytes is correct.
void VerifyDrawMessage(const std::vector<uint8_t>& message_bytes) {
  // TODO(liang.fok): Replace the following two lines with
  // `Eigen::Quaterniond::Identity()` and a method in lcmUtil.h that converts it
  // into a std::vector<float>. Related issue: #3470.
  const std::vector<float> zero_position = {0, 0, 0};
  const std::vector<float> zero_quaternion = {1, 0, 0, 0};

  // Instantiates a `drake::lcmt_viewer_draw` message that contains the expected
  // state.
  drake::lcmt_viewer_draw expected_message;
  expected_message.timestamp = 0;
  expected_message.num_links = 6;

  // Adds the world body.
  {
    expected_message.link_name.push_back("world");
    expected_message.robot_num.push_back(0);
    expected_message.position.push_back(zero_position);
    expected_message.quaternion.push_back(zero_quaternion);
  }

  // Adds the box body.
  {
    std::vector<float> position = zero_position;
    position[0] = 1;

    expected_message.link_name.push_back("box_body");
    expected_message.robot_num.push_back(0);
    expected_message.position.push_back(position);
    expected_message.quaternion.push_back(zero_quaternion);
  }

  // Adds the capsule body.
  {
    std::vector<float> position = zero_position;
    position[0] = 2;

    expected_message.link_name.push_back("capsule_body");
    expected_message.robot_num.push_back(1);
    expected_message.position.push_back(position);
    expected_message.quaternion.push_back(zero_quaternion);
  }

  // Adds the cylinder body.
  {
    std::vector<float> position = zero_position;
    position[0] = -1;

    expected_message.link_name.push_back("cylinder_body");
    expected_message.robot_num.push_back(2);
    expected_message.position.push_back(position);
    expected_message.quaternion.push_back(zero_quaternion);
  }

  // Adds the mesh body.
  {
    std::vector<float> position = zero_position;
    position[1] = -2;

    expected_message.link_name.push_back("mesh_body");
    expected_message.robot_num.push_back(3);
    expected_message.position.push_back(position);
    expected_message.quaternion.push_back(zero_quaternion);
  }

  // Adds the sphere body.
  {
    expected_message.link_name.push_back("sphere_body");
    expected_message.robot_num.push_back(4);
    expected_message.position.push_back(zero_position);
    expected_message.quaternion.push_back(zero_quaternion);
  }

  // Ensures both messages have the same length.
  const int byte_count = expected_message.getEncodedSize();
  EXPECT_EQ(byte_count, static_cast<int>(message_bytes.size()));

  // Serializes the expected message.
  std::vector<uint8_t> expected_message_bytes(byte_count);
  expected_message.encode(expected_message_bytes.data(), 0, byte_count);

  // Verifies that the messages are equal.
  EXPECT_EQ(expected_message_bytes, message_bytes);
}

// Creates a RigidBodyTree. The tree has 6 rigid bodies including the world. The
// visualizations of the rigid bodies span all possible visualization types.
// Each non-world body belongs to a different model instance.
unique_ptr<RigidBodyTree<double>> CreateRigidBodyTree() {
  auto tree = make_unique<RigidBodyTree<double>>();

  // Adds a RigidBody that looks like a box to the tree to achieve some level
  // of unit test coverage for the box geometry.
  {
    auto body = make_unique<RigidBody<double>>();
    body->set_name("box_body");
    body->set_model_instance_id(tree->add_model_instance());
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const Eigen::Vector3d box_size(0.5, 0.5, 0.5);
    const Box shape(box_size);
    const Eigen::Vector4d material(0.3, 0.4, 0.5, 1.0);

    const DrakeShapes::VisualElement visual_element(
        shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    Eigen::Isometry3d joint_transform;
    {
      const Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
      const Eigen::Vector3d xyz(1, 0, 0);
      joint_transform.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
    }

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "box_joint", joint_transform);
    body->add_joint(&tree->world(), std::move(joint));

    tree->bodies.push_back(std::move(body));
  }

  // Adds a RigidBody that looks like a capsule to the tree to achieve some
  // level of unit test coverage for the box geometry.
  {
    auto body = make_unique<RigidBody<double>>();
    body->set_name("capsule_body");
    body->set_model_instance_id(tree->add_model_instance());
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const Capsule shape(0.1, 0.5);
    const Eigen::Vector4d material(0.7, 0.7, 0.7, 1.0);

    const DrakeShapes::VisualElement visual_element(
        shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    Eigen::Isometry3d joint_transform;
    {
      const Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
      const Eigen::Vector3d xyz(2, 0, 0);
      joint_transform.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
    }

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "capsule_joint", joint_transform);
    body->add_joint(&tree->world(), std::move(joint));

    tree->bodies.push_back(std::move(body));
  }

  // Adds a RigidBody that looks like a cylinder to the tree to achieve some
  // level of unit test coverage for the cylinder geometry.
  {
    auto body = make_unique<RigidBody<double>>();
    body->set_name("cylinder_body");
    body->set_model_instance_id(tree->add_model_instance());
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const Cylinder shape(0.2, 0.25);
    const Eigen::Vector4d material(0.9, 0.0, 0.7, 1.0);

    const DrakeShapes::VisualElement visual_element(
        shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    Eigen::Isometry3d joint_transform;
    {
      Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
      Eigen::Vector3d xyz(-1, 0, 0);
      joint_transform.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
    }

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "cylinder_joint", joint_transform);
    body->add_joint(&tree->world(), std::move(joint));

    tree->bodies.push_back(std::move(body));
  }

  // Adds a RigidBody that looks like a mesh to the tree to achieve some
  // level of unit test coverage for the mesh geometry. The mesh is specified
  // by an OBJ file.
  {
    auto body = make_unique<RigidBody<double>>();
    body->set_name("mesh_body");
    body->set_model_instance_id(tree->add_model_instance());
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const Mesh shape("spherical_cap.obj", FindResourceOrThrow(
        "drake/multibody/collision/test/spherical_cap.obj"));
    const Eigen::Vector4d material(0.2, 0.7, 0.3, 1.0);

    const DrakeShapes::VisualElement visual_element(
        shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    Eigen::Isometry3d joint_transform;
    {
      const Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
      const Eigen::Vector3d xyz(0, -2, 0);
      joint_transform.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
    }

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "mesh_joint", joint_transform);
    body->add_joint(&tree->world(), std::move(joint));

    tree->bodies.push_back(std::move(body));
  }

  // Adds a RigidBody that looks like a sphere to the tree to achieve some
  // level of unit test coverage for the sphere geometry.
  {
    auto body = make_unique<RigidBody<double>>();
    body->set_name("sphere_body");
    body->set_model_instance_id(tree->add_model_instance());
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const Sphere shape(0.54);  // The sphere has a radius of 0.54 meters.
    const Eigen::Vector4d material(0.8, 0.7, 0.6, 1.0);

    const DrakeShapes::VisualElement visual_element(
        shape, Eigen::Isometry3d::Identity(), material);

    body->AddVisualElement(visual_element);

    Eigen::Isometry3d joint_transform;
    {
      const Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
      const Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
      joint_transform.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
    }

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "sphere_joint", joint_transform);
    body->add_joint(&tree->world(), std::move(joint));
    tree->bodies.push_back(std::move(body));
  }

  tree->compile();

  return tree;
}

// Helper function to publish load robot model message.
void PublishLoadRobotModelMessageHelper(
    const DrakeVisualizer& dut, Context<double>* context) {
  std::unique_ptr<State<double>> tmp_state = context->CloneState();
  dut.CalcDiscreteVariableUpdates(*context,
      tmp_state->get_mutable_discrete_state());
  context->get_mutable_state()->CopyFrom(*tmp_state);
}

// Tests the basic functionality of the DrakeVisualizer.
GTEST_TEST(DrakeVisualizerTests, BasicTest) {
  unique_ptr<RigidBodyTree<double>> tree = CreateRigidBodyTree();
  drake::lcm::DrakeMockLcm lcm;
  const DrakeVisualizer dut(*tree, &lcm);

  EXPECT_EQ("drake_visualizer", dut.get_name());

  auto context = dut.CreateDefaultContext();

  EXPECT_EQ(1, context->get_num_input_ports());

  // Initializes the system's input vector to contain all zeros.
  const int vector_size =
      tree->get_num_positions() + tree->get_num_velocities();
  auto input_data = make_unique<BasicVector<double>>(vector_size);
  input_data->set_value(Eigen::VectorXd::Zero(vector_size));

  context->SetInputPortValue(
      0, std::make_unique<systems::FreestandingInputPortValue>(
             std::move(input_data)));

  // Publishes the `RigidBodyTree` visualization messages.
  PublishLoadRobotModelMessageHelper(dut, context.get());
  dut.Publish(*context.get());

  // Verifies that the correct messages were actually transmitted.
  VerifyLoadMessage(lcm.get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"));
  VerifyDrawMessage(lcm.get_last_published_message("DRAKE_VIEWER_DRAW"));
}

// Tests that the published LCM message has the expected timestamps.
GTEST_TEST(DrakeVisualizerTests, TestPublishPeriod) {
  const double kPublishPeriod = 1.5;  // Seconds between publications.

  unique_ptr<RigidBodyTree<double>> tree = CreateRigidBodyTree();
  drake::lcm::DrakeMockLcm lcm;

  // Instantiates the "device under test".
  DrakeVisualizer dut(*tree, &lcm);
  dut.set_publish_period(kPublishPeriod);
  unique_ptr<Context<double>> context = dut.AllocateContext();

  const int kPortNumber = 0;
  const int num_inputs = tree->get_num_positions() + tree->get_num_velocities();
  context->FixInputPort(kPortNumber,
      make_unique<BasicVector<double>>(Eigen::VectorXd::Zero(num_inputs)));

  // Prepares to integrate.
  drake::systems::Simulator<double> simulator(dut, std::move(context));
  simulator.set_publish_every_time_step(false);
  PublishLoadRobotModelMessageHelper(dut, simulator.get_mutable_context());
  simulator.Initialize();

  for (double time = 0; time < 4; time += 0.01) {
    simulator.StepTo(time);
    EXPECT_NEAR(simulator.get_mutable_context()->get_time(), time, 1e-10);
    // Note that the expected time is in milliseconds.
    const double expected_time =
        std::floor(time / kPublishPeriod) * kPublishPeriod * 1000;
    EXPECT_EQ(lcm.DecodeLastPublishedMessageAs<lcmt_viewer_draw>(
        "DRAKE_VIEWER_DRAW").timestamp, expected_time);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
