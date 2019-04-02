/* clang-format off to disable clang-format-includes */
#include "drake/examples/valkyrie/robot_state_encoder.h"
#include "drake/examples/valkyrie/robot_state_decoder.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::move;
using std::make_pair;
using std::map;

using bot_core::robot_state_t;

using multibody::joints::FloatingBaseType;

// Transforms a spatial force expressed in the sensor frame to an equivalent
// one in a frame that is world aligned frame located at the sensor position.
static ContactForce<double> TransformContactWrench(
    const RigidBodyFrame<double>& sensor_info,
    const Isometry3<double>& body_pose,
    const SpatialForce<double>& spatial_force_in_sensor_frame) {
  Isometry3<double> sensor_pose =
      body_pose * sensor_info.get_transform_to_body();
  Isometry3<double> sensor_to_world_aligned_sensor = sensor_pose;
  sensor_to_world_aligned_sensor.translation().setZero();

  SpatialForce<double> spatial_force_in_world_aligned_sensor =
      transformSpatialForce(sensor_to_world_aligned_sensor,
                            spatial_force_in_sensor_frame);

  return ContactForce<double>(sensor_pose.translation(),
                              Vector3<double>::UnitZ(),
                              spatial_force_in_world_aligned_sensor.tail<3>(),
                              spatial_force_in_world_aligned_sensor.head<3>());
}

// This tests encoding and decoding of kinematics information to and from
// bot_core::robot_state_t. It does not test encoding or decoding of the force
// torque information, partially because it's not possible to construct
// properly contact related objects outside RigidBodyPlant, and thus hard to
// add a unit test.
void TestEncodeThenDecode(FloatingBaseType floating_base_type) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(
          "drake/examples/valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
      floating_base_type, nullptr /* weld to frame */, &tree);
  // Add a collision geom for the world.
  multibody::AddFlatTerrainToWorld(&tree, 100., 10.);

  // Make hand / foot mounted force torque sensors.
  std::vector<RigidBodyFrame<double>> force_torque_sensor_info;
  std::map<Side, RigidBodyFrame<double>> hand_ft_sensor_info;
  std::map<Side, RigidBodyFrame<double>> foot_ft_sensor_info;
  std::map<Side, std::string> foot_names;
  foot_names[Side::LEFT] = "leftFoot";
  foot_names[Side::RIGHT] = "rightFoot";
  std::map<Side, std::string> hand_names;
  hand_names[Side::LEFT] = "leftPalm";
  hand_names[Side::RIGHT] = "rightPalm";

  Isometry3<double> foot_ft_sensor_offset;
  foot_ft_sensor_offset.linear() =
      Matrix3<double>(AngleAxis<double>(M_PI, Vector3<double>::UnitX()));
  foot_ft_sensor_offset.translation() = Vector3<double>(0.02, 0., -0.09);

  Isometry3<double> hand_ft_sensor_offset;
  hand_ft_sensor_offset.linear() =
      Matrix3<double>(AngleAxis<double>(M_PI, Vector3<double>::UnitZ()));
  hand_ft_sensor_offset.translation() = Vector3<double>(0., 0., 0.05);

  for (Side side : Side::values) {
    foot_ft_sensor_info[side] = RigidBodyFrame<double>(
        foot_names[side] + "FTSensor", tree.FindBody(foot_names[side]),
        foot_ft_sensor_offset);
    hand_ft_sensor_info[side] = RigidBodyFrame<double>(
        hand_names[side] + "FTSensor", tree.FindBody(hand_names[side]),
        hand_ft_sensor_offset);
    force_torque_sensor_info.push_back(foot_ft_sensor_info[side]);
    force_torque_sensor_info.push_back(hand_ft_sensor_info[side]);
  }

  DiagramBuilder<double> builder;

  // KinematicsResults source.
  auto kinematics_results_value = make_unique<Value<KinematicsResults<double>>>(
      KinematicsResults<double>(&tree));
  KinematicsResults<double>& kinematics_results =
      kinematics_results_value->get_mutable_value();

  std::default_random_engine generator;  // Same seed every time, but that's OK.
  std::normal_distribution<double> distribution;
  auto q = tree.getRandomConfiguration(generator);
  VectorX<double> v(tree.get_num_velocities());
  for (int i = 0; i < tree.get_num_velocities(); i++) {
    v[i] = distribution(generator);
  }
  kinematics_results.Update(q, v);
  std::map<Side, Isometry3<double>> foot_poses;
  std::map<Side, Isometry3<double>> hand_poses;
  for (Side side : Side::values) {
    foot_poses[side] =
        kinematics_results.get_pose_in_world(*tree.FindBody(foot_names[side]));
    hand_poses[side] =
        kinematics_results.get_pose_in_world(*tree.FindBody(hand_names[side]));
  }

  auto& kinematics_results_source =
      *builder.AddSystem<ConstantValueSource<double>>(
          *kinematics_results_value);
  kinematics_results_source.set_name("kinematics_results_source");

  // Effort sources.
  VectorX<double> efforts = VectorX<double>::LinSpaced(
      tree.actuators.size(), 0.0, tree.actuators.size() - 1);
  std::map<const RigidBodyActuator*, const System<double>*> effort_sources;
  for (size_t i = 0; i < tree.actuators.size(); i++) {
    const auto& actuator = tree.actuators[i];
    auto effort = efforts.segment(i, 1);
    System<double>* effort_source =
        builder.AddSystem<ConstantVectorSource<double>>(effort);
    effort_source->set_name("effort_source_" + std::to_string(i));
    effort_sources.emplace(make_pair(&actuator, effort_source));
  }

  // Wrench sources.
  // These are in sensor frames.
  eigen_aligned_std_map<Side, Vector6<double>> hand_spatial_forces;
  eigen_aligned_std_map<Side, Vector6<double>> foot_spatial_forces;
  auto contact_results_value =
      make_unique<Value<ContactResults<double>>>(ContactResults<double>());
  ContactResults<double>& contact_results =
      contact_results_value->get_mutable_value();

  double spatial_forces_start = 0.0;
  for (Side side : Side::values) {
    {
      // Hand.
      Vector6<double> hand_spatial_force;
      hand_spatial_force.setLinSpaced(
          spatial_forces_start,
          spatial_forces_start + hand_spatial_force.size() - 1.0);
      spatial_forces_start += hand_spatial_force.size();
      hand_spatial_forces[side] = hand_spatial_force;

      ContactInfo<double>& contact_info = contact_results.AddContact(
          tree.FindBody(hand_names[side])->get_collision_element_ids().front(),
          tree.world().get_collision_element_ids().front());
      ContactForce<double> contact_spatial_force = TransformContactWrench(
          hand_ft_sensor_info[side], hand_poses[side], hand_spatial_force);

      contact_info.set_resultant_force(contact_spatial_force);
    }

    {
      // Foot.
      Vector6<double> foot_spatial_force;
      foot_spatial_force.setLinSpaced(
          spatial_forces_start,
          spatial_forces_start + foot_spatial_force.size() - 1.0);
      spatial_forces_start += foot_spatial_force.size();
      foot_spatial_forces[side] = foot_spatial_force;

      ContactInfo<double>& contact_info = contact_results.AddContact(
          tree.FindBody(foot_names[side])->get_collision_element_ids().front(),
          tree.world().get_collision_element_ids().front());
      ContactForce<double> contact_spatial_force = TransformContactWrench(
          foot_ft_sensor_info[side], foot_poses[side], foot_spatial_force);
      contact_info.set_resultant_force(contact_spatial_force);
    }
  }

  auto& contact_results_source =
      *builder.AddSystem<ConstantValueSource<double>>(
          *contact_results_value);
  contact_results_source.set_name("contact_results_source");

  // RobotStateEncoder and RobotStateDecoder.
  auto& robot_state_encoder =
      *builder.AddSystem<RobotStateEncoder>(tree, force_torque_sensor_info);
  robot_state_encoder.set_name("robot_state_encoder");
  auto& robot_state_decoder = *builder.AddSystem<RobotStateDecoder>(tree);
  robot_state_decoder.set_name("robot_state_decoder");

  // Connections.
  for (const auto& actuator_and_effort_source : effort_sources) {
    const auto& actuator = actuator_and_effort_source.first;
    const auto& effort_source = actuator_and_effort_source.second;
    builder.Connect(effort_source->get_output_port(0),
                    robot_state_encoder.effort_port(*actuator));
  }

  builder.Connect(kinematics_results_source.get_output_port(0),
                  robot_state_encoder.kinematics_results_port());
  builder.Connect(contact_results_source.get_output_port(0),
                  robot_state_encoder.contact_results_port());
  builder.Connect(robot_state_encoder, robot_state_decoder);

  // Diagram outputs.
  builder.ExportOutput(robot_state_decoder.get_output_port(0));
  builder.ExportOutput(robot_state_encoder.get_output_port(0));

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput();
  diagram->CalcOutput(*context, output.get());

  // TODO(tkoolen): magic numbers.
  auto cache_output = output->get_data(0)->get_value<KinematicsCache<double>>();
  auto msg_output = output->get_data(1)->get_value<robot_state_t>();

  // Tolerance because we're converting to float and back:
  double tolerance = 10. * std::numeric_limits<float>::epsilon();

  // Test message contents.
  // The message's "joint" layout should match q's non floating base portion.
  int q_non_floating_joint_start_index = 0;
  int v_non_floating_joint_start_index = 0;
  switch (floating_base_type) {
    case FloatingBaseType::kRollPitchYaw:
      q_non_floating_joint_start_index = 6;
      v_non_floating_joint_start_index = 6;
      break;
    case FloatingBaseType::kQuaternion:
    case FloatingBaseType::kExperimentalMultibodyPlantStyle:
      q_non_floating_joint_start_index = 7;
      v_non_floating_joint_start_index = 6;
      break;
    case FloatingBaseType::kFixed:
      q_non_floating_joint_start_index = 0;
      v_non_floating_joint_start_index = 0;
      break;
  }

  for (int i = 0; i < msg_output.num_joints; ++i) {
    const int q_idx = i + q_non_floating_joint_start_index;
    const int v_idx = i + v_non_floating_joint_start_index;
    EXPECT_EQ(tree.get_position_name(q_idx), msg_output.joint_name[i]);
    EXPECT_NEAR(q[q_idx], msg_output.joint_position[i], tolerance);
    EXPECT_NEAR(v[v_idx], msg_output.joint_velocity[i], tolerance);
  }

  for (size_t i = 0; i < tree.actuators.size(); i++) {
    const auto& actuator = tree.actuators[i];
    const auto& body = *actuator.body_;
    const int q_idx = body.get_position_start_index();
    const int actuator_idx = q_idx - q_non_floating_joint_start_index;
    EXPECT_NEAR(efforts[i], msg_output.joint_effort[actuator_idx], tolerance);
  }

  const auto& force_torque = msg_output.force_torque;

  // Check left foot spatial_force.
  const auto& left_foot_spatial_force = foot_spatial_forces.at(Side::LEFT);
  EXPECT_NEAR(force_torque.l_foot_torque_x,
              left_foot_spatial_force[RobotStateEncoder::kTorqueXIndex],
              tolerance);
  EXPECT_NEAR(force_torque.l_foot_torque_y,
              left_foot_spatial_force[RobotStateEncoder::kTorqueYIndex],
              tolerance);
  EXPECT_NEAR(force_torque.l_foot_force_z,
              left_foot_spatial_force[RobotStateEncoder::kForceZIndex],
              tolerance);

  // Check left hand spatial_force.
  const auto& left_hand_spatial_force = hand_spatial_forces.at(Side::LEFT);
  for (int i = 0; i < kSpaceDimension; i++) {
    EXPECT_NEAR(force_torque.l_hand_torque[i], left_hand_spatial_force[i],
                tolerance);
    EXPECT_NEAR(force_torque.l_hand_force[i],
                left_hand_spatial_force[kSpaceDimension + i], tolerance);
  }

  // Check right foot spatial_force.
  const auto& right_foot_spatial_force = foot_spatial_forces.at(Side::RIGHT);
  EXPECT_NEAR(force_torque.r_foot_torque_x,
              right_foot_spatial_force[RobotStateEncoder::kTorqueXIndex],
              tolerance);
  EXPECT_NEAR(force_torque.r_foot_torque_y,
              right_foot_spatial_force[RobotStateEncoder::kTorqueYIndex],
              tolerance);
  EXPECT_NEAR(force_torque.r_foot_force_z,
              right_foot_spatial_force[RobotStateEncoder::kForceZIndex],
              tolerance);

  // Check right hand spatial_force.
  const auto& right_hand_spatial_force = hand_spatial_forces.at(Side::RIGHT);
  for (int i = 0; i < kSpaceDimension; i++) {
    EXPECT_NEAR(force_torque.r_hand_torque[i], right_hand_spatial_force[i],
                tolerance);
    EXPECT_NEAR(force_torque.r_hand_force[i],
                right_hand_spatial_force[kSpaceDimension + i], tolerance);
  }

  // Test conversion back to KinematicsCache.
  auto q_back = cache_output.getQ();
  auto v_back = cache_output.getV();

  // Can't compare q vectors directly due to quaternion floating joints, hence
  // the following:
  for (const auto& body : tree.get_bodies()) {
    if (body->has_parent_body()) {
      const auto& joint = body->getJoint();
      if (!joint.is_fixed()) {
        auto q_joint_expected = q.segment(body->get_position_start_index(),
                                          joint.get_num_positions());
        auto q_joint_back = q_back.segment(body->get_position_start_index(),
                                           joint.get_num_positions());
        if (joint.is_floating() &&
            floating_base_type == FloatingBaseType::kQuaternion) {
          auto pos_expected = q_joint_expected.head<kSpaceDimension>();
          auto pos_back = q_joint_back.head<kSpaceDimension>();
          EXPECT_TRUE(CompareMatrices(pos_expected, pos_back, tolerance,
                                      MatrixCompareType::absolute));

          auto quat_expected = q_joint_expected.tail<kQuaternionSize>();
          auto quat_back = q_joint_back.tail<kQuaternionSize>();
          auto quat_diff = math::quatDiff(quat_expected, quat_back);
          const Eigen::Quaternion<decltype(quat_diff)::Scalar> q_eigen(
              quat_diff(0), quat_diff(1), quat_diff(2), quat_diff(3));
          Eigen::AngleAxis<decltype(quat_diff)::Scalar> angle_axis(q_eigen);
          auto angle = angle_axis.angle();
          EXPECT_NEAR(angle, 0.0, tolerance);
        } else {
          EXPECT_TRUE(CompareMatrices(q_joint_expected, q_joint_back, tolerance,
                                      MatrixCompareType::absolute));
        }
      }
    }
  }

  EXPECT_TRUE(CompareMatrices(v, cache_output.getV(), tolerance,
                              MatrixCompareType::absolute));
}

GTEST_TEST(RobotStateEncoderDecoderTest, Fixed) {
  TestEncodeThenDecode(FloatingBaseType::kFixed);
}

GTEST_TEST(RobotStateEncoderDecoderTest, RollPitchYaw) {
  TestEncodeThenDecode(FloatingBaseType::kRollPitchYaw);
}

GTEST_TEST(RobotStateEncoderDecoderTest, Quaternion) {
  TestEncodeThenDecode(FloatingBaseType::kQuaternion);
}

}  // namespace
}  // namespace systems
}  // namespace drake
