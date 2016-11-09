#include "drake/examples/Valkyrie/robot_state_encoder.h"
#include "drake/examples/Valkyrie/robot_state_decoder.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::move;
using std::make_pair;
using std::map;

using bot_core::robot_state_t;

using multibody::joints::FloatingBaseType;

void TestEncodeThenDecode(FloatingBaseType floating_base_type) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      floating_base_type, nullptr /* weld to frame */, &tree);

  DiagramBuilder<double> builder;

  // KinematicsResults source.
  auto kinematics_results = make_unique<Value<KinematicsResults<double>>>(
      KinematicsResults<double>(&tree));
  std::default_random_engine generator;  // Same seed every time, but that's OK.
  std::normal_distribution<double> distribution;
  auto q = tree.getRandomConfiguration(generator);
  VectorX<double> v(tree.get_num_velocities());
  for (int i = 0; i < tree.get_num_velocities(); i++) {
    v[i] = distribution(generator);
  }
  kinematics_results->GetMutableValue<KinematicsResults<double>>().Update(q, v);
  auto& kinematics_results_source =
      *builder.AddSystem<ConstantValueSource<double>>(move(kinematics_results));

  // Effort sources.
  VectorX<double> efforts = VectorX<double>::LinSpaced(
      tree.actuators.size(), 0.0, tree.actuators.size() - 1);
  std::map<const RigidBodyActuator*, const System<double>*> effort_sources;
  for (size_t i = 0; i < tree.actuators.size(); i++) {
    const auto& actuator = tree.actuators[i];
    auto effort = efforts.segment(i, 1);
    const auto& effort_source =
        builder.AddSystem<ConstantVectorSource<double>>(effort);
    effort_sources.emplace(make_pair(&actuator, effort_source));
  }

  // Wrench sources.
  map<Side, System<double>*> hand_wrench_sensors;
  map<Side, System<double>*> foot_wrench_sensors;
  eigen_aligned_std_map<Side, Vector6<double>> hand_wrenches;
  eigen_aligned_std_map<Side, Vector6<double>> foot_wrenches;
  double wrenches_start = 0.0;
  for (Side side : Side::values) {
    // Hand.
    Vector6<double> hand_wrench;
    hand_wrench.setLinSpaced(wrenches_start,
                             wrenches_start + hand_wrench.size() - 1.0);
    wrenches_start += hand_wrench.size();
    hand_wrench_sensors[side] =
        builder.AddSystem<ConstantVectorSource>(hand_wrench);
    hand_wrenches[side] = hand_wrench;

    // Foot.
    Vector6<double> foot_wrench;
    foot_wrench.setLinSpaced(wrenches_start,
                             wrenches_start + foot_wrench.size() - 1.0);
    wrenches_start += foot_wrench.size();
    foot_wrench_sensors[side] =
        builder.AddSystem<ConstantVectorSource>(foot_wrench);
    foot_wrenches[side] = foot_wrench;
  }

  // RobotStateEncoder and RobotStateDecoder.
  auto& robot_state_encoder = *builder.AddSystem<RobotStateEncoder>(tree);
  auto& robot_state_decoder = *builder.AddSystem<RobotStateDecoder>(tree);

  // Connections.
  for (const auto& actuator_and_effort_source : effort_sources) {
    const auto& actuator = actuator_and_effort_source.first;
    const auto& effort_source = actuator_and_effort_source.second;
    builder.Connect(effort_source->get_output_port(0),
                    robot_state_encoder.effort_port(*actuator));
  }

  for (Side side : Side::values) {
    builder.Connect(hand_wrench_sensors.at(side)->get_output_port(0),
                    robot_state_encoder.hand_contact_wrench_port(side));
    builder.Connect(foot_wrench_sensors.at(side)->get_output_port(0),
                    robot_state_encoder.foot_contact_wrench_port(side));
  }

  builder.Connect(kinematics_results_source.get_output_port(0),
                  robot_state_encoder.kinematics_results_port());
  builder.Connect(robot_state_encoder, robot_state_decoder);

  // Diagram outputs.
  builder.ExportOutput(robot_state_decoder.get_output_port(0));
  builder.ExportOutput(robot_state_encoder.get_output_port(0));

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);
  diagram->EvalOutput(*context, output.get());

  // TODO(tkoolen): magic numbers.
  auto cache_output = output->get_data(0)->GetValue<KinematicsCache<double>>();
  auto msg_output = output->get_data(1)->GetValue<robot_state_t>();

  // Tolerance because we're converting to float and back:
  double tolerance = 10. * std::numeric_limits<float>::epsilon();

  // Test message contents.
  for (size_t i = 0; i < tree.actuators.size(); i++) {
    const auto& actuator = tree.actuators[i];
    const auto& body = *actuator.body_;

    // Ensure that joint names are correct, and match the order of
    // tree.actuators.
    EXPECT_EQ(tree.get_position_name(body.get_position_start_index()),
              msg_output.joint_name[i]);

    // Ensure that joint position, joint velocity, and joint effort are correct.
    EXPECT_NEAR(efforts[i], msg_output.joint_effort[i], tolerance);
    EXPECT_NEAR(q[body.get_position_start_index()],
                msg_output.joint_position[i], tolerance);
    EXPECT_NEAR(v[body.get_velocity_start_index()],
                msg_output.joint_velocity[i], tolerance);
  }

  const auto& force_torque = msg_output.force_torque;

  // Check left foot wrench.
  const auto& left_foot_wrench = foot_wrenches.at(Side::LEFT);
  EXPECT_NEAR(force_torque.l_foot_torque_x,
              left_foot_wrench[RobotStateEncoder::kTorqueXIndex], tolerance);
  EXPECT_NEAR(force_torque.l_foot_torque_y,
              left_foot_wrench[RobotStateEncoder::kTorqueYIndex], tolerance);
  EXPECT_NEAR(force_torque.l_foot_force_z,
              left_foot_wrench[RobotStateEncoder::kForceZIndex], tolerance);

  // Check left hand wrench.
  const auto& left_hand_wrench = hand_wrenches.at(Side::LEFT);
  for (int i = 0; i < kSpaceDimension; i++) {
    EXPECT_NEAR(force_torque.l_hand_torque[i], left_hand_wrench[i], tolerance);
    EXPECT_NEAR(force_torque.l_hand_force[i],
                left_hand_wrench[kSpaceDimension + i], tolerance);
  }

  // Check right foot wrench.
  const auto& right_foot_wrench = foot_wrenches.at(Side::RIGHT);
  EXPECT_NEAR(force_torque.r_foot_torque_x,
              right_foot_wrench[RobotStateEncoder::kTorqueXIndex], tolerance);
  EXPECT_NEAR(force_torque.r_foot_torque_y,
              right_foot_wrench[RobotStateEncoder::kTorqueYIndex], tolerance);
  EXPECT_NEAR(force_torque.r_foot_force_z,
              right_foot_wrench[RobotStateEncoder::kForceZIndex], tolerance);

  // Check right hand wrench.
  const auto& right_hand_wrench = hand_wrenches.at(Side::RIGHT);
  for (int i = 0; i < kSpaceDimension; i++) {
    EXPECT_NEAR(force_torque.r_hand_torque[i], right_hand_wrench[i], tolerance);
    EXPECT_NEAR(force_torque.r_hand_force[i],
                right_hand_wrench[kSpaceDimension + i], tolerance);
  }

  // Test conversion back to KinematicsCache.
  auto q_back = cache_output.getQ();
  auto v_back = cache_output.getV();

  // Can't compare q vectors directly due to quaternion floating joints, hence
  // the following:
  for (const auto& body : tree.bodies) {
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
          auto angle_axis = math::quat2axis(quat_diff);
          // TODO(hongkai.dai): fix magic number once we use Eigen's AngleAxis:
          auto angle = angle_axis[3];
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
