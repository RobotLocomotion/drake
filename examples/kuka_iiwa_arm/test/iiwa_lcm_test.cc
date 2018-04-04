#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

static const int kNumJoints = 7;

GTEST_TEST(IiwaLcmTest, IiwaCommandReceiverTest) {
  IiwaCommandReceiver dut;
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);

  // Check that the commanded pose starts out at zero, and that we can
  // set a different initial position.
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(kNumJoints * 2);
  dut.CalcOutput(*context, output.get());
  const double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(
      expected, output->get_vector_data(0)->get_value(),
      tol, MatrixCompareType::absolute));

  Eigen::VectorXd position(kNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  dut.set_initial_position(context.get(), position);
  dut.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(
      position, output->get_vector_data(0)->get_value().head(kNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      expected.tail(kNumJoints),
      output->get_vector_data(0)->get_value().tail(kNumJoints),
      tol, MatrixCompareType::absolute));

  Eigen::VectorXd delta(kNumJoints);
  delta << 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007;

  lcmt_iiwa_command command{};
  command.num_joints = kNumJoints;
  command.joint_position.resize(kNumJoints);
  for (int i = 0; i < kNumJoints; i++) {
    command.joint_position[i] = position(i) + delta(i);
  }

  context->FixInputPort(
      0, std::make_unique<systems::Value<lcmt_iiwa_command>>(command));

  std::unique_ptr<systems::DiscreteValues<double>> update =
      dut.AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  dut.CalcDiscreteVariableUpdates(*context, update.get());
  context->set_discrete_state(std::move(update));

  dut.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(
      position + delta,
      output->get_vector_data(0)->get_value().head(kNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      delta/ kIiwaLcmStatusPeriod,
      output->get_vector_data(0)->get_value().tail(kNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      VectorX<double>::Zero(kNumJoints),
      output->get_vector_data(1)->get_value(),
      tol, MatrixCompareType::absolute));

  // Test with joint torque command.
  command.num_torques = kNumJoints;
  command.joint_torque.resize(kNumJoints);
  VectorX<double> expected_torque(kNumJoints);
  for (int i = 0; i < kNumJoints; i++) {
    command.joint_torque[i] = -1 + 3 * i;
    expected_torque[i] = command.joint_torque[i];
  }
  context->FixInputPort(
      0, std::make_unique<systems::Value<lcmt_iiwa_command>>(command));
  update = dut.AllocateDiscreteVariables();
  dut.CalcDiscreteVariableUpdates(*context, update.get());
  context->set_discrete_state(std::move(update));
  dut.CalcOutput(*context, output.get());

  EXPECT_TRUE(CompareMatrices(
      expected_torque,
      output->get_vector_data(1)->get_value(),
      tol, MatrixCompareType::absolute));
}

GTEST_TEST(IiwaLcmTest, IiwaCommandSenderTest) {
  IiwaCommandSender dut;
  std::unique_ptr<systems::Context<double>>
      context = dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);

  Eigen::VectorXd position(kNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  context->FixInputPort(dut.get_position_input_port().get_index(), position);

  dut.CalcOutput(*context, output.get());
  lcmt_iiwa_command command =
      output->get_data(0)->GetValue<lcmt_iiwa_command>();
  ASSERT_EQ(command.num_joints, kNumJoints);
  for (int i = 0; i < kNumJoints; i++) {
    EXPECT_EQ(command.joint_position[i], position(i));
  }
  EXPECT_EQ(command.num_torques, 0);

  Eigen::VectorXd torque(kNumJoints);
  torque << 1, 2, 3, 4, 5, 6, 7;
  context->FixInputPort(dut.get_torque_input_port().get_index(), torque);

  dut.CalcOutput(*context, output.get());
  command = output->get_data(0)->GetValue<lcmt_iiwa_command>();
  ASSERT_EQ(command.num_joints, kNumJoints);
  ASSERT_EQ(command.num_torques, kNumJoints);
  for (int i = 0; i < kNumJoints; i++) {
    EXPECT_EQ(command.joint_position[i], position(i));
    EXPECT_EQ(command.joint_torque[i], torque(i));
  }
}

GTEST_TEST(IiwaLcmTest, IiwaStatusReceiverTest) {
  IiwaStatusReceiver dut;
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);

  lcmt_iiwa_status status{};
  status.num_joints = kNumJoints;
  status.joint_position_measured.resize(status.num_joints, 0);
  status.joint_velocity_estimated.resize(status.num_joints, 0);
  status.joint_position_commanded.resize(status.num_joints, 0.1);
  status.joint_position_ipo.resize(status.num_joints, 0);
  status.joint_torque_measured.resize(status.num_joints, 0);
  status.joint_torque_commanded.resize(status.num_joints, 0);
  status.joint_torque_external.resize(status.num_joints, 0);

  Eigen::VectorXd delta(kNumJoints);
  delta << 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007;
  for (int i = 0; i < kNumJoints; i++) {
    status.joint_position_measured[i] += delta(i);
    status.joint_velocity_estimated[i] = delta(i) / kIiwaLcmStatusPeriod;
  }

  context->FixInputPort(
      0, std::make_unique<systems::Value<lcmt_iiwa_status>>(status));

  std::unique_ptr<systems::DiscreteValues<double>> update =
      dut.AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  dut.CalcDiscreteVariableUpdates(*context, update.get());
  context->set_discrete_state(std::move(update));

  dut.CalcOutput(*context, output.get());
  const auto measured = output->get_vector_data(
      dut.get_measured_position_output_port().get_index())->get_value();
  const auto commanded = output->get_vector_data(
      dut.get_commanded_position_output_port().get_index())->get_value();

  const double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(
      delta, measured.head(kNumJoints), tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      delta / kIiwaLcmStatusPeriod, measured.tail(kNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      Eigen::VectorXd::Ones(kNumJoints)  * 0.1, commanded,
      tol, MatrixCompareType::absolute));
}

GTEST_TEST(IiwaLcmTest, IiwaStatusSenderTest) {
  IiwaStatusSender dut;
  std::unique_ptr<systems::Context<double>>
      context = dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);

  Eigen::VectorXd position(kNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;

  Eigen::VectorXd command = Eigen::VectorXd::Zero(kNumJoints * 2);
  command.head(kNumJoints) = position * 0.5;
  context->FixInputPort(dut.get_command_input_port().get_index(), command);

  Eigen::VectorXd state = Eigen::VectorXd::Zero(kNumJoints * 2);
  state.head(kNumJoints) = position;
  state.tail(kNumJoints) << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  context->FixInputPort(dut.get_state_input_port().get_index(), state);

  Eigen::VectorXd torque = Eigen::VectorXd::Zero(kNumJoints);
  torque << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7;
  context->FixInputPort(
      dut.get_commanded_torque_input_port().get_index(), torque);

  dut.CalcOutput(*context, output.get());
  lcmt_iiwa_status status =
      output->get_data(0)->GetValue<lcmt_iiwa_status>();
  ASSERT_EQ(status.num_joints, kNumJoints);
  for (int i = 0; i < kNumJoints; i++) {
    EXPECT_EQ(status.joint_position_commanded[i], command(i));
    EXPECT_EQ(status.joint_position_measured[i], state(i));
    EXPECT_EQ(status.joint_velocity_estimated[i], state(i + kNumJoints));
    EXPECT_EQ(status.joint_torque_commanded[i], torque(i));
    // If the measured torque input port is not connected, the commanded torque
    // is used instead.
    EXPECT_EQ(status.joint_torque_measured[i], torque(i));
    // When not connected, this field should always be 0.
    EXPECT_EQ(status.joint_torque_external[i], 0);
  }

  // Now fix the measured torque input.
  torque << 11, 12, 13, 14, 15, 16, 17;
  context->FixInputPort(
      dut.get_measured_torque_input_port().get_index(), torque);
  context->FixInputPort(
      dut.get_external_torque_input_port().get_index(), torque);
  dut.CalcOutput(*context, output.get());
  status = output->get_data(0)->GetValue<lcmt_iiwa_status>();
  for (int i = 0; i < kNumJoints; i++) {
    EXPECT_EQ(status.joint_torque_measured[i], torque(i));
    EXPECT_EQ(status.joint_torque_external[i], torque(i));
  }
}

// Builds a RBT with two iiwas, but only select the second iiwa's generalized
// force when converting to external joint torque.
GTEST_TEST(IiwaLcmTest, IiwaContactResultsToExternalTorque) {
  const std::string kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  auto tree_builder =
      std::make_unique<manipulation::util::WorldSimTreeBuilder<double>>();
  tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);

  tree_builder->AddFixedModelInstance("iiwa", Vector3<double>::Zero());
  int id1 =
      tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, 1));

  auto tree = tree_builder->Build();

  // Only interested in the second iiwa's external torque output.
  IiwaContactResultsToExternalTorque dut(*tree, {id1});
  std::unique_ptr<systems::Context<double>>
      context = dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);

  VectorX<double> expected(tree->get_num_velocities());
  for (int i = 0; i < expected.size(); i++)
    expected[i] = 42 + i;
  systems::ContactResults<double> contact_results;
  contact_results.set_generalized_contact_force(expected);

  context->FixInputPort(0,
      systems::AbstractValue::Make<systems::ContactResults<double>>(
          contact_results));

  // Check output.
  dut.CalcOutput(*context, output.get());
  const auto ext_torque = output->get_vector_data(0)->get_value();
  EXPECT_EQ(ext_torque.size(), 7);
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(ext_torque[i], expected[7 + i]);
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
