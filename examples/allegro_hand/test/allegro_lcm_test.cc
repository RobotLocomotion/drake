#include "drake/examples/allegro_hand/allegro_lcm.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace allegro_hand {

GTEST_TEST(AllegroLcmTest, AllegroCommandReceiver) {
  AllegroCommandReceiver dut;
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput();

  // Check that the commanded pose starts out at zero, and that we can
  // set a different initial position.
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(kAllegroNumJoints * 2);
  dut.CalcOutput(*context, output.get());
  const double tol = 1e-5;
  EXPECT_TRUE(CompareMatrices(
      expected, output->get_vector_data(0)->value(),
      tol, MatrixCompareType::absolute));

  Eigen::VectorXd position(kAllegroNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
              1.2, 1.3, 1.4, 1.5, 1.6;
  dut.set_initial_position(context.get(), position);
  dut.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(
      position, output->get_vector_data(0)->value()
      .head(kAllegroNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      expected.tail(kAllegroNumJoints),
      output->get_vector_data(0)->value().tail(kAllegroNumJoints),
      tol, MatrixCompareType::absolute));

  Eigen::VectorXd delta(kAllegroNumJoints);
  delta << 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009,
           0.010, 0.011, 0.012, 0.013, 0.014, 0.015, 0.016;

  lcmt_allegro_command command{};
  command.num_joints = kAllegroNumJoints;
  command.joint_position.resize(kAllegroNumJoints);
  for (int i = 0; i < kAllegroNumJoints; i++) {
    command.joint_position[i] = position(i) + delta(i);
  }

  dut.get_input_port(0).FixValue(context.get(), command);

  std::unique_ptr<systems::DiscreteValues<double>> update =
      dut.AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  dut.CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().SetFrom(*update);

  dut.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(
      position + delta,
      output->get_vector_data(0)->value().head(kAllegroNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      VectorX<double>::Zero(kAllegroNumJoints),
      output->get_vector_data(0)->value().tail(kAllegroNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      VectorX<double>::Zero(kAllegroNumJoints),
      output->get_vector_data(1)->value(),
      tol, MatrixCompareType::absolute));
}

GTEST_TEST(AllegroLcmTest, AllegroStatusSenderTest) {
  AllegroStatusSender dut;
  std::unique_ptr<systems::Context<double>>
      context = dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput();

  Eigen::VectorXd position(kAllegroNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
              1.2, 1.3, 1.4, 1.5, 1.6;

  Eigen::VectorXd command = Eigen::VectorXd::Zero(kAllegroNumJoints * 2);
  command.head(kAllegroNumJoints) = position * 0.5;
  dut.get_command_input_port().FixValue(context.get(), command);

  Eigen::VectorXd state = Eigen::VectorXd::Zero(kAllegroNumJoints * 2);
  state.head(kAllegroNumJoints) = position;
  state.tail(kAllegroNumJoints) << 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8,
                                   0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  dut.get_state_input_port().FixValue(context.get(), state);

  Eigen::VectorXd torque = Eigen::VectorXd::Zero(kAllegroNumJoints);
  torque << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1,
            2.2, 2.3, 2.4, 2.5, 2.6;;
  dut.get_commanded_torque_input_port().FixValue(context.get(), torque);

  dut.CalcOutput(*context, output.get());
  lcmt_allegro_status status =
      output->get_data(0)->get_value<lcmt_allegro_status>();
  ASSERT_EQ(status.num_joints, kAllegroNumJoints);
  for (int i = 0; i < kAllegroNumJoints; i++) {
    EXPECT_EQ(status.joint_position_commanded[i], command(i));
    EXPECT_EQ(status.joint_position_measured[i], state(i));
    EXPECT_EQ(status.joint_velocity_estimated[i], state(
                                                  i + kAllegroNumJoints));
    EXPECT_EQ(status.joint_torque_commanded[i], torque(i));
  }
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
