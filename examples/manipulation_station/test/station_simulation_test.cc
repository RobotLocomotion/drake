#include "drake/examples/manipulation_station/station_simulation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using Eigen::VectorXd;
using systems::BasicVector;

GTEST_TEST(SimulationStationTest, CheckPlantBasics) {
  StationSimulation<double> station(0.001);
  station.Finalize();

  auto& plant = station.get_multibody_plant();
  EXPECT_EQ(plant.num_actuated_dofs(), 9);  // 7 iiwa + 2 wsg.

  auto context = station.CreateDefaultContext();
  VectorXd q = VectorXd::LinSpaced(7, 0.1, 0.7),
           v = VectorXd::LinSpaced(7, 1.1, 1.7),
           q_command = VectorXd::LinSpaced(7, 2.1, 2.7),
           tau_ff = VectorXd::LinSpaced(7, 3.1, 3.7);

  // Set positions and read them back out, multiple ways.
  station.SetIiwaPosition(q, context.get());
  EXPECT_TRUE(CompareMatrices(q, station.GetIiwaPosition(*context)));
  EXPECT_TRUE(CompareMatrices(q, station.GetOutputPort("iiwa_position_measured")
                                     .Eval<BasicVector<double>>(*context)
                                     .get_value()));

  // Set velocities and read them back out, multiple ways.
  station.SetIiwaVelocity(v, context.get());
  EXPECT_TRUE(CompareMatrices(v, station.GetIiwaVelocity(*context)));
  EXPECT_TRUE(
      CompareMatrices(v, station.GetOutputPort("iiwa_velocity_estimated")
                             .Eval<BasicVector<double>>(*context)
                             .get_value()));

  // Check position command pass through.
  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        q_command);
  EXPECT_TRUE(CompareMatrices(q_command,
                              station.GetOutputPort("iiwa_position_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // Check feedforward_torque command.
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));
  VectorXd tau_with_no_ff = station.GetOutputPort("iiwa_torque_commanded")
                                .Eval<BasicVector<double>>(*context)
                                .get_value();
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(), tau_ff);
  EXPECT_TRUE(CompareMatrices(tau_with_no_ff + tau_ff,
                              station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // Check iiwa_torque_commanded == iiwa_torque_measured.
  EXPECT_TRUE(CompareMatrices(station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              station.GetOutputPort("iiwa_torque_measured")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));
}

GTEST_TEST(SimulationStationTest, CheckDynamics) {
  const double kTimeStep = 0.001;
  StationSimulation<double> station(kTimeStep);
  station.Finalize();

  auto context = station.CreateDefaultContext();

  // Expect state from the desired_state_from_position and from the plant.
  EXPECT_EQ(context->get_num_discrete_state_groups(), 2);

  // The tests below expect desired_state_from_position to be the second
  // state.  Verify this by checking the sizes.
  const int plant_index = 0, state_from_position_index = 1;
  EXPECT_GT(context->get_discrete_state(plant_index).size(),
            7);  // iiwa+wsg pos+vel.
  EXPECT_EQ(context->get_discrete_state(state_from_position_index).size(), 7);

  // If all inputs and states are zero, (robot is vertical, and desired state
  // == actual state), then it just so happens to be a fixed point for the
  // controller (torque_commanded == 0).
  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        VectorXd::Zero(7));
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));
  context->get_mutable_discrete_state(plant_index).SetZero();
  context->get_mutable_discrete_state(state_from_position_index).SetZero();

  auto next_state = station.AllocateDiscreteVariables();
  station.CalcDiscreteVariableUpdates(*context, next_state.get());

  EXPECT_TRUE(
      next_state->get_vector(state_from_position_index).get_value().isZero(0));

  EXPECT_TRUE(station.GetOutputPort("iiwa_torque_commanded")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero(0));

  // Note the large tolerance here (still investigating):
  EXPECT_TRUE(next_state->get_vector(plant_index).get_value().isZero(1e-4));

  // Pretend we had previously commanded some non-zero position.  This would
  // lead a non-zero desired velocity, which should make the commanded torque
  // no-longer zero.
  VectorXd last_position_command = VectorXd::LinSpaced(7, 0.432, 0.641);
  context->get_mutable_discrete_state(state_from_position_index)
      .SetFromVector(last_position_command);

  EXPECT_FALSE(station.GetOutputPort("iiwa_torque_commanded")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero(0));
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
