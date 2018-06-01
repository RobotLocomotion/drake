#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

/// Runs the controller for a brief period with the specified initial
/// conditions and returns the commanded force.
double RunWsgControllerTestStep(const lcmt_schunk_wsg_command& wsg_command,
                                double wsg_position) {
  SchunkWsgController dut;
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);
  context->FixInputPort(
      dut.get_command_input_port().get_index(),
      std::make_unique<systems::Value<lcmt_schunk_wsg_command>>(
          wsg_command));
  Eigen::VectorXd wsg_position_vec = Eigen::VectorXd::Zero(10);
  wsg_position_vec(0) = -(wsg_position / 1e3) / 2.;
  context->FixInputPort(dut.get_state_input_port().get_index(),
                        wsg_position_vec);
  systems::Simulator<double> simulator(dut, std::move(context));
  simulator.StepTo(1.0);
  dut.CalcOutput(simulator.get_context(), output.get());
  return output->get_vector_data(0)->GetAtIndex(0);
}

GTEST_TEST(SchunkWsgControllerTest, SchunkWsgControllerTest) {
  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  lcmt_schunk_wsg_command wsg_command{};
  wsg_command.target_position_mm = 100;
  wsg_command.force = 40;
  double commanded_force = RunWsgControllerTestStep(wsg_command, 0);
  EXPECT_FLOAT_EQ(commanded_force, -wsg_command.force);

  // Move in toward the middle of the range with lower force from the outside.
  wsg_command.target_position_mm = 50;
  wsg_command.force = 20;
  commanded_force = RunWsgControllerTestStep(wsg_command, 100);
  EXPECT_FLOAT_EQ(commanded_force, wsg_command.force);

  // Set the position to something near the target and observe zero force.
  commanded_force = RunWsgControllerTestStep(
      wsg_command, wsg_command.target_position_mm * 0.99);
  EXPECT_NEAR(commanded_force, 0, 1);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
