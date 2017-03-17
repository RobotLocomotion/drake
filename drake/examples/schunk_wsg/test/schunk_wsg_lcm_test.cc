#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

GTEST_TEST(SchunkWsgLcmTest, SchunkWsgTrajectoryGeneratorTest) {
  SchunkWsgTrajectoryGenerator dut(1, 0);
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);

  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  lcmt_schunk_wsg_command initial_command{};
  initial_command.target_position_mm = 100;
  initial_command.force = 40;
  std::unique_ptr<systems::FreestandingInputPortValue> input_command =
      std::make_unique<systems::FreestandingInputPortValue>(
          std::make_unique<systems::Value<lcmt_schunk_wsg_command>>(
              initial_command));
  context->SetInputPortValue(0, std::move(input_command));
  context->FixInputPort(1, Eigen::VectorXd::Zero(1));

  // Step a little bit. We should be commanding a point on the
  // trajectory wider than zero, but not to the target yet.
  const double expected_target = -0.05;  // 50mm
  systems::Simulator<double> simulator(dut, std::move(context));
  simulator.StepTo(0.1);
  dut.CalcOutput(simulator.get_context(), output.get());
  EXPECT_LT(output->get_vector_data(0)->GetAtIndex(0), 0);
  EXPECT_GT(output->get_vector_data(0)->GetAtIndex(0), expected_target);

  // Step quite a bit longer and see that we get there.
  simulator.StepTo(2.0);
  dut.CalcOutput(simulator.get_context(), output.get());
  EXPECT_FLOAT_EQ(output->get_vector_data(0)->GetAtIndex(0), expected_target);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
