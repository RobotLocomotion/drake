#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, BasicTest) {
  SchunkWsgTrajectoryGenerator dut(1, 0);
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output = dut.AllocateOutput();

  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  dut.get_desired_position_input_port().FixValue(context.get(), 0.05);
  dut.get_force_limit_input_port().FixValue(context.get(), 40.);
  dut.get_state_input_port().FixValue(context.get(), 0.0);

  // Step a little bit. We should be commanding a point on the
  // trajectory wider than zero, but not to the target yet.
  const double expected_target = -0.05;  // 50mm
  systems::Simulator<double> simulator(dut, std::move(context));
  simulator.AdvanceTo(0.1);
  dut.CalcOutput(simulator.get_context(), output.get());
  EXPECT_LT(output->get_vector_data(0)->GetAtIndex(0), 0);
  EXPECT_GT(output->get_vector_data(0)->GetAtIndex(0), expected_target);

  // Step quite a bit longer and see that we get there.
  simulator.AdvanceTo(2.0);
  dut.CalcOutput(simulator.get_context(), output.get());
  EXPECT_FLOAT_EQ(output->get_vector_data(0)->GetAtIndex(0), expected_target);
}

// Test the specific case when the last command does not match the current
// command but the difference between the measured position and the desired
// position is zero.
GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, DeltaEqualsZero) {
  SchunkWsgTrajectoryGenerator dut(1, 0);
  auto context = dut.CreateDefaultContext();

  SchunkWsgTrajectoryGeneratorStateVector<double> state;
  state.set_last_target_position(0.0);
  context->SetDiscreteState(state.value());

  const double pos = 0.06;
  dut.get_desired_position_input_port().FixValue(context.get(), pos);
  dut.get_force_limit_input_port().FixValue(context.get(), 40.0);
  dut.get_state_input_port().FixValue(context.get(), -pos / 2.0);

  systems::Simulator<double> simulator(dut, std::move(context));
  simulator.AdvanceTo(0.1);
  Eigen::Vector2d target =
      dut.get_target_output_port().Eval(simulator.get_context());
  EXPECT_EQ(target[0], -pos);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
