#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/fixed_input_port_value.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

using Eigen::Vector2d;
using systems::BasicVector;

GTEST_TEST(SchunkWsgLcmTest, SchunkWsgCommandReceiverTest) {
  const double initial_position = 0.03;
  const double initial_force = 27;

  SchunkWsgCommandReceiver dut(initial_position, initial_force);
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();

  // Check that we get the initial position and force before receiving a
  // command.
  lcmt_schunk_wsg_command initial_command{};
  context->FixInputPort(
      0,
      systems::AbstractValue::Make<lcmt_schunk_wsg_command>(initial_command));
  EXPECT_EQ(dut.get_commanded_position_output_port()
                .Eval<BasicVector<double>>(*context)
                .GetAtIndex(0),
            initial_position);
  EXPECT_EQ(dut.get_force_limit_output_port()
                .Eval<BasicVector<double>>(*context)
                .GetAtIndex(0),
            initial_force);

  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  initial_command.utime = 1;
  initial_command.target_position_mm = 100;
  initial_command.force = 40;
  context->FixInputPort(
      0,
      systems::AbstractValue::Make<lcmt_schunk_wsg_command>(initial_command));
  EXPECT_EQ(dut.get_commanded_position_output_port()
                .Eval<BasicVector<double>>(*context)
                .GetAtIndex(0),
            0.05);
  EXPECT_EQ(dut.get_force_limit_output_port()
                .Eval<BasicVector<double>>(*context)
                .GetAtIndex(0),
            40);
}

GTEST_TEST(SchunkWsgLcmTest, SchunkWsgStatusSenderTest) {
  SchunkWsgStatusSender dut;
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();

  const double position = 0.001;
  const double velocity = 0.04;
  context->FixInputPort(dut.get_state_input_port().get_index(),
                        Vector2d(position, velocity));

  // Check that the force input port is indeed optional.
  {
    const lcmt_schunk_wsg_status& status =
        dut.get_output_port(0).Eval<lcmt_schunk_wsg_status>(*context);

    EXPECT_EQ(status.actual_force, 0.0);
    EXPECT_EQ(status.actual_position_mm, 2.0);
    EXPECT_EQ(status.actual_speed_mm_per_s, 80.0);
  }

  // Check that we can also set the force.
  const double force = 32.0;
  context->FixInputPort(dut.get_force_input_port().get_index(),
                        Vector1d(force));

  EXPECT_EQ(dut.get_output_port(0)
                .Eval<lcmt_schunk_wsg_status>(*context)
                .actual_force,
            force);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
