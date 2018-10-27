#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/fixed_input_port_value.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

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
  context->FixInputPort(0,
      systems::AbstractValue::Make<lcmt_schunk_wsg_command>(initial_command));
  EXPECT_EQ(dut.get_commanded_position_output_port()
            .Eval<BasicVector<double>>(*context).GetAtIndex(0),
            initial_position);
  EXPECT_EQ(dut.get_force_limit_output_port()
            .Eval<BasicVector<double>>(*context).GetAtIndex(0),
            initial_force);

  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  initial_command.utime = 1;
  initial_command.target_position_mm = 100;
  initial_command.force = 40;
  context->FixInputPort(0,
      systems::AbstractValue::Make<lcmt_schunk_wsg_command>(initial_command));
  EXPECT_EQ(dut.get_commanded_position_output_port()
            .Eval<BasicVector<double>>(*context).GetAtIndex(0), 0.05);
  EXPECT_EQ(dut.get_force_limit_output_port()
            .Eval<BasicVector<double>>(*context).GetAtIndex(0), 40);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
