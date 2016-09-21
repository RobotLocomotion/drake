#include "drake/automotive/automotive_simulator.h"

#include "gtest/gtest.h"

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace automotive {
namespace {

// Simple touches on the getters.
GTEST_TEST(AutomotiveSimulatorTest, BasicTest) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  EXPECT_NE(nullptr, simulator->get_lcm());
  EXPECT_NE(nullptr, simulator->get_builder());
}

// Cover AddSimpleCar (and thus AddPublisher), Start, StepBy, GetSystemByName.
GTEST_TEST(AutomotiveSimulatorTest, SimpleCarTest) {
  const std::string driving_command_name =
      systems::lcm::LcmSubscriberSystem::get_name("DRIVING_COMMAND");
  // TODO(jwnimmer-tri) Do something better than "0_" here.
  const std::string joint_state_name =
      systems::lcm::LcmPublisherSystem::get_name("0_FLOATING_JOINT_STATE");

  // Set up a basic simulation with just SimpleCar and its hangers-on.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->AddSimpleCar();
  // Grab the pieces we want (testing the GetSystemByName in the process).
  auto& command_sub = dynamic_cast<systems::lcm::LcmSubscriberSystem&>(
      simulator->GetBuilderSystemByName(driving_command_name));
  auto& state_pub = dynamic_cast<systems::lcm::LcmPublisherSystem&>(
      simulator->GetBuilderSystemByName(joint_state_name));

  // Set full throttle.
  DrivingCommand<double> command;
  command.set_throttle(1.0);
  command_sub.SetMessage(0.0 /* time */, command);

  // Immediately after starting, we should have not have moved much.
  simulator->Start();
  simulator->StepBy(0.01);
  EulerFloatingJointState<double> joint_value;
  state_pub.GetMessage(&joint_value);
  EXPECT_GT(joint_value.x(), 0.0);
  EXPECT_LT(joint_value.x(), 0.001);

  // Move a lot.  Confirm that we're moving in +x.
  for (int i = 0; i < 100; ++i) {
    simulator->StepBy(0.01);
  }
  // TODO(jwnimmer-tri) Check the timestamp of the final publication.
  state_pub.GetMessage(&joint_value);
  EXPECT_GT(joint_value.x(), 1.0);

  // The subsystem pointers must not change.
  EXPECT_EQ(
      &simulator->GetDiagramSystemByName(driving_command_name), &command_sub);
  EXPECT_EQ(&simulator->GetDiagramSystemByName(joint_state_name), &state_pub);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
