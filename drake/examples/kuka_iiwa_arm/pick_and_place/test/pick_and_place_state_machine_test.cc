#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <gtest/gtest.h>
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

const char* const kIiwaUrdf =
    "/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

struct TestStep {
  int iiwa_plan_count_expected;
  int wsg_command_count_expected;
  PickAndPlaceState state_expected;
};

// Create a test scenario where the iiwa picks up an object 80cm in
// front of it and moves it 72cm to the right (symmetric about the
// center of the robot).  The test uses a single place location and
// does not loop.  The choice of the pick/place location is arbitrary.
GTEST_TEST(PickAndPlaceStateMachineTest, StateMachineTest) {
  Isometry3<double> place_location;
  place_location.translation() = Eigen::Vector3d(0.80, 0.36, 0);
  place_location.linear().setIdentity();
  std::vector<Isometry3<double>> place_locations;
  place_locations.push_back(place_location);

  // Test the non-looping configuration.
  PickAndPlaceStateMachine dut(place_locations, false);

  // Create world state and initialize with a trivial configuration.
  WorldState world_state(GetDrakePath() + kIiwaUrdf, "iiwa_link_ee");

  bot_core::robot_state_t iiwa_msg{};
  iiwa_msg.utime = 1000;
  iiwa_msg.pose.translation.x = 0;
  iiwa_msg.pose.translation.y = 0;
  iiwa_msg.pose.translation.z = 0;
  iiwa_msg.pose.rotation.w = 1;
  iiwa_msg.pose.rotation.x = 0;
  iiwa_msg.pose.rotation.y = 0;
  iiwa_msg.pose.rotation.z = 0;
  iiwa_msg.num_joints = kIiwaArmNumJoints;
  iiwa_msg.joint_name.push_back("iiwa_joint_1");
  iiwa_msg.joint_name.push_back("iiwa_joint_2");
  iiwa_msg.joint_name.push_back("iiwa_joint_3");
  iiwa_msg.joint_name.push_back("iiwa_joint_4");
  iiwa_msg.joint_name.push_back("iiwa_joint_5");
  iiwa_msg.joint_name.push_back("iiwa_joint_6");
  iiwa_msg.joint_name.push_back("iiwa_joint_7");

  iiwa_msg.joint_position.resize(kIiwaArmNumJoints, 0);
  iiwa_msg.joint_velocity.resize(kIiwaArmNumJoints, 0);
  world_state.HandleIiwaStatus(iiwa_msg);

  lcmt_schunk_wsg_status wsg_msg;
  wsg_msg.utime = iiwa_msg.utime;
  wsg_msg.actual_position_mm = 0;
  wsg_msg.actual_force = 0;
  world_state.HandleWsgStatus(wsg_msg);

  bot_core::robot_state_t object_msg{};
  object_msg.utime = 1000;
  object_msg.pose.translation.x = 0.80;
  object_msg.pose.translation.y = -0.36;
  object_msg.pose.translation.z = 0.27;
  object_msg.pose.rotation.w = 1;
  object_msg.pose.rotation.x = 0;
  object_msg.pose.rotation.y = 0;
  object_msg.pose.rotation.z = 0;
  world_state.HandleObjectStatus(object_msg);

  int iiwa_plan_count = 0;
  robotlocomotion::robot_plan_t iiwa_plan{};
  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      [&iiwa_plan_count, &iiwa_plan](
          const robotlocomotion::robot_plan_t* plan) {
    iiwa_plan_count++;
    iiwa_plan = *plan;
  };

  int wsg_command_count = 0;
  lcmt_schunk_wsg_command wsg_command{};
  PickAndPlaceStateMachine::WsgPublishCallback wsg_callback =
      [&wsg_command_count, &wsg_command](
          const lcmt_schunk_wsg_command* command) {
    wsg_command_count++;
    wsg_command = *command;
  };

  manipulation::planner::ConstraintRelaxingIk planner(
      GetDrakePath() + kIiwaUrdf, "iiwa_link_ee",
      Isometry3<double>::Identity());

  dut.Update(world_state, iiwa_callback, wsg_callback, &planner);

  std::vector<TestStep> steps;
  steps.push_back(TestStep{0, 1, kOpenGripper});
  steps.push_back(TestStep{0, 1, kApproachPickPregrasp});
  steps.push_back(TestStep{1, 1, kApproachPickPregrasp});
  steps.push_back(TestStep{1, 1, kApproachPick});
  steps.push_back(TestStep{2, 1, kApproachPick});
  steps.push_back(TestStep{3, 1, kGrasp});
  steps.push_back(TestStep{3, 2, kGrasp});
  steps.push_back(TestStep{3, 2, kGrasp});
  steps.push_back(TestStep{3, 2, kLiftFromPick});
  steps.push_back(TestStep{4, 2, kLiftFromPick});
  steps.push_back(TestStep{4, 2, kApproachPlacePregrasp});
  steps.push_back(TestStep{5, 2, kApproachPlacePregrasp});
  steps.push_back(TestStep{5, 2, kApproachPlace});
  steps.push_back(TestStep{6, 2, kApproachPlace});
  steps.push_back(TestStep{7, 2, kPlace});
  steps.push_back(TestStep{7, 3, kPlace});
  steps.push_back(TestStep{7, 3, kPlace});
  steps.push_back(TestStep{7, 3, kLiftFromPlace});
  steps.push_back(TestStep{8, 3, kLiftFromPlace});
  steps.push_back(TestStep{9, 3, kDone});
  steps.push_back(TestStep{9, 3, kDone});

  for (const TestStep& step : steps) {
    // Steps are long (5 seconds) so actions always complete in a
    // small number of steps.
    iiwa_msg.utime += 5000000;
    if (!iiwa_plan.plan.empty()) {
      iiwa_msg.joint_position = iiwa_plan.plan.back().joint_position;
    }
    world_state.HandleIiwaStatus(iiwa_msg);

    wsg_msg.utime = iiwa_msg.utime;
    wsg_msg.actual_position_mm = wsg_command.target_position_mm;
    world_state.HandleWsgStatus(wsg_msg);

    // Warp the object to the target y position when we expect to be
    // transitioning to kApproachPlace (this is the y value it would
    // have had after kApproachPlacePregrasp completed successfully).
    if (step.state_expected == kApproachPlace) {
      object_msg.pose.translation.y = place_location.translation()(1);
      world_state.HandleObjectStatus(object_msg);
    }

    dut.Update(world_state, iiwa_callback, wsg_callback, &planner);

    EXPECT_EQ(iiwa_plan_count, step.iiwa_plan_count_expected);
    EXPECT_EQ(wsg_command_count, step.wsg_command_count_expected);
    EXPECT_EQ(dut.state(), step.state_expected);
  }
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
