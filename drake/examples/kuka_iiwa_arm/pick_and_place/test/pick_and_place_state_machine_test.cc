#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <gtest/gtest.h>
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
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
  PlannerConfiguration planner_configuration;
  planner_configuration.model_path = kIiwaUrdf;
  planner_configuration.end_effector_name = "iiwa_link_ee";
  planner_configuration.target_dimensions = {0.06, 0.06, 0.06};
  planner_configuration.num_tables = 2;

  // Arbitrary object (O) initial position.
  Vector3<double> p_WO{0.8, -0.36, 0.27};
  // Arbitrary destination table (T) fixed position.
  Vector3<double> p_WT{0.8, 0.36, 0.0};

  // Test the non-looping configuration.
  PickAndPlaceStateMachine dut(planner_configuration, true /*single_move*/);

  // Create world state and initialize with a trivial configuration.
  WorldState world_state(FindResourceOrThrow(kIiwaUrdf), "iiwa_link_ee",
                         planner_configuration.num_tables,
                         planner_configuration.target_dimensions);

  Isometry3<double> iiwa_base{Isometry3<double>::Identity()};
  lcmt_iiwa_status iiwa_msg{};
  iiwa_msg.utime = 1000;
  iiwa_msg.num_joints = kIiwaArmNumJoints;

  iiwa_msg.joint_position_measured.resize(kIiwaArmNumJoints, 0);
  iiwa_msg.joint_velocity_estimated.resize(kIiwaArmNumJoints, 0);
  world_state.HandleIiwaStatus(iiwa_msg, iiwa_base);

  lcmt_schunk_wsg_status wsg_msg;
  wsg_msg.utime = iiwa_msg.utime;
  wsg_msg.actual_position_mm = 0;
  wsg_msg.actual_force = 0;
  world_state.HandleWsgStatus(wsg_msg);

  bot_core::robot_state_t object_msg{};
  object_msg.utime = 1000;
  object_msg.pose.translation.x = p_WO.x();
  object_msg.pose.translation.y = p_WO.y();
  object_msg.pose.translation.z = p_WO.z();
  object_msg.pose.rotation.w = 1;
  object_msg.pose.rotation.x = 0;
  object_msg.pose.rotation.y = 0;
  object_msg.pose.rotation.z = 0;
  world_state.HandleObjectStatus(object_msg);

  Isometry3<double> X_WT;
  X_WT.translation() = p_WT;
  X_WT.linear().setIdentity();
  world_state.HandleTableStatus(0, X_WT);

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
      FindResourceOrThrow(kIiwaUrdf), "iiwa_link_ee",
      Isometry3<double>::Identity());

  dut.Update(world_state, iiwa_callback, wsg_callback);

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
      for (int i = 0; i < kIiwaArmNumJoints; ++i) {
        iiwa_msg.joint_position_measured[i] =
            iiwa_plan.plan.back().joint_position[i];
      }
    }
    world_state.HandleIiwaStatus(iiwa_msg, iiwa_base);

    wsg_msg.utime = iiwa_msg.utime;
    wsg_msg.actual_position_mm = wsg_command.target_position_mm;
    world_state.HandleWsgStatus(wsg_msg);

    // Warp the object to the target y position when we expect to be
    // transitioning to kApproachPlace (this is the y value it would
    // have had after kApproachPlacePregrasp completed successfully).
    if (step.state_expected == kApproachPlace) {
      object_msg.pose.translation.y = X_WT.translation()(1);
      world_state.HandleObjectStatus(object_msg);
    }

    dut.Update(world_state, iiwa_callback, wsg_callback);

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
