/**
 * @file This file implements a state machine that drives the kuka iiwa arm to
 * pick up a block from one table to place it on another repeatedly.
 */

#include <iostream>
#include <list>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

class WorldStateSubscriber {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WorldStateSubscriber)

  WorldStateSubscriber(lcm::LCM* lcm, WorldState* state)
      : lcm_(lcm),
        state_(state) {
    DRAKE_DEMAND(state);

    lcm_subscriptions_.push_back(
        lcm_->subscribe("EST_ROBOT_STATE",
                        &WorldStateSubscriber::HandleIiwaStatus, this));
    lcm_subscriptions_.push_back(
        lcm_->subscribe("SCHUNK_WSG_STATUS",
                        &WorldStateSubscriber::HandleWsgStatus, this));
    lcm_subscriptions_.push_back(
        lcm_->subscribe("OBJECT_STATE_EST",
                        &WorldStateSubscriber::HandleObjectStatus, this));
  }

  ~WorldStateSubscriber() {
    for (lcm::Subscription* sub : lcm_subscriptions_) {
      int status = lcm_->unsubscribe(sub);
      DRAKE_DEMAND(status == 0);
    }
    lcm_subscriptions_.clear();
  }

 private:
  // Handles iiwa states from the LCM message.
  void HandleIiwaStatus(const lcm::ReceiveBuffer*, const std::string&,
                        const bot_core::robot_state_t* iiwa_msg) {
    DRAKE_DEMAND(iiwa_msg != nullptr);
    // Extract robot base pose.
    Isometry3<double> X_WB = DecodePose(iiwa_msg->pose);
    // Extract joint positions.
    lcmt_iiwa_status iiwa_status;
    iiwa_status.utime = iiwa_msg->utime;
    iiwa_status.num_joints = iiwa_msg->num_joints;
    std::transform(iiwa_msg->joint_position.cbegin(),
                   iiwa_msg->joint_position.cend(),
                   std::back_inserter(iiwa_status.joint_position_measured),
                   [](float q) -> double { return q; });
    state_->HandleIiwaStatus(iiwa_status, X_WB);
  }

  // Handles WSG states from the LCM message.
  void HandleWsgStatus(const lcm::ReceiveBuffer*, const std::string&,
                       const lcmt_schunk_wsg_status* wsg_msg) {
    DRAKE_DEMAND(wsg_msg != nullptr);
    state_->HandleWsgStatus(*wsg_msg);
  }

  // Handles object states from the LCM message.
  void HandleObjectStatus(const lcm::ReceiveBuffer*,
                          const std::string&,
                          const bot_core::robot_state_t* obj_msg) {
    DRAKE_DEMAND(obj_msg != nullptr);
    state_->HandleObjectStatus(*obj_msg);
  }

  // LCM subscription management.
  lcm::LCM* lcm_;
  WorldState* state_;
  std::list<lcm::Subscription*> lcm_subscriptions_;
};


using manipulation::planner::ConstraintRelaxingIk;

// Makes a state machine that drives the iiwa to pick up a block from one table
// and place it on the other table.
void RunPickAndPlaceDemo() {
  lcm::LCM lcm;

  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf");
  const std::string iiwa_end_effector_name = "iiwa_link_ee";

  // Makes a WorldState, and sets up LCM subscriptions.
  WorldState env_state(iiwa_path, iiwa_end_effector_name);
  WorldStateSubscriber env_state_subscriber(&lcm, &env_state);

  // Spins until at least one message is received from every LCM channel.
  while (lcm.handleTimeout(10) == 0 || env_state.get_iiwa_time() == -1 ||
         env_state.get_obj_time() == -1 || env_state.get_wsg_time() == -1) {
  }

  // Makes a planner.
  const Isometry3<double>& iiwa_base = env_state.get_iiwa_base();
  ConstraintRelaxingIk planner(
      iiwa_path, iiwa_end_effector_name, iiwa_base);
  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        lcm.publish("COMMITTED_ROBOT_PLAN", plan);
      });

  PickAndPlaceStateMachine::WsgPublishCallback wsg_callback =
      ([&](const lcmt_schunk_wsg_command* msg) {
        lcm.publish("SCHUNK_WSG_COMMAND", msg);
      });

  PlannerConfiguration planner_configuration;
  planner_configuration.model_path =
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";
  planner_configuration.end_effector_name = "iiwa_link_ee";
  // This needs to match the object model file in iiwa_wsg_simulation.cc
  planner_configuration.target_dimensions = {0.06, 0.06, 0.1};
  planner_configuration.num_tables = 2;
  Isometry3<double> X_WT{Isometry3<double>::Identity()};
  X_WT.translation() = Vector3<double>(0, 0.8, 0);
  env_state.HandleTableStatus(0, X_WT);

  X_WT.translation() = Vector3<double>(0.8, 0, 0);
  env_state.HandleTableStatus(1, X_WT);

  PickAndPlaceStateMachine machine(planner_configuration, true);

  // lcm handle loop
  while (true) {
    // Handles all messages.
    while (lcm.handleTimeout(10) == 0) {}
    machine.Update(env_state, iiwa_callback, wsg_callback);
  }
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::kuka_iiwa_arm::pick_and_place::RunPickAndPlaceDemo();
  return 0;
}
