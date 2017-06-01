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
#include "drake/common/drake_path.h"
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
        lcm_->subscribe("IIWA_STATE_EST",
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
    state_->HandleIiwaStatus(*iiwa_msg);
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

  const std::string iiwa_path = GetDrakePath() +
      "/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf";
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

  // This needs to match the object model file in iiwa_wsg_simulation.cc
  const double half_box_height = 0.1;
  std::vector<Isometry3<double>> place_locations;
  Isometry3<double> place_location;
  // TODO(sam.creasey) fix these
  place_location.translation() = Vector3<double>(0, 0.8, half_box_height);
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  place_location.translation() = Vector3<double>(0.8, 0, half_box_height);
  place_location.linear().setIdentity();
  place_locations.push_back(place_location);

  PickAndPlaceStateMachine machine(place_locations, true);

  // lcm handle loop
  while (true) {
    // Handles all messages.
    while (lcm.handleTimeout(10) == 0) {}
    machine.Update(env_state, iiwa_callback, wsg_callback, &planner);
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
