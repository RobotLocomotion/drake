#pragma once

#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <string>
#include <vector>

#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/world_state.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

/**
 * Base class that represents actions for the pick and place demo. E.g. moving
 * iiwa arm, open / close gripper. The commands are packaged and sent through
 * LCM.
 */
class Action {
 public:
  explicit Action(lcm::LCM* lcm) : lcm_(lcm) { Reset(); }

  /**
   * Returns true if the action has finished given estimated state.
   */
  virtual bool ActionFinished(const WorldState& est_state) const = 0;

  /**
   * Returns true if the action has failed given estimated state.
   */
  virtual bool ActionFailed(const WorldState& est_state) const = 0;

  /**
   * Returns true if the action has started.
   */
  virtual bool ActionStarted() const {
    if (act_start_time_ < 0) return false;
    return true;
  }

  /**
   * Returns the time when the action has started.
   */
  double get_action_start_time() const { return act_start_time_; }

  /**
   * Returns elapsed time since beginning of the action.
   */
  double get_time_since_action_start(double time) const {
    if (ActionStarted())
      return time - act_start_time_;
    else
      return 0;
  }

  /**
   * Resets the action's internal states.
   */
  virtual void Reset() { act_start_time_ = -1; }

 protected:
  // Sets the action initial time.
  virtual void StartAction(double start_time) {
    DRAKE_DEMAND(start_time >= 0);
    act_start_time_ = start_time;
  }

  lcm::LCM* lcm_;

 private:
  double act_start_time_;
};

/**
 * A class that represents action that sends a sequence of desired joint
 * positions through LCM to move the iiwa arm.
 */
class IiwaMove : public Action {
 public:
  /**
   * Constructs an Action class to move the iiwa arm.
   * @param iiwa Reference to a RigidBodyTree that represents the iiwa robot.
   * Its life span must be longer than this instance.
   * @param channel Lcm message channel name of the output
   * robotlocomotion::robot_plan_t.
   */
  IiwaMove(const RigidBodyTree<double>& iiwa, const std::string& channel,
           lcm::LCM* lcm)
      : Action(lcm), iiwa_(iiwa), pub_channel_(channel) {}

  /**
   * Returns a constant reference to the iiwa model.
   */
  const RigidBodyTree<double>& get_iiwa() const { return iiwa_; }

  /**
   * Sends a LCM message that moves the iiwa arm through the joint positions
   * in @p q at time @p time.
   */
  void MoveJoints(const WorldState& est_state, const std::vector<double>& time,
                  const std::vector<VectorX<double>>& q);

  /**
   * Resets all internal states including IK waypoints solutions.
   */
  void Reset() override;

  // TODO(siyuanfeng): have something meaningful here, like the object slipped
  // out.
  bool ActionFailed(const WorldState& est_state) const override {
    return false;
  }

  /**
   * Returns ture if time since beginning of action is longer than then the
   * duration of the desired motion, and the arm stopped moving.
   */
  bool ActionFinished(const WorldState& est_state) const override;

 private:
  const RigidBodyTree<double>& iiwa_;
  const std::string pub_channel_;
  double finish_time_;
};

/**
 * An Action that closes / open the gripper.
 */
class WsgAction : public Action {
 public:
  WsgAction(const std::string& channel, lcm::LCM* lcm)
      : Action(lcm), pub_channel_(channel) {}

  /*
   * Sends a lcm message that tells the WSG gripper driver to open all the way.
   */
  void OpenGripper(const WorldState& est_state);

  /*
   * Sends a lcm message that tells the WSG gripper driver to close all the way.
   */
  void CloseGripper(const WorldState& est_state);

  // TODO(siyuanfeng): have something meaningful here, like check for force
  // threshold.
  bool ActionFailed(const WorldState& est_state) const { return false; }

  /**
   * Returns true if the gripper stopped moving, and it is at least 0.5 second
   * after
   * a Open / Close command was last issued.
   */
  bool ActionFinished(const WorldState& est_state) const;

 private:
  const std::string pub_channel_;
};

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
