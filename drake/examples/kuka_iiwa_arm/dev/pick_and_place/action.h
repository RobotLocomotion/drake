#pragma once

#include <memory>
#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/world_state.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

/**
 * Base class for actions used by the pick and place demo. E.g., moving the KUKA
 * iiwa arm, and opening / closing the gripper. The commands are packaged and
 * sent through LCM.
 */
class Action {
 public:
  explicit Action(lcm::LCM* lcm) : lcm_(lcm) { Reset(); }

  virtual ~Action() {}

  /**
   * Returns true if the action has finished based on the provided estimated
   * state.
   */
  virtual bool ActionFinished(const WorldState& est_state) const = 0;

  /**
   * Returns true if the action has failed based on the provided estimated
   * state.
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
 * A class that represents an action that sends a sequence of desired joint
 * positions through LCM to move the KUKA iiwa arm.
 */
class IiwaMove : public Action {
 public:
  /**
   * Constructs an Action class to move the iiwa arm.
   *
   * @param iiwa The KUKA iiwa model. Its life span must be longer than the
   * instance of this class.
   * @param channel Name of the LCM message channel containing the output
   * robotlocomotion::robot_plan_t.
   */
  IiwaMove(const RigidBodyTree<double>& iiwa, const std::string& channel,
           lcm::LCM* lcm)
      : Action(lcm), iiwa_(iiwa), pub_channel_(channel) {}

  ~IiwaMove() override {}

  /**
   * Returns a constant reference to the KUKA iiwa model.
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
   * Returns true if the time since the beginning of the action is longer than
   * the duration of the desired motion, and the arm stopped moving.
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

  ~WsgAction() override {}

  /**
   * Sends an LCM message that tells the WSG gripper driver to fully open.
   */
  void OpenGripper(const WorldState& est_state);

  /**
   * Sends an LCM message that tells the WSG gripper driver to fully close.
   */
  void CloseGripper(const WorldState& est_state);

  // TODO(siyuanfeng): Implement something meaningful here like a check for a
  // force threshold being crossed.
  bool ActionFailed(const WorldState& est_state) const override {
    return false;
  }

  /**
   * Returns true if the gripper stopped moving, and it is at least 0.5 seconds
   * after an Open / Close command was last issued.
   */
  bool ActionFinished(const WorldState& est_state) const override;

 private:
  const std::string pub_channel_;
};

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
