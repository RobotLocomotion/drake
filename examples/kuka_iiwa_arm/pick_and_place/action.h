#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"
#include "drake/lcmt_schunk_wsg_command.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/**
 * Base class for actions used by the pick and place demo. E.g.,
 * moving the KUKA iiwa arm, and opening / closing the gripper. The
 * commands generate LCM messages.
 */
class Action {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Action)

  virtual ~Action();

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
   * Returns true if the action started.
   */
  virtual bool ActionStarted() const;

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
  virtual void Reset();

 protected:
  Action() { Reset(); }

  /// Set the action initial time.
  void StartAction(double start_time);

 private:
  double act_start_time_{};
};

/**
 * A class that represents an action that sends a sequence of desired joint
 * positions through LCM to move the KUKA iiwa arm.
 */
class IiwaMove : public Action {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IiwaMove)

  IiwaMove();

  /**
   * Populates @p plan with a robot plan that moves the iiwa arm
   * described by @p iiwa through the joint positions in @p q at the
   * times in @p time.  (Note: @p time and @p q must be of the same
   * length).
   */
  void MoveJoints(const WorldState& est_state,
                  const RigidBodyTree<double>& iiwa,
                  const std::vector<double>& time,
                  const std::vector<VectorX<double>>& q,
                  robotlocomotion::robot_plan_t* plan);

  void Reset() override;

  // TODO(siyuanfeng): have something meaningful here, like the object slipped
  // out.
  bool ActionFailed(const WorldState&) const override {
    return false;
  }

  /**
   * Returns true if the time since the beginning of the action is longer than
   * the duration of the desired motion, and the arm stopped moving.
   */
  bool ActionFinished(const WorldState& est_state) const override;

 private:
  double duration_{};
};

/**
 * An Action that closes / opens the gripper.
 */
class WsgAction : public Action {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WsgAction)

  WsgAction();

  /**
   * Populates @p msg with an LCM message that tells the WSG gripper
   * driver to fully open.
   */
  void OpenGripper(const WorldState& est_state,
                   lcmt_schunk_wsg_command* msg);

  /**
   * Populates @p msg with an LCM message that tells the WSG gripper
   * driver to fully close.
   */
  void CloseGripper(const WorldState& est_state,
                    lcmt_schunk_wsg_command* msg);

  // TODO(siyuanfeng): Implement something meaningful here like a check for a
  // force threshold being crossed.
  bool ActionFailed(const WorldState&) const override {
    return false;
  }

  /**
   * Returns true if the following criteria are satisfied:
   *  - The gripper speed is less than the final speed threshold.
   *  - 0.5 s have elapsed since the last Open/Close command was issued.
   *  - The gripper position is greater than (for an Open command) or less than
   *    (for a Close command) the open position threshold.
   */
  bool ActionFinished(const WorldState& est_state) const override;

 private:
  enum { kOpen, kClose } last_command_{kOpen};
  static constexpr double kFinalSpeedThreshold = 1e-2;  // m/s
  static constexpr double kOpenPositionThreshold = .095;   // m
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
