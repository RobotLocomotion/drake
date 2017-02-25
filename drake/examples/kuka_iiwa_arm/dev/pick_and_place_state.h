#pragma once

#include <lcm/lcm-cpp.hpp>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"

#include "bot_core/robot_state_t.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

/**
 * A class that represents the iiwa pick and place world, which contains a
 * kuka iiwa arm, a schunk wsg gripper and an object that is being
 * manipulated. These states are updated through LCM messages.
 */
class EnvState {
 public:
  /**
   * Constructs an EnvState object that holds the states represent a pick and
   * place scenario.
   */
  EnvState(const std::string& iiwa_model_path,
           const std::string end_effector_name, lcm::LCM* lcm);

  virtual ~EnvState();

  /**
   * Adds a lcm callback listening to @p channel to process iiwa status.
   */
  void SubscribeToIiwaStatus(const std::string& channel);

  /**
   * Adds a lcm callback listening to @p channel to process wsg gripper status.
   */
  void SubscribeToWsgStatus(const std::string& channel);

  /**
   * Adds a lcm callback listening to @p channel to process object status.
   */
  void SubscribeToObjectStatus(const std::string& channel);

  double get_iiwa_time() const { return iiwa_time_; }
  double get_wsg_time() const { return wsg_time_; }
  double get_obj_time() const { return obj_time_; }
  const Isometry3<double>& get_object_pose() const { return obj_pose_; }
  const Vector6<double>& get_object_velocity() const { return obj_vel_; }
  const Isometry3<double>& get_iiwa_base() const { return iiwa_base_; }
  const Isometry3<double>& get_iiwa_end_effector_pose() const {
    return iiwa_ee_pose_;
  }
  const Vector6<double>& get_iiwa_end_effector_velocity() const {
    return iiwa_ee_vel_;
  }
  const VectorX<double>& get_iiwa_q() const { return iiwa_q_; }
  const VectorX<double>& get_iiwa_v() const { return iiwa_v_; }
  double get_wsg_q() const { return wsg_q_; }
  double get_wsg_v() const { return wsg_v_; }

  const RigidBodyTree<double>& get_iiwa() const { return *iiwa_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Handles iiwa states from the lcm message.
  void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const bot_core::robot_state_t* iiwa_msg);

  // Handles wsg states from the lcm message.
  void HandleWsgStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const lcmt_schunk_wsg_status* wsg_msg);

  // Handles object states from the lcm message.
  void HandleObjectStatus(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan,
                          const bot_core::robot_state_t* obj_msg);

  // Since we can't initialize the RBT unless we know where its base is,
  // which is coming from lcm. So it's easier for us to own a model internally.
  std::unique_ptr<RigidBodyTree<double>> iiwa_;
  const std::string iiwa_model_path_;
  const std::string ee_name_;
  const RigidBody<double>* end_effector_{nullptr};

  // Iiwa status
  double iiwa_time_;
  Isometry3<double> iiwa_base_;
  VectorX<double> iiwa_q_;
  VectorX<double> iiwa_v_;
  Isometry3<double> iiwa_ee_pose_;
  Vector6<double> iiwa_ee_vel_;

  // Gripper status
  double wsg_time_;
  double wsg_q_;  // units [m]
  double wsg_v_;  // units [m/s]
  double wsg_force_;

  // Object status
  double obj_time_;
  Isometry3<double> obj_pose_;
  Vector6<double> obj_vel_;

  // Lcm subscription management
  lcm::LCM* lcm_;
  std::list<lcm::Subscription*> lcm_subscriptions_;
};

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
  virtual bool ActionFinished(const EnvState& est_state) const = 0;

  /**
   * Returns true if the action has failed given estimated state.
   */
  virtual bool ActionFailed(const EnvState& est_state) const = 0;

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
  void MoveJoints(const EnvState& est_state, const std::vector<double>& time,
                  const std::vector<VectorX<double>>& q);

  /**
   * Resets all internal states including IK waypoints solutions.
   */
  void Reset() override;

  // TODO(siyuanfeng): have something meaningful here, like the object slipped
  // out.
  bool ActionFailed(const EnvState& est_state) const override { return false; }

  /**
   * Returns ture if time since beginning of action is longer than then the
   * duration of the desired motion, and the arm stopped moving.
   */
  bool ActionFinished(const EnvState& est_state) const override;

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
  void OpenGripper(const EnvState& est_state);

  /*
   * Sends a lcm message that tells the WSG gripper driver to close all the way.
   */
  void CloseGripper(const EnvState& est_state);

  // TODO(siyuanfeng): have something meaningful here, like check for force
  // threshold.
  bool ActionFailed(const EnvState& est_state) const { return false; }

  /**
   * Returns true if the gripper stopped moving, and it is at least 0.5 second
   * after
   * a Open / Close command was last issued.
   */
  bool ActionFinished(const EnvState& est_state) const;

 private:
  const std::string pub_channel_;
};

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
