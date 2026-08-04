#pragma once

#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/** This config struct specifies how to wire up Drake systems between an LCM
interface and the actuation input ports of a MultibodyPlant. This simulates the
role that driver software and control cabinets would take in real life.

It creates an LCM publisher on the `IIWA_STATUS` channel and an LCM subscriber
on the `IIWA_COMMAND` channel. */
struct IiwaDriver {
  /** The name of the model (`name` element of the `add_model` directive) in
  the simulation that the driver will analyze to compute end effector inertia
  for its copy of the arm in inverse dynamics. */
  std::string hand_model_name;

  /** A time constant used to low-pass filter external torque inputs. */
  double ext_joint_filter_tau{0.01};

  /** Optionally pass in gains corresponding to the Iiwa Dof (7) in the
  controller.  If no value is passed, the gains derived from hardware will be
  used instead (hardcoded within the implementations of functions accepting
  this struct). These gains must be nullopt if `control_mode` does not include
  position control. */
  std::optional<Eigen::VectorXd> desired_kp_gains;

  /** The driver's control mode. Valid options (per ParseIiwaControlMode) are:
  - "position_only"
  - "position_and_torque" (default)
  - "torque_only" */
  std::string control_mode{"position_and_torque"};

  /** Optionally give an alternative frame on the arm model for its weld point
   to the world. If not supplied, the `child_frame_name` in the arm's
   ModelInstanceInfo will be used. */
  std::optional<std::string> arm_child_frame_name;

  /** Optionally give an alternative frame on the arm model for its weld point
  to the gripper. If not supplied, the `parent_frame_name` in the gripper's
  ModelInstanceInfo will be used. */
  std::optional<std::string> gripper_parent_frame_name;

  std::string lcm_bus{"default"};

  /** The period in seconds at which status reports are expected. */
  double lcm_status_period{kIiwaLcmStatusPeriod};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(hand_model_name));
    a->Visit(DRAKE_NVP(ext_joint_filter_tau));
    a->Visit(DRAKE_NVP(desired_kp_gains));
    a->Visit(DRAKE_NVP(control_mode));
    a->Visit(DRAKE_NVP(arm_child_frame_name));
    a->Visit(DRAKE_NVP(gripper_parent_frame_name));
    a->Visit(DRAKE_NVP(lcm_bus));
    a->Visit(DRAKE_NVP(lcm_status_period));
  }
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
