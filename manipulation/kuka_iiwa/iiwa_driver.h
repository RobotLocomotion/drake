#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

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

  /** Per BuildIiwaControl. */
  double ext_joint_filter_tau{0.01};

  /// Per BuildIiwaControl. Valid options are:
  /// - "position" with optional "torque" feedforward
  /// - "torque"
  std::vector<std::string> control_mode{{"position"}, {"torque"}};

  std::string lcm_bus{"default"};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(hand_model_name));
    a->Visit(DRAKE_NVP(ext_joint_filter_tau));
    a->Visit(DRAKE_NVP(control_mode));
    a->Visit(DRAKE_NVP(lcm_bus));
  }
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
