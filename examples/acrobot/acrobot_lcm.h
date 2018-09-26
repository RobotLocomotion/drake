#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to acrobot. The classes in this file are based on
/// iiwa_lcm.h

#include <memory>

#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace acrobot {

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// acrobot state channel with LCM type lcmt_acrobot_x, and outputs the
/// acrobot states as an AcrobotState.
///
/// @system{ AcrobotStateReceiver,
///   @input_port{lcmt_acrobot_x},
///   @output_port{acrobot_state} }
///
/// @ingroup acrobot_systems
class AcrobotStateReceiver : public systems::LeafSystem<double> {
 public:
  AcrobotStateReceiver();

 private:
  void CopyStateOut(const systems::Context<double>& context,
                    AcrobotState<double>* output) const;
};

/// Receives the output of an acrobot controller, and outputs it as an LCM
/// message with type lcm_acrobot_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
///
/// @system{ AcrobotCommandSender,
///   @input_port{elbow_torque},
///   @output_port{lcm_acrobot_u} }
///
/// @ingroup acrobot_systems
class AcrobotCommandSender : public systems::LeafSystem<double> {
 public:
  AcrobotCommandSender();

 private:
  void OutputCommand(const systems::Context<double>& context,
                     lcmt_acrobot_u* output) const;
};

/// Receives the output of an LcmSubscriberSystem that subscribes to the
/// acrobot input channel with LCM type lcmt_acrobot_u, and outputs the
/// acrobot input as a BasicVector.
///
/// @system{ AcrobotCommandReceiver,
///   @input_port{lcmt_acrobot_u},
///   @output_port{elbow_torque} }
///
/// @ingroup acrobot_systems
class AcrobotCommandReceiver : public systems::LeafSystem<double> {
 public:
  AcrobotCommandReceiver();

 private:
  void OutputCommandAsVector(const systems::Context<double>& context,
                             systems::BasicVector<double>* output) const;
};

/// Receives the output of an acrobot_plant, and outputs it as an LCM
/// message with type lcm_acrobot_x. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
///
/// @system{ AcrobotStateSender,
///   @input_port{acrobot_state},
///   @output_port{lcmt_acrobot_x} }
///
/// @ingroup acrobot_systems
class AcrobotStateSender : public systems::LeafSystem<double> {
 public:
  AcrobotStateSender();

 private:
  void OutputState(const systems::Context<double>& context,
                   lcmt_acrobot_x* output) const;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
