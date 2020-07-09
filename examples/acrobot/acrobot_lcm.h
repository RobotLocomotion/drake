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
/// @system
/// name: AcrobotStateReceiver
/// input_ports:
/// - lcmt_acrobot_x
/// output_ports:
/// - acrobot_state
/// @endsystem
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
/// @system
/// name : AcrobotCommandSender
/// input_ports:
/// - elbow_torque
/// output_ports:
/// - lcm_acrobot_u
/// @endsystem
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
/// @system
/// name: AcrobotCommandReceiver
/// input_ports:
/// - lcmt_acrobot_u
/// output_ports:
/// - elbow_torque
/// @endsystem
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
/// @system
/// name: AcrobotStateSender
/// input_ports:
/// - acrobot_state
/// output_ports:
/// - lcmt_acrobot_x
/// @endsystem
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
