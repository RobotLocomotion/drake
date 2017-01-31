#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to acrobot. The classes in this file are based on
/// iiwa_lcm.h
#include <memory>
#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace acrobot {

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// acrobot state channel with LCM type lcmt_acrobot_x, and outputs the
/// acrobot states as an AcrobotStateVector.
class AcrobotStateReceiver : public systems::LeafSystem<double> {
 public:
  AcrobotStateReceiver();

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector(
      const systems::OutputPortDescriptor<double>& descriptor) const override;
};

/// Receives the output of an acrobot controller, and outputs it as an LCM
/// message with type lcm_acrobot_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class AcrobotCommandSender : public systems::LeafSystem<double> {
 public:
  AcrobotCommandSender();

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// acrobot input channel with LCM type lcmt_acrobot_u, and outputs the
/// acrobot input as a BasicVector.
class AcrobotCommandReceiver : public systems::LeafSystem<double> {
 public:
  AcrobotCommandReceiver();

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;
};

/// Receives the output of an acrobot_plant, and outputs it as an LCM
/// message with type lcm_acrobot_x. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class AcrobotStateSender : public systems::LeafSystem<double> {
 public:
  AcrobotStateSender();

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
