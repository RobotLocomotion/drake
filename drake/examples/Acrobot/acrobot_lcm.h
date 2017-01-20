#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to acrobot. The functions in this file are based on
/// iiwa_lcm.h

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace acrobot {


class AcrobotStateReceiver : public systems::LeafSystem<double> {
 public:
  AcrobotStateReceiver();

  void DoCalcOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override;
};


class AcrobotCommandSender : public systems::LeafSystem<double> {
 public:
  AcrobotCommandSender();

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  void DoCalcOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override;
};

class AcrobotCommandReceiver : public systems::LeafSystem<double> {
 public:
  AcrobotCommandReceiver();

  void DoCalcOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override;
};


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
