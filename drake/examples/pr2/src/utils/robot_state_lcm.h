#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_robot_state.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pr2 {

extern const double lcmStatusPeriod;

class RobotStateReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateReceiver)

  explicit RobotStateReceiver(int num_joints);

  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const VectorX<double>> x) const;

 private:
  void OutputCommand(const systems::Context<double>& context,
                     systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const int num_joints_;
};

class RobotStateSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateSender)

  explicit RobotStateSender(int num_joints);

  const systems::InputPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(1);
  }

 private:
  // This is the method to use for the output port allocator.
  lcmt_robot_state MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_robot_state* output) const;

  const int num_joints_;
};

}  // namespace pr2
}  // namespace examples
}  // namespace drake
