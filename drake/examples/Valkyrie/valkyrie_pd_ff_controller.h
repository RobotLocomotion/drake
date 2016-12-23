#pragma once

#include "lcmtypes/bot_core/atlas_command_t.hpp"

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

class ValkyriePDAndFeedForwardController : public systems::LeafSystem<double> {
 public:
  ValkyriePDAndFeedForwardController(const RigidBodyTree<double>& robot,
                         const VectorX<double>& nominal_position,
                         const VectorX<double>& nominal_torque,
                         const VectorX<double>& Kp, const VectorX<double>& Kd);

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);

    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<bot_core::atlas_command_t>(bot_core::atlas_command_t())));
    return std::move(output);
  }

  inline const SystemPortDescriptor<double>& get_input_port_kinematics_result()
      const {
    return get_input_port(input_port_index_kinematics_result_);
  }

  inline const SystemPortDescriptor<double>& get_output_port_atlas_command()
      const {
    return get_output_port(output_port_index_atlas_command_);
  }

 private:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override;

  const RigidBodyTree<double>& robot_;
  int input_port_index_kinematics_result_;
  int output_port_index_atlas_command_;

  VectorX<double> desired_position_;
  VectorX<double> feedforward_torque_;
  VectorX<double> Kp_;
  VectorX<double> Kd_;
};

}  // namespace systems
}  // namespace drake
