#pragma once

#include <memory>
#include <utility>

#include "lcmtypes/bot_core/atlas_command_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

class ValkyriePDAndFeedForwardController : public systems::LeafSystem<double> {
 public:
  ValkyriePDAndFeedForwardController(const RigidBodyTree<double>& robot,
                         const VectorX<double>& nominal_position,
                         const VectorX<double>& nominal_torque,
                         const VectorX<double>& Kp, const VectorX<double>& Kd);

  inline const InputPortDescriptor<double>& get_input_port_kinematics_result()
      const {
    return get_input_port(input_port_index_kinematics_result_);
  }

  inline const OutputPortDescriptor<double>& get_output_port_atlas_command()
      const {
    return get_output_port(output_port_index_atlas_command_);
  }

 protected:
  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<double>& descriptor) const override {
    return std::make_unique<Value<bot_core::atlas_command_t>>(
        bot_core::atlas_command_t());
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
