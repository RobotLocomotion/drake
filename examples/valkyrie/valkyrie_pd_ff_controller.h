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

  inline const OutputPort<double>& get_output_port_atlas_command()
      const {
    return get_output_port(output_port_index_atlas_command_);
  }

 private:
  // This is the calculator method for the output port.
  void OutputCommand(const Context<double>& context,
                     bot_core::atlas_command_t* output) const;

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
