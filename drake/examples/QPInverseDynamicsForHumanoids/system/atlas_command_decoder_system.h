#pragma once

#include "bot_core/atlas_command_t.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::SystemPortDescriptor;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::Value;
using systems::BasicVector;

/**
 * A stub translator from bot_core::robot_state_t to inputs for the simulator
 * (which is a raw vector of generalized acceleration for the dummy simulator
 * in this PR). #4004 has implementation of similar functionalities, and will
 * replace this.
 *
 * This acceleration is NOT supposed to be part of the
 * bot_core::robot_state_t, and the real inputs should have been torques, but
 * that requires a real simulator.
 *
 * Input: lcm message bot_core::robot_state_t
 * Output: raw vector of generalized acceleration.
 */
class AtlasCommandDecoderSystem : public systems::LeafSystem<double> {
 public:
  explicit AtlasCommandDecoderSystem(const RigidBodyTree<double>& robot)
      : robot_(robot) {
    input_port_index_lcm_msg_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();

    output_port_index_vd_ =
        DeclareOutputPort(systems::kVectorValued, robot_.get_num_velocities(),
                          systems::kInheritedSampling).get_index();
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // Input:
    const bot_core::atlas_command_t* msg =
        EvalInputValue<bot_core::atlas_command_t>(context,
                                                  input_port_index_lcm_msg_);
    // Output:
    BasicVector<double>* vd =
        output->GetMutableVectorData(output_port_index_vd_);

    // Set to zero acceleration before getting any commands from the
    // controller.
    for (int i = 0; i < robot_.get_num_velocities(); ++i)
      vd->SetAtIndex(i, 0.0);

    if (msg->velocity.size() != 0 &&
        static_cast<int>(msg->velocity.size()) != robot_.get_num_velocities()) {
      throw std::runtime_error("invalid acceleration dimension.");
    }
    // Assuming the the velocity term contains acceleration information
    // computed by the qp controller, and the acceleration is in the same order
    // as robot_'s.
    for (size_t i = 0; i < msg->velocity.size(); ++i) {
      vd->SetAtIndex(i, static_cast<double>(msg->velocity[i]));
    }
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);

    output->get_mutable_ports()->emplace_back(
        new systems::OutputPort(AllocateOutputVector(get_output_port_vd())));

    return std::move(output);
  }

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const SystemPortDescriptor<double>& get_input_port_atlas_command_msg()
      const {
    return get_input_port(input_port_index_lcm_msg_);
  }

  /**
   * @return Port for the output: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_output_port_vd() const {
    return get_output_port(output_port_index_vd_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int input_port_index_lcm_msg_;
  int output_port_index_vd_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
