#pragma once

#include <string>
#include <tuple>
#include <unordered_map>

#include "drake/manipulation/robot_bridge/motion_primitive.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

/**
 * Basically selects and forwards the outputs from a set of MotionPrimitives.
 * This is also a discrete time system. The states are:
 * - identifier for the active primitive (whose outputs are forwarded)
 * - all the primitive's MotionSummary outputs from the last time step.
 *
 * Active primitive's ouputs are forwarded in the CalcOutputs methods.
 * At each time step, in the unrestricted update, we poll all the MotionSummary
 * outputs and compare its status field against what we stored in the abstract
 * state. If we see a change from anything not kExecuting to kExecuting, the
 * corresponding primitve will be set as the new active primitive. There is a
 * check for making sure only one such change can happen in each update.
 */
class MotionPrimitiveSelector : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MotionPrimitiveSelector)

  explicit MotionPrimitiveSelector(int num_q, double timestep);

  void Finalize();

  const systems::OutputPort<double>& get_position_output() const {
    return *position_output_port_;
  }

  const systems::OutputPort<double>& get_torque_output() const {
    return *torque_output_port_;
  }

  const systems::OutputPort<double>& get_motion_summary_output() const {
    return *summary_output_port_;
  }

  const systems::InputPort<double>& get_motion_summary_input(
      const std::string& motion_primitive_name) const;

  const systems::InputPort<double>& get_position_input(
      const std::string& motion_primitive_name) const;

  const systems::InputPort<double>& get_torque_input(
      const std::string& motion_primitive_name) const;

  std::tuple<const systems::InputPort<double>*,
             const systems::InputPort<double>*,
             const systems::InputPort<double>*>
  DeclareInputs(const MotionPrimitive& primitive);

 private:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const override;

  void CalcTorqueOutput(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

  void CalcPositionOutput(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;

  void CalcMotionSummary(const systems::Context<double>& context,
                         MotionSummary* summary) const;

  std::unordered_map<std::string, systems::InputPortIndex>
      primitive_name_to_input_index_;
  std::unordered_map<std::string, systems::AbstractStateIndex>
      primitive_name_to_state_index_;

  const int num_q_;
  const systems::OutputPort<double>* position_output_port_;
  const systems::OutputPort<double>* torque_output_port_;
  const systems::OutputPort<double>* summary_output_port_;

  systems::AbstractStateIndex active_primitive_name_index_;

  bool finalized_{false};
};

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake
