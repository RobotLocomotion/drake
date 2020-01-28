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
 * Selects an active primitive from a set of MotionPrimitives. This system is
 * designed to work with systems::PortSwitch.
 *
 * There is one abstract output port of type systems::InputPortIndex that
 * identifies which motion primitive is active. There is one abstract input
 * port of type MotionSummary for each motion primitive. This is also a
 * discrete time system, and the states are:
 * - identifier for the active primitive.
 * - all the primitive's MotionSummary outputs from the last time step.
 *
 * At each time step, in the unrestricted update, we poll all the MotionSummary
 * outputs and compare its status field against what we stored in the abstract
 * state. If we see a change from anything not kExecuting to kExecuting, the
 * corresponding primitve will be set as the new active primitive. There is a
 * check for making sure only one such change can happen in each update.
 */
class MotionPrimitiveSelector : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MotionPrimitiveSelector)

  explicit MotionPrimitiveSelector(double timestep);

  const systems::OutputPort<double>& get_selection_output() const {
    return *selection_output_port_;
  }

  const systems::InputPort<double>& get_motion_summary_input(
      const std::string& motion_primitive_name) const;

  const systems::InputPort<double>& DeclareInputForMotionPrimitive(
      const MotionPrimitive& primitive);

 private:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const override;

  void CalcSelection(const systems::Context<double>& context,
                     systems::InputPortIndex* summary) const;

  const systems::OutputPort<double>* selection_output_port_;

  std::unordered_map<std::string, systems::InputPortIndex>
      primitive_name_to_input_index_;
  std::unordered_map<std::string, systems::AbstractStateIndex>
      primitive_name_to_state_index_;

  systems::AbstractStateIndex active_primitive_name_index_;
};

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake
