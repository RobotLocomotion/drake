#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitive_base.h"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// An `ActionPrimitive` for commanding the Schunk WSG Gripper.
/// It maintains an internal state and switches based on input.
/// Note that if the internal state is waiting, the output port is garbage,
/// the down stream systems need to be aware.
class GripperAction : public ActionPrimitive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperAction)

  explicit GripperAction(double desired_update_interval);

  /// This gets the abstract input port corresponding to the
  /// `GripperActionInput`.
  const systems::InputPortDescriptor<double>& get_primitive_input_port() const {
    return this->get_input_port(input_port_primitive_input_);
  }

  /// This gets the abstract output port corresponding to the
  /// `lcmt_schunk_wsg_command`.
  const systems::OutputPort<double>& get_robot_plan_output_port()
      const {
    return this->get_output_port(plan_output_port);
  }

 protected:
  std::vector<std::unique_ptr<systems::AbstractValue>>
  AllocateExtendedAbstractState() const override;

  void SetExtendedDefaultState(
      const systems::Context<double>& context,
      systems::State<double>* state) const override;

  void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const override;

 private:
  lcmt_schunk_wsg_command MakePlanOutput() const;
  void OutputCurrentPlan(const systems::Context<double>& context,
                         lcmt_schunk_wsg_command* wsg_plan_output) const;

  // InternalState relevant only to this primitive.
  struct InternalState;

  const unsigned int internal_state_index_{0};
  const int plan_output_port{-1};
  const int input_port_primitive_input_{-1};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
