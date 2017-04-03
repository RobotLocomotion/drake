#pragma once

#include <memory>
#include <string>

#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_base.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

class GripperAction : public ActionPrimitive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperAction)

  explicit GripperAction(double desired_update_interval);

  const systems::InputPortDescriptor<double>& get_plan_input_port() const {
    return this->get_input_port(input_port_primitive_);
  }

  const systems::OutputPortDescriptor<double>& get_robot_plan_output_port()
      const {
    return this->get_output_port(plan_output_port);
  }

  virtual std::vector<std::unique_ptr<systems::AbstractValue>>
  AllocateExtendedAbstractState() const override;

  virtual std::unique_ptr<systems::AbstractValue>
  ExtendedAllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  void DoExtendedCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const override;

  void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const override;

 private:
  // InternalState relevant only to this primitive.
  struct InternalState;

  const int plan_output_port{};
  const int input_port_primitive_{};
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake