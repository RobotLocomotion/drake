#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPControllerSystem::QPControllerSystem(const RigidBodyTree<double>& robot,
                                       double dt)
    : robot_(robot), control_dt_(dt) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  input_port_index_qp_input_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_output_ = DeclareAbstractOutputPort().get_index();

  set_name("qp_controller");
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);
}

void QPControllerSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Copies QpInput from AbstractState.
  QpOutput& qp_output = output->GetMutableData(output_port_index_qp_output_)
                          ->GetMutableValue<QpOutput>();
  qp_output =
      context.get_abstract_state<QpOutput>(abstract_state_qp_output_index_);
}

void QPControllerSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Inputs:
  const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  const QpInput* qp_input =
      EvalInputValue<QpInput>(context, input_port_index_qp_input_);

  // Calls the controller.
  QpOutput& qp_output = get_mutable_qp_output(state);

  if (qp_controller_.Control(*rs, *qp_input, &qp_output) < 0) {
    std::stringstream err;
    err << rs->position().transpose() << "\n";
    err << rs->velocity().transpose() << "\n";
    err << *qp_input << std::endl;
    throw std::runtime_error("QPControllerSystem: QP cannot solve\n" +
                             err.str());
  }
}

std::unique_ptr<systems::AbstractState>
QPControllerSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(1);
  abstract_vals[abstract_state_qp_output_index_] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<QpOutput>(QpOutput(GetDofNames(robot_))));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue>
QPControllerSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_qp_output_);

  return systems::AbstractValue::Make<QpOutput>(QpOutput(GetDofNames(robot_)));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
