#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"

#include <memory>
#include <utility>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPControllerSystem::QPControllerSystem(const RigidBodyTree<double>& robot)
    : robot_(robot) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  input_port_index_qp_input_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

  // Declare discrete time controller.
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  DRAKE_ASSERT(this->get_num_input_ports() == 2);
  DRAKE_ASSERT(this->get_num_output_ports() == 1);

  set_name("qp_controller");
}

void QPControllerSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  systems::AbstractState* abs_state = state->get_mutable_abstract_state();
  DRAKE_DEMAND(abs_state->size() == 1);

  // Gets the controller from abstract state.
//  QPController& qp_controller =
//      abs_state->get_mutable_abstract_state(0).GetMutableValue<QPController>();
  QpOutput& qp_output =
      abs_state->get_mutable_abstract_state(0).GetMutableValue<QpOutput>();

  // Inputs:
  const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  const QpInput* qp_input =
      EvalInputValue<QpInput>(context, input_port_index_qp_input_);

  if (qp_controller_.Control(*rs, *qp_input, &qp_output) < 0) {
    std::cout << rs->position().transpose() << std::endl;
    std::cout << rs->velocity().transpose() << std::endl;
    std::cout << *qp_input << std::endl;
    throw std::runtime_error("System2QP: QP cannot solve\n");
  }
}

void QPControllerSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Output:
  QpOutput& qp_output = output->GetMutableData(output_port_index_qp_input_)
                            ->GetMutableValue<QpOutput>();

  qp_output = context.get_abstract_state<QpOutput>(0);
}

std::unique_ptr<systems::AbstractState>
QPControllerSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.reserve(1);
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<QpOutput>(QpOutput(GetDofNames(robot_)))));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

std::unique_ptr<systems::SystemOutput<double>>
QPControllerSystem::AllocateOutput(
    const systems::Context<double>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<double>> output(
      new systems::LeafSystemOutput<double>);
  output->add_port(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<QpOutput>(QpOutput(GetDofNames(robot_)))));
  return std::move(output);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
