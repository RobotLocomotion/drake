#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"

#include <memory>
#include <vector>
#include <utility>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPControllerSystem::QPControllerSystem(const RigidBodyTree<double>& robot)
    : robot_(robot) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  input_port_index_qp_input_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_output_ = DeclareAbstractOutputPort().get_index();

  set_name("qp_controller");
}

void QPControllerSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Inputs:
  const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  const QpInput* qp_input =
      EvalInputValue<QpInput>(context, input_port_index_qp_input_);

  // Output:
  QpOutput& qp_output = output->GetMutableData(output_port_index_qp_output_)
                            ->GetMutableValue<QpOutput>();

  if (qp_controller_.Control(*rs, *qp_input, &qp_output) < 0) {
    std::stringstream err;
    err << rs->position().transpose() << "\n";
    err << rs->velocity().transpose() << "\n";
    err << *qp_input << std::endl;
    throw std::runtime_error(
        "QPControllerSystem: QP cannot solve\n" + err.str());
  }
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
