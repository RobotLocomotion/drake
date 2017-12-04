#include "drake/systems/controllers/qp_inverse_dynamics/qp_output_translator_system.h"

#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

QpOutputTranslatorSystem::QpOutputTranslatorSystem(
    const RigidBodyTree<double>& robot)
    : robot_(robot) {
  input_port_index_qp_output_ = DeclareAbstractInputPort().get_index();
  output_port_index_torque_ =
      DeclareVectorOutputPort(
          systems::BasicVector<double>(robot_.get_num_actuators()),
          &QpOutputTranslatorSystem::CalcActuationTorques)
          .get_index();
}

void QpOutputTranslatorSystem::CalcActuationTorques(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  // Input:
  const QpOutput* qp_output =
      EvalInputValue<QpOutput>(context, input_port_index_qp_output_);

  // Output:
  auto out_vector = output->get_mutable_value();

  // TODO(sherm1) This computation should be cached so it will be evaluated
  // when first requested and then available for re-use.
  out_vector = robot_.B.transpose() * qp_output->dof_torques();

  DRAKE_ASSERT(out_vector.size() == robot_.get_num_actuators());
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
