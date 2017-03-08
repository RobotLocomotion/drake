#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

JointLevelControllerBaseSystem::JointLevelControllerBaseSystem(
    const RigidBodyTree<double>& robot)
    : robot_(robot) {
  input_port_index_qp_output_ = DeclareAbstractInputPort().get_index();
  output_port_index_torque_ =
      DeclareOutputPort(systems::kVectorValued, robot_.get_num_actuators())
          .get_index();
}

void JointLevelControllerBaseSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Input:
  const QpOutput* qp_output =
      EvalInputValue<QpOutput>(context, input_port_index_qp_output_);

  // Output:
  auto out_vector = GetMutableOutputVector(output, output_port_index_torque_);

  out_vector = robot_.B.transpose() * qp_output->dof_torques();

  DRAKE_ASSERT(out_vector.size() == robot_.get_num_actuators());

  // Call Derived class's extended methods.
  DoCalcExtendedOutput(context, output);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
