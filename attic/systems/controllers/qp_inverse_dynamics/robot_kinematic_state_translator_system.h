#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/qp_inverse_dynamics/deprecated.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

/**
 * Provides transformation from a state vector to RobotKinematicState.
 */
template <typename T>
class DRAKE_DEPRECATED_QPID RobotKinematicStateTranslatorSystem
    : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotKinematicStateTranslatorSystem)

  /**
   * Constructor.
   * @param robot Pointer to a RigidBodyTree. The lifespan of @p robot must
   * be longer than this object.
   */
  explicit RobotKinematicStateTranslatorSystem(const RigidBodyTree<T>* robot)
      : robot_(*robot), default_output_(robot) {
    const int kDim = robot->get_num_positions() + robot->get_num_velocities();
    this->DeclareInputPort(systems::kVectorValued, kDim).get_index();
    this->template DeclareAbstractOutputPort<
        RobotKinematicStateTranslatorSystem, RobotKinematicState<T>>(
        default_output_, &RobotKinematicStateTranslatorSystem::Translate);
  }

  /**
   * Returns the output port.
   */
  const OutputPort<T>& get_output_port() const {
    return System<T>::get_output_port(0);
  }

  /**
   * Returns the input port for the state vector.
   */
  const InputPort<T>& get_input_port() const {
    return System<T>::get_input_port(0);
  }

  /**
   * Returns a const reference to the RigidBodyTree.
   */
  const RigidBodyTree<T>& get_robot() const { return robot_; }

 private:
  void Translate(const systems::Context<T>& context,
                 RobotKinematicState<T>* output) const {
    const VectorX<T> x = this->EvalEigenVectorInput(context, 0);

    const int kPosDim = get_robot().get_num_positions();
    const int kVelDim = get_robot().get_num_velocities();
    output->UpdateKinematics(context.get_time(), x.head(kPosDim),
                             x.tail(kVelDim));
  }

  const RigidBodyTree<T>& robot_;
  const RobotKinematicState<T> default_output_;

  int output_port_index_robot_kinematic_state_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
