#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
class KukaTorqueController
    : public systems::Diagram<T>,
      public systems::controllers::StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KukaTorqueController)
  KukaTorqueController(std::unique_ptr<RigidBodyTree<T>> tree,
                       const VectorX<double>& kp,
                       const VectorX<double>& damping);

  const systems::InputPort<T>& get_input_port_estimated_state() const override {
    return systems::Diagram<T>::get_input_port(
        input_port_index_estimated_state_);
  }

  /** Provides the desired state for the controller.
    * Note that the controller is emulating a spring-damper system at each of
    * the joints and ignores the velocity inputs. */
  const systems::InputPort<T>& get_input_port_desired_state() const override {
    return systems::Diagram<T>::get_input_port(input_port_index_desired_state_);
  }

  const systems::InputPort<T>& get_input_port_commanded_torque() const {
    return systems::Diagram<T>::get_input_port(
        input_port_index_commanded_torque_);
  }

  const systems::OutputPort<T>& get_output_port_control() const override {
    return systems::Diagram<T>::get_output_port(output_port_index_control_);
  }

 private:
  void SetUp(const VectorX<double>& kp, const VectorX<double>& damping);
  std::unique_ptr<RigidBodyTree<T>> robot_for_control_{nullptr};
  int input_port_index_estimated_state_{-1};
  int input_port_index_desired_state_{-1};
  int input_port_index_commanded_torque_{-1};
  int output_port_index_control_{-1};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
