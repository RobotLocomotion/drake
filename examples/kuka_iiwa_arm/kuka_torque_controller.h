#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

// N.B. Inheritance order must remain fixed for pydrake (#9243).
/**
 * Controller that take emulates the kuka_iiwa_arm when operated in torque
 * control mode. The controller specifies a stiffness and damping ratio at each
 * of the joints. Because the critical damping constant is a function of the
 * configuration the damping is non-linear. See
 * https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/kuka-driver/sunrise_1.11/DrakeFRITorqueDriver.java
 *
 * for details on the low-level controller. Note that the
 * input_port_desired_state() method takes a full state for convenient wiring
 * with other Systems, but ignores the velocity component.
 *
 * @tparam_double_only
 */
template <typename T>
class KukaTorqueController
    : public systems::Diagram<T>,
      public systems::controllers::StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KukaTorqueController)

  /// @p plant is aliased and must remain valid for the lifetime of the
  /// controller.
  KukaTorqueController(
      const multibody::MultibodyPlant<T>& plant,
      const VectorX<double>& stiffness,
      const VectorX<double>& damping);

  const systems::InputPort<T>& get_input_port_commanded_torque() const {
    return systems::Diagram<T>::get_input_port(
        input_port_index_commanded_torque_);
  }

  const systems::InputPort<T>& get_input_port_estimated_state() const override {
    return systems::Diagram<T>::get_input_port(
        input_port_index_estimated_state_);
  }

  const systems::InputPort<T>& get_input_port_desired_state() const override {
    return systems::Diagram<T>::get_input_port(input_port_index_desired_state_);
  }

  const systems::OutputPort<T>& get_output_port_control() const override {
    return systems::Diagram<T>::get_output_port(output_port_index_control_);
  }

 private:
  const multibody::MultibodyPlant<T>& plant_;
  systems::InputPortIndex input_port_index_estimated_state_;
  systems::InputPortIndex input_port_index_desired_state_;
  systems::InputPortIndex input_port_index_commanded_torque_;
  systems::OutputPortIndex output_port_index_control_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
