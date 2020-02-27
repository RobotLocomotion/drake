#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/examples/hsr/parameters/robot_parameters.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace hsr {
namespace controllers {

/// This class calculates the torque for all the actuators of the robot
/// using different controllers for different parts. This class will be used
/// to wrap different sub-controllers for different parts of the robot. A
/// controller selector can be created to expose this option to the user.
///
/// This class takes three arguments: one is the multibody plant
/// of the robot with floating base. The second argument is the same
/// plant of the first argument but with the robot base welded to the ground.
/// Note that, only the robot base is fixed and the wheel joints are still free
/// to rotate. Fixing the robot base link to the ground gives us a fully
/// actuated robot model, which will be used for inverse dynamics controller
/// particularly since the Drake inverse dynamics controller only supports
/// fully actuated model. The third argument is RobotParameters struct which
/// contains the PID gains for each joint.
///
/// @system{ MainController,
///   @input_port{input_port_index_estimated_state_}  Estimated state of the
///          floating base robot model.
///   @input_port{input_port_index_desired_state_}  Desired state of the
///          floating base robot model.
///   @output_port{output_port_index_generalized_force_} Generalized
///          control from the inverse dynamics controller, which only
///          includes the generalized torque for the upper body.
///   @output_port{output_port_index_actuation_} Commanded actuation for the
///          robot. This port is required by the multibody plant. In current
///          implementation, only the generalized force is used. Therefore, this
///          port only outputs zero values.
/// }
class MainController final : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MainController)
  /// @p welded_robot_plant is aliased and must remain valid for the lifetime
  /// of the controller.
  MainController(const multibody::MultibodyPlant<double>& robot_plant,
                 const multibody::MultibodyPlant<double>& welded_robot_plant,
                 const hsr::parameters::RobotParameters<double>& parameters);

  const systems::InputPort<double>& get_estimated_state_input_port() const {
    return systems::Diagram<double>::get_input_port(
        input_port_index_estimated_state_);
  }

  const systems::InputPort<double>& get_desired_state_input_port() const {
    return systems::Diagram<double>::get_input_port(
        input_port_index_desired_state_);
  }

  const systems::OutputPort<double>& get_generalized_force_output_port() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_generalized_force_);
  }

  const systems::OutputPort<double>& get_actuation_output_port() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_actuation_);
  }

  const hsr::parameters::RobotParameters<double>& parameters() const {
    return parameters_;
  }

 private:
  int input_port_index_estimated_state_{-1};
  int input_port_index_desired_state_{-1};
  int output_port_index_generalized_force_{-1};
  int output_port_index_actuation_{-1};

  const hsr::parameters::RobotParameters<double> parameters_;
};

}  // namespace controllers
}  // namespace hsr
}  // namespace examples
}  // namespace drake
