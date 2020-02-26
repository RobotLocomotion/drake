// Copyright 2019 Toyota Research Institute. All rights reserved.
#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/hsr/common/robot_parameters.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace hsr {
namespace controller {

/// This class implements a specific version of inverse dynamics controller
/// for the floating base robot. It takes three arguments: one is the multibody
/// plant of the robot with floating base. The second argument is the same
/// plant of the first argument but with the robot base welded to the ground.
/// Note that, only the robot base is fixed and the wheel joints are still free
/// to rotate. Fixing the robot base link to the ground gives us a fully
/// actuated robot model, which will be used for inverse dynamics controller
/// particularly since the Drake inverse dynamics controller only supports
/// fully actuated model. Note that, fixing the base of the robot does not
/// necessarily give a fully actuated robot. This assumption is the requirement
/// of using this class. Inside this class, mappings are necessary to
/// convert states and torques between the floating base plant and fixed base
/// plant. The third argument is the parameters that specify the PID gains for
/// the controller.
///
/// @system{ InverseDynamicsController,
///   @input_port{input_port_index_estimated_state_}  Estimated state of the
///          floating base robot model.
///   @input_port{input_port_index_desired_state_}  Desired state of the
///          floating base robot model.
///   @output_port{get_generalized_force_output_port} Generalized
///          control from the inverse dynamics controller, which only
///          includes the generalized torque for the upper body
/// }

class InverseDynamicsController final : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamicsController)
  /// @p welded_robot_plant is aliased and must remain valid for the lifetime
  /// of the controller.
  InverseDynamicsController(
      const drake::multibody::MultibodyPlant<double>& robot_plant,
      const drake::multibody::MultibodyPlant<double>& welded_robot_plant,
      const RobotParameters<double>& parameters);

  const drake::systems::InputPort<double>& get_estimated_state_input_port()
      const {
    return drake::systems::Diagram<double>::get_input_port(
        input_port_index_estimated_state_);
  }

  const drake::systems::InputPort<double>& get_desired_state_input_port()
      const {
    return drake::systems::Diagram<double>::get_input_port(
        input_port_index_desired_state_);
  }

  const drake::systems::OutputPort<double>& get_generalized_force_output_port()
      const {
    return drake::systems::Diagram<double>::get_output_port(
        output_port_index_generalized_force_);
  }

 private:
  /// Load the PID gains from the robot parameters to separate vectors, which
  /// will be used by the Drake internal inverse dynamics controller.
  void LoadPidGainsFromRobotParameters(
      const RobotParameters<double>& parameters,
      const multibody::MultibodyPlant<double>& welded_robot_plant,
      VectorX<double>* kp, VectorX<double>* kd, VectorX<double>* ki);

  int input_port_index_estimated_state_{-1};
  int input_port_index_desired_state_{-1};
  int output_port_index_generalized_force_{-1};
};

}  // namespace controller
}  // namespace hsr
}  // namespace examples
}  // namespace drake
