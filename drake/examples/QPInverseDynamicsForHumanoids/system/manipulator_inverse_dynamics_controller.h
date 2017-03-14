#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_plan_eval_system.h"
#include "drake/systems/controllers/model_based_controller_base.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Builds a Diagram of a Kuka IIWA arm controlled by inverse dynamics to follow
 * a desired trajectory.
 */
class ManipulatorInverseDynamicsController
    : public systems::ModelBasedController<double> {
 public:
  /**
   * Constructs a inverse dynamics controller for the Kuka iiwa arm. It
   * maintains a separate RigidBodyTree just for the controller, which can be
   * instantiated with different model file than the one used for simulation.
   * @param model_path Path to the Kuka iiwa model, from which the internal
   * model is instantiated.
   * @param alias_group_path Path to the alias groups file. Used by the
   * controller to understand to topology of the robot.
   * @param controller_config_path Path the config file for the controller.
   * @param dt Time step
   * @param world_offset RigidBodyFrame X_WB, where B is the base of the robot.
   */
  ManipulatorInverseDynamicsController(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path, double dt,
      std::shared_ptr<RigidBodyFrame<double>> world_offset = nullptr);

  void Initialize(systems::Context<double>* context);

  const systems::InputPortDescriptor<double>&
  get_input_port_desired_acceleration() const {
    return systems::Diagram<double>::get_input_port(
        input_port_index_desired_acceleration_);
  }

  const systems::OutputPortDescriptor<double>&
  get_output_port_plan_eval_debug_info() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_plan_eval_debug_);
  }

  const systems::OutputPortDescriptor<double>&
  get_output_port_inverse_dynamics_debug_info() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_inverse_dynamics_debug_);
  }

 private:
  ManipulatorPlanEvalSystem* servo_{nullptr};
  int input_port_index_desired_acceleration_{};
  int output_port_index_plan_eval_debug_{};
  int output_port_index_inverse_dynamics_debug_{};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
