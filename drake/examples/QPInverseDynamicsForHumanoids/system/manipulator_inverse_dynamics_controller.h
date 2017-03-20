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
 * This class implements an example of a general purpose controller for a
 * fixed based manipulator based on qp inverse dynamics. It contains two main
 * components, plan eval and qp inverse dynamics. Plan eval is responsible for
 * implementing various desired behaviors. It outputs a QpInput struct which is
 * handed to qp inverse dynamics. The latter is responsible for generating
 * torque outputs that best tracks the desired inputs while satisfying all the
 * physical constraints (torque limit, contact maintenance, etc).
 * The biggest advantage for this formulation is that the details of the
 * physical robot is hidden from the behavior level.
 *
 * This particular example implements a controller that tracks desired
 * generalized position, velocity and acceleration. It is functionally
 * identical to systems::InverseDynamicsController for the unconstrained
 * situations (no torque limits). More advanced behaviors such as end effector
 * tracking in Cartesian space, freeze if unexpected is sensed can be
 * implemented by different plan eval modules.
 */
class ManipulatorInverseDynamicsController
    : public systems::ModelBasedController<double> {
 public:
  /**
   * Constructs a inverse dynamics controller for a fixed base manipulator that
   * tracks desired generalized position, velocity and acceleration.
   * @param model_path Path to the manipulator model, from which the internal
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

  /**
   * Initializes the controller's internal state. Must be called before
   * execution.
   */
  void Initialize(systems::Context<double>* context);

  /**
   * Returns plan eval's alias groups.
   */
  const param_parsers::RigidBodyTreeAliasGroups<double>& get_alias_groups()
      const {
    return plan_eval_->get_alias_groups();
  }

  /**
   * Returns plan eval's parameters.
   */
  const param_parsers::ParamSet& get_paramset() const {
    return plan_eval_->get_paramset();
  }

  /**
   * Returns the input port for desired acceleration.
   */
  const systems::InputPortDescriptor<double>&
  get_input_port_desired_acceleration() const {
    return systems::Diagram<double>::get_input_port(
        input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for a lcm message that contains plan eval's
   * debug data.
   */
  const systems::OutputPortDescriptor<double>&
  get_output_port_plan_eval_debug_info() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_plan_eval_debug_);
  }

  /**
   * Returns the output port for a lcm message that contains qp inverse
   * dynamics' debug data.
   */
  const systems::OutputPortDescriptor<double>&
  get_output_port_inverse_dynamics_debug_info() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_inverse_dynamics_debug_);
  }

  /**
   * Returns the output port for QpInput from plan eval.
   */
  const systems::OutputPortDescriptor<double>&
  get_output_port_qp_input() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_qp_input_);
  }

  /**
   * Returns the output port for QpOutput from inverse dynamics.
   */
  const systems::OutputPortDescriptor<double>&
  get_output_port_qp_output() const {
    return systems::Diagram<double>::get_output_port(
        output_port_index_qp_output_);
  }

 private:
  ManipulatorPlanEvalSystem* plan_eval_{nullptr};
  int input_port_index_desired_acceleration_{};
  int output_port_index_plan_eval_debug_{};
  int output_port_index_qp_input_{};
  int output_port_index_inverse_dynamics_debug_{};
  int output_port_index_qp_output_{};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
