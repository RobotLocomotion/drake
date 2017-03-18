#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_base_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends PlanEvalBaseSystem. It generates QpInput to track
 * desired instantaneous position, velocity and acceleration in joint space.
 */
class ManipulatorPlanEvalSystem : public PlanEvalBaseSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorPlanEvalSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Time step
   */
  ManipulatorPlanEvalSystem(const RigidBodyTree<double>& robot,
                            const std::string& alias_groups_file_name,
                            const std::string& param_file_name, double dt);

  /**
   * Initializes the plan and gains. Must be called before execution.
   */
  void Initialize(systems::State<double>* state);

  /**
   * Initializes the plan and gains. Must be called before execution.
   */
  void Initialize(systems::Context<double>* context) {
    systems::State<double>* servo_state = context->get_mutable_state();
    Initialize(servo_state);
  }

  /**
   * Returns the input port for desired position and velocity.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_state() const {
    return get_input_port(input_port_index_desired_state_);
  }

  /**
   * Returns the input port for desired acceleration.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_acceleration() const {
    return get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for debugging information.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_debug_info() const {
    return get_output_port(output_port_index_debug_info_);
  }

 private:
  int get_num_extended_abstract_states() const override { return 2; }

  void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const override;

  void DoExtendedCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const override;

  std::vector<std::unique_ptr<systems::AbstractValue>>
  ExtendedAllocateAbstractState() const override;

  std::unique_ptr<systems::AbstractValue> ExtendedAllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  int input_port_index_desired_state_{};
  int input_port_index_desired_acceleration_{};
  int output_port_index_debug_info_{};

  const int abs_state_index_plan_{};
  const int abs_state_index_debug_{};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
