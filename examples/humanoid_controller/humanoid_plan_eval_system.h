#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/plan_eval/plan_eval_base_system.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

/**
 * This class extends PlanEvalBaseSystem to interpret plans for humanoid robots.
 * At every tick, this class generates a QpInput that tracks those trajectories
 * given the current measured robot state and parameters.
 *
 * It currently only supports manipulation plans. Upon receiving a new
 * manipulation plan specified by a sequence of keyframes, this class makes new
 * trajectories to smoothly connect them.
 *
 * @see HumanoidManipulationPlan for more details.
 * TODO(siyuan) add a walking plan.
 */
class HumanoidPlanEvalSystem
    : public systems::controllers::plan_eval::PlanEvalBaseSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HumanoidPlanEvalSystem)

  /**
   * Constructor.
   * @param robot Pointer to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Control time step.
   */
  HumanoidPlanEvalSystem(const RigidBodyTree<double>* robot,
                         const std::string& alias_groups_file_name,
                         const std::string& param_file_name, double dt);

  /**
   * Initializes the plan in @p state to maintain the current robot
   * configuration in @p current_status.
   * @param current_status Current robot status.
   * @param state State
   */
  void Initialize(const systems::controllers::qp_inverse_dynamics::
                      RobotKinematicState<double>& current_status,
                  systems::State<double>* state) const;

  /**
   * Returns input port of type robotlocomotion::robot_plan_t message that
   * contains the manipulation plan.
   */
  const systems::InputPortDescriptor<double>& get_input_port_manip_plan_msg()
      const {
    return get_input_port(input_port_index_manip_plan_msg_);
  }

 private:
  int get_num_extended_abstract_states() const override { return 1; }

  void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const override;

  int abs_state_index_plan_{};
  int input_port_index_manip_plan_msg_{};
};

}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
