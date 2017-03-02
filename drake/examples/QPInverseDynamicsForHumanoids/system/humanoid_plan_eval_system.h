#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_base_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends PlanEvalBaseSystem. The implemented behavior moves the
 * robot's pelvis height following a sine wave while holding everything else
 * stationary. It assumes the robot is in double stance, and the stationary
 * set point is set by SetDesired().
 */
class HumanoidPlanEvalSystem : public PlanEvalBaseSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HumanoidPlanEvalSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Control time step.
   */
  HumanoidPlanEvalSystem(const RigidBodyTree<double>& robot,
                         const std::string& alias_groups_file_name,
                         const std::string& param_file_name, double dt);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  /**
   * Initializes the internal plan in @p state to track the desired
   * configuration @p q.
   * @param q_d Desired generalized position.
   * @param state State
   */
  void SetDesired(const VectorX<double>& q, systems::State<double>* state);
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
