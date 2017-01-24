#pragma once

#include <memory>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class HumanoidPlanEvalSystem : public PlanEvalSystem {
 public:
  HumanoidPlanEvalSystem(const RigidBodyTree<double>& robot,
      const std::string& alias_groups_file_name,
      const std::string& param_file_name);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  /**
   * Set the set point for tracking.
   * @param q_d Desired generalized position.
   */
  void SetDesired(const VectorX<double>& q, systems::State<double>* state);
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
