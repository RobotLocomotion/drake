#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/discrete_time_plan_eval_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends DiscreteTimePlanEvalSystem. It generates QpInput to track
 * a desired trajectory.
 */
class KukaPlanEvalSystem : public DiscreteTimePlanEvalSystem {
 public:
  KukaPlanEvalSystem(const RigidBodyTree<double>& robot,
                     const std::string& alias_groups_file_name,
                     const std::string& param_file_name, double dt);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  /**
   * Makes a plan folloing @p traj and stores it in @p state
   * @param traj Desired trajectory
   * @param state Holds the plan
   */
  void SetDesiredTrajectory(const PiecewisePolynomialTrajectory& traj,
                            systems::State<double>* state);
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
