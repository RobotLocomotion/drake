#pragma once

#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/cost.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Implements a cost of the form `-c * w(q)`, where `w` is Yoshikawa's
 * manipulability index of frame B relative to frame A, so that minimizing the
 * cost maximizes manipulability. With J the `6 x n` spatial velocity Jacobian
 * of frame B measured in frame A, keeping only the columns of the n
 * generalized velocities on the kinematic path between A and B, <pre>
 *     w(q) = √det(J Jᵀ).
 * </pre>
 * The index depends only on the bodies of frames A and B, not on the frames'
 * poses on those bodies, and swapping A and B leaves it unchanged. If a
 * floating joint is on the path (e.g. A is the world frame and the robot's base
 * floats), its six columns make J Jᵀ nonsingular everywhere; to measure an
 * arm's manipulability, use the arm's base as frame A. At a singularity
 * (det(J Jᵀ) = 0) the cost and its gradient are zero. The gradient is exact,
 * computed from MultibodyPlant::CalcHessianSpatialVelocity().
 *
 * T. Yoshikawa, "Manipulability of Robotic Mechanisms," The International
 * Journal of Robotics Research, 4(2), pp. 3-9, 1985.
 *
 * @ingroup solver_evaluators
 */
class ManipulabilityCost : public solvers::Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulabilityCost);

  /**
   * Constructs ManipulabilityCost object.
   * @param plant The MultibodyPlant on which the cost is implemented. `plant`
   *   should be alive during the lifetime of this cost.
   * @param frameA The frame in which frame B's motion is measured.
   * @param frameB The frame whose manipulability is measured.
   * @param plant_context A context for the `plant`.
   * @param c A positive scalar cost weight.
   *
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @throws std::exception if `c` is not positive.
   * @throws std::exception if fewer than six generalized velocities are on the
   *   kinematic path between frame A and frame B (det(J Jᵀ) would always be
   *   zero).
   */
  ManipulabilityCost(const MultibodyPlant<double>* plant,
                     const Frame<double>& frameA, const Frame<double>& frameB,
                     systems::Context<double>* plant_context, double c = 1.0);

  ~ManipulabilityCost() override;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override;

  // Returns w at q; if `dw_dq` is non-null, also sets it to ∂w/∂q.
  double CalcIndex(const Eigen::Ref<const Eigen::VectorXd>& q,
                   Eigen::VectorXd* dw_dq) const;

  const MultibodyPlant<double>& plant_;
  const FrameIndex frameA_index_;
  const FrameIndex frameB_index_;
  systems::Context<double>* const context_;
  const double c_;
  // Indices into v of the generalized velocities on the path between A and B.
  std::vector<int> path_velocities_;
};
}  // namespace multibody
}  // namespace drake
