#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Constrains the spatial velocity of a frame C, rigidly attached to a frame B,
 * measured and expressed in frame A. It should be bound with decision
 * variables corresponding to the multibody state x=[q,v] of the `plant` passed
 * to the constructor.
 *
 * @ingroup solver_evaluators
 */
class SpatialVelocityConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpatialVelocityConstraint);

  /** Parametrizes bounds on the magnitude and direction of the angular
   * velocity vector. */
  struct AngularVelocityBounds {
    /** Lower bound on the magnitude of the angular velocity vector. Must be
     * non-negative. The actual constraint will be implemented as a constraint
     * on the squared magnitude.*/
    double magnitude_lower{0};

    /** Upper bound on the magnitude of the angular velocity vector. Must be ≥
     * magnitude_lower. The actual constraint will be implemented as a
     * constraint on the squared magnitude.*/
    double magnitude_upper{std::numeric_limits<double>::infinity()};

    /** Reference direction of the angular velocity vector. (Only the direction
     * matters, the magnitude does not). */
    Eigen::Vector3d reference_direction{0, 0, 1};

    /** The angle difference between w_AC and reference_direction will be
     * constrained to θ ∈ [0,π] ≤ θ_bound. Must be nonnegative. The actual
     * constraint will be implemented as cos(θ_bound) ≤ cos(θ) ≤ 1. When the
     * norm of w_AC is very close to zero, we add a small number to the norm to
     * avoid numerical difficulties. */
    double theta_bound{M_PI};
  };

  /**
   * Constructs SpatialVelocityConstraint object.
   * @param plant The MultibodyPlant on which the constraint is imposed.
   *   `plant` should be alive during the lifetime of this constraint.
   * @param frameA The frame in which frame C's spatial velocity is measured
   *   and expressed.
   * @param v_AC_lower The lower bound on the translational velocity of C,
   *   expressed in frame A.
   * @param v_AC_upper The upper bound on the translational velocity of C,
   *   expressed in frame A.
   * @param frameB The frame to which frame C is rigidly attached.
   * @param p_BCo The position of the origin of frame C, rigidly attached to
   *   frame B, expressed in frame B. We take R_BC to be the identity rotation.
   * @param plant_context The Context that has been allocated for this `plant`.
   *   We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @param w_AC_bounds If provided, then the number of constraints will be 5:
   *   3 for translational velocities and then two more for the angular
   *   velocity magnitude and angle.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @pre v_AC_lower(i) <= v_AC_upper(i) for i = 1, 2, 3.
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @throws std::exception if invalid w_AC_bounds are provided.
   */
  SpatialVelocityConstraint(
      const MultibodyPlant<AutoDiffXd>* plant, const Frame<AutoDiffXd>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& v_AC_lower,
      const Eigen::Ref<const Eigen::Vector3d>& v_AC_upper,
      const Frame<AutoDiffXd>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BCo,
      systems::Context<AutoDiffXd>* plant_context,
      const std::optional<AngularVelocityBounds>& w_AC_bounds = std::nullopt);

  ~SpatialVelocityConstraint() final;

  // TODO(russt): Implement set_bounds, UpdateLowerBound, and UpdateUpperBound
  // wrappers.

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "SpatialVelocityConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  const MultibodyPlant<AutoDiffXd>* const plant_{};
  systems::Context<AutoDiffXd>* const context_{};
  const FrameIndex frameA_index_;
  const FrameIndex frameB_index_;
  const Eigen::Vector3d p_BC_;
  std::optional<Eigen::Vector3d> w_AC_reference_direction_{std::nullopt};
};
}  // namespace multibody
}  // namespace drake
