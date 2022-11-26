#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Constrains the spatial velocity of a point Q, rigidly attached to a frame B,
 * measured and expressed in frame A. It should be bound with decision
 * variables corresponding to the multibody state x=[q,v] of the `plant` passed
 * to the constructor.
 *
 * @ingroup solver_evaluators
 */
class SpatialVelocityConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpatialVelocityConstraint)

  /** Parametrizes bounds on the magnitude and direction of the angular
   * velocity vector. */
  struct AngularVelocityBounds {
    /** Lower bound on the magnitude of the angular velocity vector. Must be
     * non-negative. */
    double magnitude_lower{0};

    /** Upper bound on the magnitude of the angular velocity vector. Must be
     * nonnegative.*/
    double magnitude_upper{std::numeric_limits<double>::infinity()};

    /** Nominal direction of the angular velocity vector. (Only the direction
     * matters, the magnitude does not). */
    Eigen::Vector3d nominal_direction{0, 0, 1};

    /** The angle difference between w_AQ and w_AQ_nominal_direction will be
     * constrained to θ ∈ [0,π] ≤ θ_bound. Must be nonnegative. */
    double theta_bound{M_PI};
  };

  /**
   * Constructs SpatialVelocityConstraint object.
   * @param plant The MultibodyPlant on which the constraint is imposed.
   *   `plant` should be alive during the lifetime of this constraint.
   * @param frameA The frame in which point Q's position is measured.
   * @param v_AQ_lower The lower bound on the translational velocity of point
   *   Q, measured and expressed in frame A.
   * @param v_AQ_upper The upper bound on the translational velocity of point
   *   Q, measured and expressed in frame A.
   * @param frameB The frame to which point Q is rigidly attached.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   *   measured and expressed in frame B.
   * @param plant_context The Context that has been allocated for this `plant`.
   *   We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @param w_AQ_bounds If provided, then the number of constraints
   *   will be 5: 3 for translational velocities and then two more for the
   *   angular velocity magnitude and angle.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @pre v_AQ_lower(i) <= v_AQ_upper(i) for i = 1, 2, 3.
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @throws std::exception if invalid w_AQ_bounds are provided.
   */
  SpatialVelocityConstraint(const MultibodyPlant<AutoDiffXd>* plant,
                            const Frame<AutoDiffXd>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& v_AQ_lower,
                            const Eigen::Ref<const Eigen::Vector3d>& v_AQ_upper,
                            const Frame<AutoDiffXd>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                            systems::Context<AutoDiffXd>* plant_context,
                            const std::optional<AngularVelocityBounds>&
                                w_AQ_bounds = std::nullopt);

  ~SpatialVelocityConstraint() override {}

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

  const MultibodyPlant<AutoDiffXd>* const plant_;
  systems::Context<AutoDiffXd>* const context_;
  const FrameIndex frameA_index_;
  const FrameIndex frameB_index_;
  const Eigen::Vector3d p_BQ_;
  std::optional<Eigen::Vector3d> w_AQ_nominal_direction_{std::nullopt};
};
}  // namespace multibody
}  // namespace drake
