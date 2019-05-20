#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/** Computes the penalty function γ(x) and its derivatives dγ(x)/dx. Valid
penalty functions must meet the following criteria:

1.     γ(x) ≥ 0 ∀ x ∈ ℝ.
2. dγ(x)/dx ≤ 0 ∀ x ∈ ℝ.
3.     γ(x) = 0 ∀ x ≥ 0.
4. dγ(x)/dx < 0 ∀ x < 0. */
using MinimumDistancePenaltyFunction =
    std::function<void(double x, double* penalty, double* dpenalty_dx)>;

/** A hinge loss function smoothed by exponential function. This loss
function is differentiable everywhere. The fomulation is described in
section II.C of [2].
The penalty is
<pre>
       ⎧ 0            if x ≥ 0
γ(x) = ⎨
       ⎩  -x exp(1/x) if x < 0.
</pre>
[2] "Whole-body Motion Planning with Centroidal Dynamics and Full
Kinematics" by Hongkai Dai, Andres Valenzuela and Russ Tedrake, IEEE-RAS
International Conference on Humanoid Robots, 2014. */
void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

/** A linear hinge loss, smoothed with a quadratic loss near the origin. The
formulation is in equation (6) of [1].
The penalty is
<pre>
       ⎧  0        if x ≥ 0
γ(x) = ⎨  x²/2     if -1 < x < 0
       ⎩  -0.5 - x if x ≤ -1.
</pre>
[1] "Loss Functions for Preference Levels: Regression with Discrete Ordered
Labels." by Jason Rennie and Nathan Srebro, Proceedings of IJCAI
multidisciplinary workshop on Advances in preference handling. */
void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

/** Constrain the signed distance between all candidate pairs of geometries
(according to the logic of SceneGraphInspector::GetCollisionCandidates()) to be
no smaller than a specified minimum distance.

The formulation of the constraint is

0 ≤ SmoothMax( γ((dᵢ - d_influence)/(d_influence - dₘᵢₙ)) / γ(-1) ) ≤ 1

where dᵢ is the signed distance of the i-th pair, dₘᵢₙ is the minimum allowable
distance, d_influence is the "influence distance" (the distance below which a
pair of geometries influences the constraint), γ is a
multibody::MinimumDistancePenaltyFunction, and SmoothMax(d) is smooth
approximation of max(d). We require that dₘᵢₙ < d_influence. The input scaling
(dᵢ - d_influence)/(d_influence - dₘᵢₙ) ensures that at the boundary of the
feasible set (when dᵢ == dₘᵢₙ), we evaluate the penalty function at -1, where it
is required to have a non-zero gradient.
*/
class MinimumDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceConstraint)

  /** Constructs a MinimumDistanceConstraint.
  @param plant The multibody system on which the constraint will be evaluated.
  @param minimum_distance The minimum allowed value, dₘᵢₙ, of the signed
  distance between any candidate pair of geometries.
  @param penalty_function The penalty function formulation.
  @default QuadraticallySmoothedHinge
  @param influence_distance_offset The difference (in meters) between the
  influence distance, d_influence, and the minimum distance, dₘᵢₙ (see class
  documentation). This value must be finite and strictly positive, as it is used
  to scale the signed distances between pairs of geometries. Smaller values may
  improve performance, as fewer pairs of geometries need to be considered in
  each constraint evaluation. @default 1 meter
  @throws std::invalid_argument if `plant` has not registered its geometry with
  a SceneGraph object.
  @throws std::invalid_argument if influence_distance_offset = ∞.
  @throws std::invalid_argument if influence_distance_offset ≤ 0.
  */
  MinimumDistanceConstraint(
      const multibody::MultibodyPlant<double>* const plant,
      double minimum_distance, systems::Context<double>* plant_context,
      MinimumDistancePenaltyFunction penalty_function =
          QuadraticallySmoothedHingeLoss,
      double influence_distance_offset = 1);

  ~MinimumDistanceConstraint() override {}

  /** Getter for the minimum distance. */
  double minimum_distance() const { return minimum_distance_; }

  /** Getter for the influence distance. */
  double influence_distance() const { return influence_distance_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "MinimumDistanceConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  const multibody::MultibodyPlant<double>& plant_;
  const double minimum_distance_;
  const double influence_distance_;
  /** Stores the value of
  1 / γ((dₘᵢₙ - d_influence)/(d_influence - dₘᵢₙ)) = 1 / γ(-1). This is used to
  scale the output of the penalty function to be 1 when d == dₘᵢₙ. */
  const double penalty_output_scaling_;
  int num_collision_candidates_{};
  systems::Context<double>* const plant_context_;
  MinimumDistancePenaltyFunction penalty_function_;
};
}  // namespace multibody
}  // namespace drake
