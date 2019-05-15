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
(according to the logic of SceneGraph::GetCollisionCandidates()) to be no
smaller than a specified minimum distance.

The formulation of the constraint is

0 ≤ SmoothMax( γ((dᵢ - dₜₕᵣₑₛₕ)/(dₜₕᵣₑₛₕ - dₘᵢₙ)) / γ(-1) ) ≤ 1

where dᵢ is the signed distance of the i-th pair, dₘᵢₙ is the minimum allowable
distance, dₜₕᵣₑₛₕ is the distance beyond which pairs of geometries are ignored,
γ is a MinimumDistancePenaltyFunction, and SmoothMax(d) is smooth approximation
of max(d). We require that dₘᵢₙ < dₜₕᵣₑₛₕ. The input scaling
(dᵢ - dₜₕᵣₑₛₕ)/(dₜₕᵣₑₛₕ - dₘᵢₙ) ensures that at the boundary of the feasible set
(when dᵢ == dₘᵢₙ), we evaluate the penalty function at -1, where it is required
to have a non-zero gradient.
*/
class MinimumDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimumDistanceConstraint)

  /** Constructs a MinimumDistanceConstraint.
  @param plant The robot on which the inverse kinematics problem will be
  solved. This plant has to have registered its geometry with a SceneGraph
  object. @throws invalid_argument if the plant has not registered its
  geometry.
  @param minimum_distance The minimum value of the signed distance between
  any admissible pairs of objects.
  @param penalty_type The penalty function formulation.
  @param threshold_distance The penalty function formulation.
  @pre The MultibodyPlant passed in the constructor of InverseKinematics has
  registered its geometry with a SceneGraph object already.
  @pre minimum_distance > 0.
  @throw invalid_argument if the geometry hasn't been registered. */
  MinimumDistanceConstraint(
      const multibody::MultibodyPlant<double>* const plant,
      double minimum_distance, systems::Context<double>* plant_context,
      MinimumDistancePenaltyFunction penalty_function =
          QuadraticallySmoothedHingeLoss,
      double threshold_distance = 1);

  ~MinimumDistanceConstraint() override {}

  /** Getter for the minimum distance. */
  double minimum_distance() const { return minimum_distance_; }

  /** Getter for the threshold distance. */
  double threshold_distance() const { return threshold_distance_; }

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
  const double threshold_distance_;
  /** Stores the value of 1 / γ((dₘᵢₙ - dₜₕᵣₑₛₕ)/(dₜₕᵣₑₛₕ - dₘᵢₙ)) = 1 / γ(-1).
  This is used to scale the output of the penalty function to be 1 when
  d == dₘᵢₙ. */
  const double penalty_output_scaling_;
  int num_collision_candidates_{};
  systems::Context<double>* const plant_context_;
  MinimumDistancePenaltyFunction penalty_function_;
};
}  // namespace multibody
}  // namespace drake
