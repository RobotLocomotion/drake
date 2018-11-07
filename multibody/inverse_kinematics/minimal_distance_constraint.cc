#include "drake/multibody/inverse_kinematics/minimal_distance_constraint.h"

#include <limits>
#include <vector>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * Implements the penalty function  γ(φᵢ/dₘᵢₙ - 1)
 * where φᵢ is the signed distance of the i'th pair, dₘᵢₙ is the minimal
 * allowable distance, and γ is a penalizing function defined as
 * γ(x) = 0 if x ≥ 0
 * γ(x) = -x exp(1/x) if x < 0
 * @param distance φᵢ in the documentation above.
 * @param distance_threshold dₘᵢₙ in the documentation above.
 * @param penalty the penalty γ.
 * @param dpenalty_ddistance The gradient dγ/dφᵢ.
 */
void Penalty(double distance, double distance_threshold, double* penalty,
             double* dpenalty_ddistance) {
  if (distance >= distance_threshold) {
    *penalty = 0;
    *dpenalty_ddistance = 0;
  } else {
    const double x = distance / distance_threshold - 1;
    const double exp_one_over_x = std::exp(1.0 / x);
    *penalty = -x * exp_one_over_x;
    const double dpenalty_dx = -exp_one_over_x + exp_one_over_x / x;
    *dpenalty_ddistance = dpenalty_dx / distance_threshold;
  }
}

MinimalDistanceConstraint::MinimalDistanceConstraint(
    const multibody::multibody_plant::MultibodyPlant<AutoDiffXd>& plant,
    double minimal_distance, systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, plant.num_positions(), Vector1d(0), Vector1d(0)),
      plant_{plant},
      minimal_distance_{minimal_distance},
      plant_context_{plant_context} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimalDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet.");
  }
  if (minimal_distance_ <= 0) {
    throw std::invalid_argument(
        "MinimalDistanceConstraint: minimal_distance should be positive.");
  }
}

void MinimalDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void MinimalDistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  y->resize(1);

  UpdateContextConfiguration(
      x, dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(plant_context_));
  const geometry::QueryObject<AutoDiffXd>& query_object =
      plant_
          .EvalAbstractInput(*plant_context_,
                             plant_.get_geometry_query_input_port().get_index())
          ->GetValue<geometry::QueryObject<AutoDiffXd>>();

  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();

  (*y) = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(0), Eigen::RowVectorXd::Zero(x(0).derivatives().size()));
  // The distance is d = sign * |p_CbCa_W| = sign * |p_WCa - p_WCb|, where the
  // closest points are Ca on object A, and Cb on object B. Namely
  // d = sign * |p_WA + R_WA * p_ACa - p_WB + R_WB * p_BCb|
  // So the gradient ∂d/∂q = sign * p_CbCa_W / d² * (∂p_WCa/∂q - ∂p_WCb/∂q)
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const double distance = signed_distance_pair.distance;
    if (distance < minimal_distance_) {
      const double sign = distance > 0 ? 1 : -1;

      Vector3<AutoDiffXd> p_WCa, p_WCb;
      const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
          query_object.inspector();
      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      plant_.tree().CalcPointsPositions(
          *plant_context_, plant_.GetBodyFromFrameId(frame_A_id)->body_frame(),
          signed_distance_pair.p_ACa.cast<AutoDiffXd>(), plant_.world_frame(),
          &p_WCa);
      plant_.tree().CalcPointsPositions(
          *plant_context_, plant_.GetBodyFromFrameId(frame_B_id)->body_frame(),
          signed_distance_pair.p_BCb.cast<AutoDiffXd>(), plant_.world_frame(),
          &p_WCb);
      double penalty, dpenalty_ddistance;
      Penalty(distance, minimal_distance_, &penalty, &dpenalty_ddistance);

      const AutoDiffXd distance_autodiff = sign * (p_WCa - p_WCb).norm();
      Eigen::RowVectorXd distance_gradient = distance_autodiff.derivatives();
      Vector1<AutoDiffXd> penalty_autodiff =
          math::initializeAutoDiffGivenGradientMatrix(
              Vector1d(penalty), dpenalty_ddistance * distance_gradient);
      *y += penalty_autodiff;
    }
  }
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
