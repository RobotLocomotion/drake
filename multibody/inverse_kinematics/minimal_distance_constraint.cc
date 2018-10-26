#include "drake/multibody/inverse_kinematics/minimal_distance_constraint.h"

#include <limits>
#include <vector>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
MinimalDistanceConstraint::MinimalDistanceConstraint(
    const multibody::multibody_plant::MultibodyPlant<AutoDiffXd>& plant,
    double minimal_distance, systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, plant.num_positions(), Vector1d(minimal_distance),
                          Vector1d(std::numeric_limits<double>::infinity())),
      plant_{plant},
      minimal_distance_{minimal_distance},
      plant_context_{plant_context} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimalDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet.");
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
  // Get the pair with the minimal distance.
  const geometry::SignedDistancePair<double>* closest_pair = nullptr;
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.distance < min_distance) {
      closest_pair = &signed_distance_pair;
      min_distance = signed_distance_pair.distance;
    }
  }

  // The distance is d = sign * |p_CbCa_W| = sign * |p_WCa - p_WCb|, where the
  // closest points are Ca on object A, and Cb on object B. Namely
  // d = sign * |p_WA + R_WA * p_ACa - p_WB + R_WB * p_BCb|
  // So the gradient ∂d/∂q = sign * p_CbCa_W / d² * (∂p_WCa/∂q - ∂p_WCb/∂q)
  const double sign = min_distance > 0 ? 1 : -1;

  Vector3<AutoDiffXd> p_WCa, p_WCb;
  const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
      query_object.inspector();
  const geometry::FrameId frame_A_id = inspector.GetFrameId(closest_pair->id_A);
  const geometry::FrameId frame_B_id = inspector.GetFrameId(closest_pair->id_B);
  plant_.tree().CalcPointsPositions(
      *plant_context_, plant_.GetBodyFromFrameId(frame_A_id)->body_frame(),
      closest_pair->p_ACa.cast<AutoDiffXd>(), plant_.world_frame(), &p_WCa);
  plant_.tree().CalcPointsPositions(
      *plant_context_, plant_.GetBodyFromFrameId(frame_B_id)->body_frame(),
      closest_pair->p_BCb.cast<AutoDiffXd>(), plant_.world_frame(), &p_WCb);
  (*y)(0) = sign * (p_WCa - p_WCb).norm();
  (*y)(0).value() = min_distance;
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
