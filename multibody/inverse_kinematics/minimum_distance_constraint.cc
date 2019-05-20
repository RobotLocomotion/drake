#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance, systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(1)),
      plant_{RefFromPtrOrThrow(plant)},
      plant_context_{plant_context} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  if (!std::isfinite(influence_distance_offset)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance_offset must be finite.");
  }
  if (influence_distance_offset <= 0) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance_offset must be "
        "positive.");
  }
  const auto& query_port = plant_.get_geometry_query_input_port();
  if (!query_port.HasValue(*plant_context_)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  // Maximum number of SignedDistancePairs returned by calls to
  // ComputeSignedDistancePairwiseClosestPoints().
  const int num_collision_candidates =
      query_port.Eval<geometry::QueryObject<double>>(*plant_context_)
          .inspector()
          .GetCollisionCandidates()
          .size();
  minimum_value_constraint_ = std::make_unique<solvers::MinimumValueConstraint>(
      this->num_vars(), minimum_distance, influence_distance_offset,
      num_collision_candidates,
      std::bind(&MinimumDistanceConstraint::Distances<AutoDiffXd>, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&MinimumDistanceConstraint::Distances<double>, this,
                std::placeholders::_1, std::placeholders::_2));
  if (penalty_function) {
    minimum_value_constraint_->set_penalty_function(penalty_function);
  }
}

namespace {
void Distance(const MultibodyPlant<double>&, const systems::Context<double>&,
              const Frame<double>&, const Frame<double>&,
              const Eigen::Vector3d&, double distance, const Eigen::Vector3d&,
              const Eigen::Ref<const AutoDiffVecXd>&, double* distance_double) {
  *distance_double = distance;
}

void Distance(const MultibodyPlant<double>& plant,
              const systems::Context<double>& context,
              const Frame<double>& frameA, const Frame<double>& frameB,
              const Eigen::Vector3d& p_ACa, double distance,
              const Eigen::Vector3d& nhat_BA_W,
              const Eigen::Ref<const AutoDiffVecXd>& q,
              AutoDiffXd* distance_autodiff) {
  // The distance is d = sign * |p_CbCa_B|, where the
  // closest points are Ca on object A, and Cb on object B.
  // So the gradient ∂d/∂q = p_CbCa_W * ∂p_BCa_B/∂q / d (Note that
  // ∂p_BCa_B/∂q = ∂p_CbCa_B/∂q).
  //
  // Since dividing by d is undefined when d = 0, and p_CbCa_W / d =
  // nhat_BA_W whenever d ≠ 0, we use ∂d/∂q = nhat_BA_W′ * ∂p_BCa_B/∂q. This
  // allows us to compute a gradient at d = 0 in certain cases (See
  // geometry::SignedDistancePair for details).
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jq_v_BCa_W(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameA, p_ACa, frameB,
                                          plant.world_frame(), &Jq_v_BCa_W);
  const Eigen::RowVectorXd ddistance_dq = nhat_BA_W.transpose() * Jq_v_BCa_W;
  distance_autodiff->value() = distance;
  distance_autodiff->derivatives() =
      ddistance_dq * math::autoDiffToGradientMatrix(q);
}
}  // namespace

template <typename T>
VectorX<T> MinimumDistanceConstraint::Distances(
    const Eigen::Ref<const VectorX<T>>& q, double influence_distance) const {
  internal::UpdateContextConfiguration(plant_context_, plant_, q);
  const auto& query_port = plant_.get_geometry_query_input_port();
  if (!query_port.HasValue(*plant_context_)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(*plant_context_);

  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints(
              influence_distance);
  VectorX<T> distances(signed_distance_pairs.size());
  int distance_count{0};
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.distance < influence_distance) {
      const geometry::SceneGraphInspector<double>& inspector =
          query_object.inspector();
      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      const Frame<double>& frameA =
          plant_.GetBodyFromFrameId(frame_A_id)->body_frame();
      const Frame<double>& frameB =
          plant_.GetBodyFromFrameId(frame_B_id)->body_frame();
      Distance(plant_, *plant_context_, frameA, frameB,
               inspector.X_FG(signed_distance_pair.id_A) *
                   signed_distance_pair.p_ACa,
               signed_distance_pair.distance, signed_distance_pair.nhat_BA_W, q,
               &distances(distance_count++));
    }
  }
  distances.resize(distance_count);
  return distances;
}

template <typename T>
void MinimumDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  minimum_value_constraint_->Eval(x, y);
}

void MinimumDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void MinimumDistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
