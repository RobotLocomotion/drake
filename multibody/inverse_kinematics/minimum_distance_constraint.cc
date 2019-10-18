#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/inverse_kinematics/distance_constraint_utilities.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

template <typename T, typename S>
VectorX<S> Distances(const MultibodyPlant<T>& plant,
                     systems::Context<T>* context,
                     const Eigen::Ref<const VectorX<S>>& q,
                     double influence_distance) {
  internal::UpdateContextConfiguration(context, plant, q);
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(*context)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.template Eval<geometry::QueryObject<T>>(*context);

  const std::vector<geometry::SignedDistancePair<T>> signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(
          influence_distance);
  VectorX<S> distances(signed_distance_pairs.size());
  int distance_count{0};
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.distance < influence_distance) {
      const geometry::SceneGraphInspector<T>& inspector =
          query_object.inspector();
      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      const Frame<T>& frameA =
          plant.GetBodyFromFrameId(frame_A_id)->body_frame();
      const Frame<T>& frameB =
          plant.GetBodyFromFrameId(frame_B_id)->body_frame();
      internal::CalcDistanceDerivatives(
          plant, *context, frameA, frameB,
          // GetPoseInFrame() returns RigidTransform<double> -- we can't
          // multiply across heterogeneous scalar types; so we cast the double
          // to T.
          inspector.GetPoseInFrame(signed_distance_pair.id_A)
                  .template cast<T>() *
              signed_distance_pair.p_ACa,
          signed_distance_pair.distance, signed_distance_pair.nhat_BA_W, q,
          &distances(distance_count++));
    }
  }
  distances.resize(distance_count);
  return distances;
}

template <typename T>
void MinimumDistanceConstraint::Initialize(
    const MultibodyPlant<T>& plant, systems::Context<T>* plant_context,
    double minimum_distance, double influence_distance_offset,
    MinimumDistancePenaltyFunction penalty_function) {
  CheckPlantIsConnectedToSceneGraph(plant, *plant_context);
  if (!std::isfinite(influence_distance_offset)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance_offset must be finite.");
  }
  if (influence_distance_offset <= 0) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: influence_distance_offset must be "
        "positive.");
  }
  const auto& query_port = plant.get_geometry_query_input_port();
  // Maximum number of SignedDistancePairs returned by calls to
  // ComputeSignedDistancePairwiseClosestPoints().
  const int num_collision_candidates =
      query_port.template Eval<geometry::QueryObject<T>>(*plant_context)
          .inspector()
          .GetCollisionCandidates()
          .size();
  minimum_value_constraint_ = std::make_unique<solvers::MinimumValueConstraint>(
      this->num_vars(), minimum_distance, influence_distance_offset,
      num_collision_candidates,
      [&plant, plant_context](const auto& x, double influence_distance) {
        return Distances<T, AutoDiffXd>(plant, plant_context, x,
                                        influence_distance);
      },
      [&plant, plant_context](const auto& x, double influence_distance) {
        return Distances<T, double>(plant, plant_context, x,
                                    influence_distance);
      });
  this->set_bounds(minimum_value_constraint_->lower_bound(),
                   minimum_value_constraint_->upper_bound());
  if (penalty_function) {
    minimum_value_constraint_->set_penalty_function(penalty_function);
  }
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance, systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_double_{plant},
      plant_context_double_{plant_context},
      plant_autodiff_{nullptr},
      plant_context_autodiff_{nullptr} {
  Initialize(*plant_double_, plant_context_double_, minimum_distance,
             influence_distance_offset, penalty_function);
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<AutoDiffXd>* const plant,
    double minimum_distance, systems::Context<AutoDiffXd>* plant_context,
    MinimumDistancePenaltyFunction penalty_function,
    double influence_distance_offset)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_double_{nullptr},
      plant_context_double_{nullptr},
      plant_autodiff_{plant},
      plant_context_autodiff_{plant_context} {
  Initialize(*plant_autodiff_, plant_context_autodiff_, minimum_distance,
             influence_distance_offset, penalty_function);
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
