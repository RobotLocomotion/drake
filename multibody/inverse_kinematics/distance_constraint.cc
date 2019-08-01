#include "drake/multibody/inverse_kinematics/distance_constraint.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/inverse_kinematics/distance_constraint_utilities.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

namespace {
const double kInf = std::numeric_limits<double>::infinity();
}  // namespace

DistanceConstraint::DistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    SortedPair<geometry::GeometryId> geometry_pair,
    systems::Context<double>* plant_context, double distance_lower,
    double distance_upper)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(distance_lower), Vector1d(distance_upper)),
      plant_double_{plant},
      plant_context_double_{plant_context},
      geometry_pair_{std::move(geometry_pair)},
      plant_autodiff_{nullptr},
      plant_context_autodiff_{nullptr} {
  CheckPlantIsConnectedToSceneGraph(*plant_double_, *plant_context_double_);
}

DistanceConstraint::DistanceConstraint(
    const multibody::MultibodyPlant<AutoDiffXd>* const plant,
    SortedPair<geometry::GeometryId> geometry_pair,
    systems::Context<AutoDiffXd>* plant_context, double distance_lower,
    double distance_upper)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(distance_lower), Vector1d(distance_upper)),
      plant_double_{nullptr},
      plant_context_double_{nullptr},
      geometry_pair_{std::move(geometry_pair)},
      plant_autodiff_{plant},
      plant_context_autodiff_{plant_context} {
  CheckPlantIsConnectedToSceneGraph(*plant_autodiff_, *plant_context_autodiff_);
}

template <typename T, typename S>
void EvalDistance(const MultibodyPlant<T>& plant,
                  const SortedPair<geometry::GeometryId>& geometry_pair,
                  systems::Context<T>* context,
                  const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  internal::UpdateContextConfiguration(context, plant, x);
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<geometry::QueryObject<T>>(*context);
  // TODO(hongkai.dai): call the distance for the single pair instead for all
  // pairs when SceneGraph provides the API.
  const std::vector<geometry::SignedDistancePair<T>> signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(kInf);
  bool found_pair = false;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (SortedPair<geometry::GeometryId>(signed_distance_pair.id_A,
                                         signed_distance_pair.id_B) ==
        geometry_pair) {
      found_pair = true;
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
          signed_distance_pair.distance, signed_distance_pair.nhat_BA_W, x,
          y->data());
      break;
    }
  }
  if (!found_pair) {
    throw std::runtime_error(
        "DistanceConstraint::DoEvalGeneric(): SceneGraph did not report the "
        "distance between the pair of geometry (" +
        std::to_string(geometry_pair.first().get_value()) + ", " +
        std::to_string(geometry_pair.second().get_value()) + ")");
  }
}

template <typename T>
void DistanceConstraint::DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                                       VectorX<T>* y) const {
  if (use_autodiff()) {
    EvalDistance<AutoDiffXd, T>(*plant_autodiff_, geometry_pair_,
                                plant_context_autodiff_, x, y);
  } else {
    EvalDistance<double, T>(*plant_double_, geometry_pair_,
                            plant_context_double_, x, y);
  }
}

void DistanceConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void DistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
