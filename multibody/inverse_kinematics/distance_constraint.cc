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
      plant_{RefFromPtrOrThrow(plant)},
      plant_context_{plant_context},
      geometry_pair_{std::move(geometry_pair)} {
  CheckPlantIsConnectedToSceneGraph(plant_, *plant_context_);
}

template <typename T>
void DistanceConstraint::DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                                       VectorX<T>* y) const {
  y->resize(num_constraints());
  internal::UpdateContextConfiguration(plant_context_, plant_, x);
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(*plant_context_);
  // TODO(hongkai.dai): call the distance for the single pair instead for all
  // pairs when SceneGraph provides the API.
  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints(kInf);
  bool found_pair = false;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (SortedPair<geometry::GeometryId>(signed_distance_pair.id_A,
                                         signed_distance_pair.id_B) ==
        geometry_pair_) {
      found_pair = true;
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
      internal::CalcDistanceDerivatives(
          plant_, *plant_context_, frameA, frameB,
          inspector.X_FG(signed_distance_pair.id_A) *
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
        std::to_string(geometry_pair_.first().get_value()) + ", " +
        std::to_string(geometry_pair_.second().get_value()) + ")");
  }
}

void DistanceConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void DistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
