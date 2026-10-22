#include "drake/multibody/inverse_kinematics/distance_constraint_utilities.h"

#include <type_traits>
#include <vector>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {
// Computes ∂distance/∂q for the pair whose witness point on geometry A is
// p_ACa, using Jq_v_BCa_W (3 × nq) as scratch.
void CalcDdistanceDq(const MultibodyPlant<double>& plant,
                     const systems::Context<double>& context,
                     const Frame<double>& frameA, const Frame<double>& frameB,
                     const Eigen::Vector3d& p_ACa,
                     const Eigen::Vector3d& nhat_BA_W,
                     Eigen::Matrix3Xd* Jq_v_BCa_W,
                     EigenPtr<Eigen::RowVectorXd> ddistance_dq) {
  // Derivation to compute the gradient of the signed distance function w.r.t q:
  // The distance is
  // d = n̂_BA_Wᵀ * (p_WCa - p_WCb)             (1)
  //   = n̂_BA_Wᵀ * p_CbCa_W
  // where the Ca is the witness point on geometry A, and Cb is the witness
  // point on geometry B.
  // Hence when we take the gradient on both sides
  // ∂d / ∂q =   p_CbCa_Wᵀ * (∂n̂_BA_W / ∂q) + n̂_BA_Wᵀ * (∂p_CbCa_W /∂q)   (2)
  // Using the fact that p_CbCa_W = d * n̂_BA_W, we have
  // ∂d / ∂q = d * n̂_BA_Wᵀ * (∂n̂_BA_W / ∂q) + n̂_BA_Wᵀ * (∂p_CbCa_W /∂q)   (3)
  // We also know that for whatever q, the normal n̂_BA_W is always a unit length
  // vector, hence n̂_BA_Wᵀ * n̂_BA_W = 1 for whatever q. This invariance means
  // that the gradient of (n̂_BA_Wᵀ * n̂_BA_W) w.r.t q is zero, namely
  // ∂1 / ∂q = ∂(n̂_BA_Wᵀ * n̂_BA_W) / ∂q = 2 * n̂_BA_Wᵀ * (∂n̂_BA_W / ∂q)
  // Since ∂1 / ∂q = 0, we conclude that n̂_BA_Wᵀ * (∂n̂_BA_W / ∂q) = 0, hence the
  // first term in the right-hand side equation (3) disappears. Thus
  // ∂d / ∂q = n̂_BA_Wᵀ * (∂p_CbCa_W / ∂q)                                  (4)
  // Note that ∂p_CbCa_W / ∂q is computed as a Jacobian matrix in
  // MultibodyPlant.
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameA, p_ACa, frameB,
                                          plant.world_frame(), Jq_v_BCa_W);
  ddistance_dq->noalias() = nhat_BA_W.transpose() * *Jq_v_BCa_W;
}
}  // namespace

void CalcDistanceDerivatives(const MultibodyPlant<double>& plant,
                             const systems::Context<double>& context,
                             const Frame<double>& frameA,
                             const Frame<double>& frameB,
                             const Eigen::Vector3d& p_ACa, double distance,
                             const Eigen::Vector3d& nhat_BA_W,
                             const Eigen::Ref<const AutoDiffVecXd>& q,
                             AutoDiffXd* distance_autodiff) {
  Eigen::Matrix3Xd Jq_v_BCa_W(3, plant.num_positions());
  Eigen::RowVectorXd ddistance_dq(plant.num_positions());
  CalcDdistanceDq(plant, context, frameA, frameB, p_ACa, nhat_BA_W, &Jq_v_BCa_W,
                  &ddistance_dq);
  distance_autodiff->value() = distance;
  distance_autodiff->derivatives() = ddistance_dq * math::ExtractGradient(q);
}

void CalcDistanceDerivatives(const MultibodyPlant<AutoDiffXd>&,
                             const systems::Context<AutoDiffXd>&,
                             const Frame<AutoDiffXd>&, const Frame<AutoDiffXd>&,
                             const Vector3<AutoDiffXd>&,
                             const AutoDiffXd& distance_autodiff,
                             const Vector3<AutoDiffXd>&,
                             const Eigen::Ref<const Eigen::VectorXd>&,
                             double* distance) {
  *distance = distance_autodiff.value();
}

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
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();

  const std::vector<geometry::SignedDistancePair<T>> signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(
          influence_distance);
  const int num_pairs = std::ssize(signed_distance_pairs);
  constexpr bool kDoubleToAutoDiff =
      std::is_same_v<T, double> && std::is_same_v<S, AutoDiffXd>;
  VectorX<T> distances(num_pairs);
  // Row-major so that each row is contiguous and can be passed to
  // CalcDdistanceDq() as an EigenPtr<RowVectorXd>.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      ddistance_dq;
  Eigen::Matrix3Xd Jq_v_BCa_W;
  if constexpr (kDoubleToAutoDiff) {
    ddistance_dq.resize(num_pairs, plant.num_positions());
    Jq_v_BCa_W.resize(3, plant.num_positions());
  }
  for (int i = 0; i < num_pairs; ++i) {
    const geometry::SignedDistancePair<T>& pair = signed_distance_pairs[i];
    distances(i) = pair.distance;
    if constexpr (kDoubleToAutoDiff) {
      const Frame<double>& frameA =
          plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_A))
              ->body_frame();
      const Frame<double>& frameB =
          plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_B))
              ->body_frame();
      const Eigen::Vector3d p_ACa =
          inspector.GetPoseInFrame(pair.id_A) * pair.p_ACa;
      auto ddistance_dq_row = ddistance_dq.row(i);
      CalcDdistanceDq(plant, *context, frameA, frameB, p_ACa, pair.nhat_BA_W,
                      &Jq_v_BCa_W, &ddistance_dq_row);
    }
  }
  if constexpr (kDoubleToAutoDiff) {
    return math::InitializeAutoDiff(distances,
                                    ddistance_dq * math::ExtractGradient(q));
  } else if constexpr (std::is_same_v<T, S>) {
    return distances;
  } else {
    return math::ExtractValue(distances);
  }
}

Eigen::VectorXd Distances(
    const planning::CollisionChecker& collision_checker,
    planning::CollisionCheckerContext* collision_checker_context,
    const Eigen::Ref<const Eigen::VectorXd>& x, double influence_distance_val) {
  return collision_checker
      .CalcContextRobotClearance(collision_checker_context, x,
                                 influence_distance_val)
      .distances();
}

AutoDiffVecXd Distances(
    const planning::CollisionChecker& collision_checker,
    planning::CollisionCheckerContext* collision_checker_context,
    const Eigen::Ref<const AutoDiffVecXd>& x, double influence_distance_val) {
  const planning::RobotClearance robot_clearance =
      collision_checker.CalcContextRobotClearance(collision_checker_context,
                                                  math::ExtractValue(x),
                                                  influence_distance_val);
  return math::InitializeAutoDiff(
      robot_clearance.distances(),
      robot_clearance.jacobians() * math::ExtractGradient(x));
}

// Explicit instantiation
template VectorX<double> Distances<double, double>(
    const MultibodyPlant<double>&, systems::Context<double>*,
    const Eigen::Ref<const VectorX<double>>&, double);
template VectorX<AutoDiffXd> Distances<double, AutoDiffXd>(
    const MultibodyPlant<double>&, systems::Context<double>*,
    const Eigen::Ref<const VectorX<AutoDiffXd>>&, double);
template VectorX<double> Distances<AutoDiffXd, double>(
    const MultibodyPlant<AutoDiffXd>&, systems::Context<AutoDiffXd>*,
    const Eigen::Ref<const VectorX<double>>&, double);
template VectorX<AutoDiffXd> Distances<AutoDiffXd, AutoDiffXd>(
    const MultibodyPlant<AutoDiffXd>&, systems::Context<AutoDiffXd>*,
    const Eigen::Ref<const VectorX<AutoDiffXd>>&, double);
}  // namespace internal
}  // namespace multibody
}  // namespace drake
