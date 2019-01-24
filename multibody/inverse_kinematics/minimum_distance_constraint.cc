#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;
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
    if (dpenalty_ddistance) {
      *dpenalty_ddistance = 0;
    }
  } else {
    const double x = distance / distance_threshold - 1;
    const double exp_one_over_x = std::exp(1.0 / x);
    *penalty = -x * exp_one_over_x;
    if (dpenalty_ddistance) {
      const double dpenalty_dx = -exp_one_over_x + exp_one_over_x / x;
      *dpenalty_ddistance = dpenalty_dx / distance_threshold;
    }
  }
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimal_distance, systems::Context<double>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_{RefFromPtrOrThrow(plant)},
      minimal_distance_{minimal_distance},
      plant_context_{plant_context} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  if (minimal_distance_ <= 0) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: minimal_distance should be positive.");
  }
}

namespace {
void InitializeY(const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y) {
  (*y)(0) = 0;
}

void InitializeY(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  (*y) = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(0), Eigen::RowVectorXd::Zero(x(0).derivatives().size()));
}
void AddPenalty(const MultibodyPlant<double>&, const systems::Context<double>&,
                const Frame<double>&, const Frame<double>&,
                const Eigen::Vector3d&, double distance,
                double minimal_distance, const Eigen::Vector3d&,
                const Eigen::Vector3d&,
                const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y) {
  double penalty;
  Penalty(distance, minimal_distance, &penalty, nullptr);
  (*y)(0) += penalty;
}

void AddPenalty(const MultibodyPlant<double>& plant,
                const systems::Context<double>& context,
                const Frame<double>& frameA, const Frame<double>& frameB,
                const Eigen::Vector3d& p_ACa, double distance,
                double minimal_distance, const Eigen::Vector3d& p_WCa,
                const Eigen::Vector3d& p_WCb,
                const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  // The distance is d = sign * |p_CbCa_B|, where the
  // closest points are Ca on object A, and Cb on object B.
  // So the gradient ∂d/∂q = p_CbCa_W * ∂p_BCa_B/∂q / d
  // where p_CbCa_W = p_WCa - p_WCb = p_WA + R_WA * p_ACa - (p_WB + R_WB *
  // p_BCb)
  double penalty, dpenalty_ddistance;
  Penalty(distance, minimal_distance, &penalty, &dpenalty_ddistance);

  Eigen::Matrix<double, 6, Eigen::Dynamic> Jq_V_BCa(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frameA,
                                    p_ACa, frameB, frameB, &Jq_V_BCa);

  const Eigen::RowVectorXd ddistance_dq =
      (p_WCa - p_WCb).transpose() * Jq_V_BCa.bottomRows<3>() / distance;
  const Vector1<AutoDiffXd> penalty_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(
          Vector1d(penalty), dpenalty_ddistance * ddistance_dq *
                                 math::autoDiffToGradientMatrix(x));
  *y += penalty_autodiff;
}
}  // namespace

template <typename T>
void MinimumDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);

  internal::UpdateContextConfiguration(plant_context_, plant_, x);
  const AbstractValue* plant_geometry_query_object = plant_.EvalAbstractInput(
      *plant_context_, plant_.get_geometry_query_input_port().get_index());
  if (plant_geometry_query_object == nullptr) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  const geometry::QueryObject<double>& query_object =
      plant_geometry_query_object->GetValue<geometry::QueryObject<double>>();

  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();

  InitializeY(x, y);

  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const double distance = signed_distance_pair.distance;
    if (distance < minimal_distance_) {
      Vector3<double> p_WCa, p_WCb;
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
      plant_.CalcPointsPositions(*plant_context_, frameA,
                                 inspector.X_FG(signed_distance_pair.id_A) *
                                     signed_distance_pair.p_ACa,
                                 plant_.world_frame(), &p_WCa);
      plant_.CalcPointsPositions(*plant_context_, frameB,
                                 inspector.X_FG(signed_distance_pair.id_B) *
                                     signed_distance_pair.p_BCb,
                                 plant_.world_frame(), &p_WCb);
      AddPenalty(plant_, *plant_context_, frameA, frameB,
                 inspector.X_FG(signed_distance_pair.id_A) *
                     signed_distance_pair.p_ACa,
                 distance, minimal_distance_, p_WCa, p_WCb, x, y);
    }
  }
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
