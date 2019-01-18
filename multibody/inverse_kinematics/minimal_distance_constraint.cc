#include "drake/multibody/inverse_kinematics/minimal_distance_constraint.h"

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

MinimalDistanceConstraint::MinimalDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimal_distance, systems::Context<double>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_{RefFromPtrOrThrow(plant)},
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

void InitializeY(const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y) {
  (*y)(0) = 0;
}

void InitializeY(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  (*y) = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(0), Eigen::RowVectorXd::Zero(x(0).derivatives().size()));
}
void AddPenalty(const MultibodyPlant<double>&, const systems::Context<double>&,
                const Frame<double>&, const Frame<double>&,
                const Eigen::Vector3d&, const Eigen::Vector3d&,
                const Frame<double>&, double distance, double minimal_distance,
                const Eigen::Vector3d&, const Eigen::Vector3d&,
                const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y) {
  double penalty;
  Penalty(distance, minimal_distance, &penalty, nullptr);
  (*y)(0) += penalty;
}

void AddPenalty(const MultibodyPlant<double>& plant,
                const systems::Context<double>& context,
                const Frame<double>& frameA, const Frame<double>& frameB,
                const Eigen::Vector3d& p_ACa, const Eigen::Vector3d& p_BCb,
                const Frame<double>& world_frame, double distance,
                double minimal_distance, const Eigen::Vector3d& p_WCa,
                const Eigen::Vector3d& p_WCb,
                const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  // The distance is d = sign * |p_CbCa_W| = sign * |p_WCa - p_WCb|, where the
  // closest points are Ca on object A, and Cb on object B. Namely
  // d = sign * |p_WA + R_WA * p_ACa - p_WB + R_WB * p_BCb|
  // So the gradient ∂d/∂q = p_CaCb_W * (∂p_WCa/∂q - ∂p_WCb/∂q) / d
  // where p_CaCb_W = p_WCa - p_WCb = p_WA + R_WA * p_ACa - p_WB + R_WB * p_BCb
  double penalty, dpenalty_ddistance;
  Penalty(distance, minimal_distance, &penalty, &dpenalty_ddistance);

  Eigen::Matrix<double, 6, Eigen::Dynamic> Jq_V_WCa(6, plant.num_positions());
  Eigen::Matrix<double, 6, Eigen::Dynamic> Jq_V_WCb(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frameA,
                                    p_ACa, world_frame, world_frame, &Jq_V_WCa);
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frameB,
                                    p_BCb, world_frame, world_frame, &Jq_V_WCb);

  Eigen::RowVectorXd ddistance_dq =
      (p_WCa - p_WCb).transpose() *
      (Jq_V_WCa.bottomRows<3>() - Jq_V_WCb.bottomRows<3>()) / distance;
  Vector1<AutoDiffXd> penalty_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(
          Vector1d(penalty), dpenalty_ddistance * ddistance_dq *
                                 math::autoDiffToGradientMatrix(x));
  *y += penalty_autodiff;
}

template <typename T>
void MinimalDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);

  internal::UpdateContextConfiguration(plant_context_, plant_, x);
  const geometry::QueryObject<double>& query_object =
      plant_
          .EvalAbstractInput(*plant_context_,
                             plant_.get_geometry_query_input_port().get_index())
          ->GetValue<geometry::QueryObject<double>>();

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
                                 signed_distance_pair.p_ACa,
                                 plant_.world_frame(), &p_WCa);
      plant_.CalcPointsPositions(*plant_context_, frameB,
                                 signed_distance_pair.p_BCb,
                                 plant_.world_frame(), &p_WCb);
      AddPenalty(plant_, *plant_context_, frameA, frameB,
                 signed_distance_pair.p_ACa, signed_distance_pair.p_BCb,
                 plant_.world_frame(), distance, minimal_distance_, p_WCa,
                 p_WCb, x, y);
    }
  }
}

void MinimalDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void MinimalDistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
