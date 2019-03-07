#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    const double exp_one_over_x = std::exp(1.0 / x);
    *penalty = -x * exp_one_over_x;
    if (dpenalty_dx) {
      *dpenalty_dx = -exp_one_over_x + exp_one_over_x / x;
    }
  }
}

void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    if (x > -1) {
      *penalty = x * x / 2;
      if (dpenalty_dx) {
        *dpenalty_dx = x;
      }
    } else {
      *penalty = -0.5 - x;
      if (dpenalty_dx) {
        *dpenalty_dx = -1;
      }
    }
  }
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance, systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(0)),
      plant_{RefFromPtrOrThrow(plant)},
      minimum_distance_{minimum_distance},
      plant_context_{plant_context},
      penalty_function_{penalty_function} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  if (minimum_distance_ <= 0) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: minimum_distance should be positive.");
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
                double minimum_distance,
                MinimumDistancePenaltyFunction penalty_function,
                const Eigen::Vector3d&, const Eigen::Vector3d&,
                const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y) {
  double penalty;
  const double x = distance / minimum_distance - 1;
  penalty_function(x, &penalty, nullptr);
  (*y)(0) += penalty;
}

void AddPenalty(const MultibodyPlant<double>& plant,
                const systems::Context<double>& context,
                const Frame<double>& frameA, const Frame<double>& frameB,
                const Eigen::Vector3d& p_ACa, double distance,
                double minimum_distance,
                MinimumDistancePenaltyFunction penalty_function,
                const Eigen::Vector3d& p_WCa, const Eigen::Vector3d& p_WCb,
                const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  // The distance is d = sign * |p_CbCa_B|, where the
  // closest points are Ca on object A, and Cb on object B.
  // So the gradient ∂d/∂q = p_CbCa_W * ∂p_BCa_B/∂q / d
  // where p_CbCa_W = p_WCa - p_WCb = p_WA + R_WA * p_ACa - (p_WB + R_WB *
  // p_BCb)
  double penalty, dpenalty_dx;
  const double penalty_x = distance / minimum_distance - 1;
  penalty_function(penalty_x, &penalty, &dpenalty_dx);
  const double dpenalty_ddistance = dpenalty_dx / minimum_distance;

  Eigen::Matrix<double, 6, Eigen::Dynamic> Jq_V_BCa_W(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frameA,
                                    p_ACa, frameB, plant.world_frame(),
                                    &Jq_V_BCa_W);
  const Eigen::RowVectorXd ddistance_dq =
      (p_WCa - p_WCb).transpose() * Jq_V_BCa_W.bottomRows<3>() / distance;
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
          query_object.ComputeSignedDistancePairwiseClosestPoints();

  InitializeY(x, y);

  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const double distance = signed_distance_pair.distance;
    if (distance < minimum_distance_) {
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
                 distance, minimum_distance_, penalty_function_, p_WCa, p_WCb,
                 x, y);
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
