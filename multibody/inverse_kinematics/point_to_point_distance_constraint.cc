#include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
PointToPointDistanceConstraint::PointToPointDistanceConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frame1,
    const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
    const Frame<double>& frame2,
    const Eigen::Ref<const Eigen::Vector3d>& p_B2P2, double distance_lower,
    double distance_upper, systems::Context<double>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(distance_lower * distance_lower),
                          Vector1d(distance_upper * distance_upper)),
      plant_double_(plant),
      frame1_index_(frame1.index()),
      frame2_index_(frame2.index()),
      p_B1P1_(p_B1P1),
      p_B2P2_(p_B2P2),
      context_double_(plant_context),
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
  if (plant_context == nullptr) {
    throw std::invalid_argument("plant_context is nullptr");
  }
  DRAKE_DEMAND(distance_lower >= 0);
  DRAKE_DEMAND(distance_upper >= distance_lower);
}

PointToPointDistanceConstraint::PointToPointDistanceConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frame1,
    const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
    const Frame<AutoDiffXd>& frame2,
    const Eigen::Ref<const Eigen::Vector3d>& p_B2P2, double distance_lower,
    double distance_upper, systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(distance_lower * distance_lower),
                          Vector1d(distance_upper * distance_upper)),
      plant_double_(nullptr),
      frame1_index_(frame1.index()),
      frame2_index_(frame2.index()),
      p_B1P1_(p_B1P1),
      p_B2P2_(p_B2P2),
      context_double_{nullptr},
      plant_autodiff_(plant),
      context_autodiff_(plant_context) {
  if (plant_context == nullptr) {
    throw std::invalid_argument("plant_context is nullptr");
  }
  DRAKE_DEMAND(distance_lower >= 0);
  DRAKE_DEMAND(distance_upper >= distance_lower);
}

template <typename T>
void EvalPointToPointDistanceConstraintGradient(
    const systems::Context<T>&, const MultibodyPlant<T>&, const Frame<T>&,
    const Frame<T>&, const Eigen::Vector3d&, const Vector3<T>& p_P1P2_B1,
    const Eigen::Ref<const VectorX<T>>&, VectorX<T>* y) {
  (*y)(0) = p_P1P2_B1.squaredNorm();
}

void EvalPointToPointDistanceConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frame1,
    const Frame<double>& frame2, const Eigen::Vector3d& p_B2P2,
    const Eigen::Vector3d& p_P1P2_B1, const Eigen::Ref<const AutoDiffVecXd>& x,
    AutoDiffVecXd* y) {
  Eigen::MatrixXd Jq_V_B1P2(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frame2,
                                    p_B2P2, frame1, frame1, &Jq_V_B1P2);
  *y = math::InitializeAutoDiff(Vector1d(p_P1P2_B1.squaredNorm()),
                                2 * p_P1P2_B1.transpose() *
                                    Jq_V_B1P2.bottomRows<3>() *
                                    math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frame1_index, const Eigen::Vector3d& p_B1P1,
                   const FrameIndex frame2_index, const Eigen::Vector3d& p_B2P2,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frame1 = plant.get_frame(frame1_index);
  const Frame<T>& frame2 = plant.get_frame(frame2_index);
  // Compute p_B1P2
  Vector3<T> p_B1P2;
  plant.CalcPointsPositions(*context, frame2, p_B2P2.cast<T>(), frame1,
                            &p_B1P2);
  const Vector3<T> p_P1P2_B1 = p_B1P2 - p_B1P1;
  EvalPointToPointDistanceConstraintGradient(*context, plant, frame1, frame2,
                                             p_B2P2, p_P1P2_B1, x, y);
}

void PointToPointDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(math::InitializeAutoDiff(x), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frame1_index_, p_B1P1_,
                  frame2_index_, p_B2P2_, x, y);
  }
}

void PointToPointDistanceConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frame1_index_, p_B1P1_,
                  frame2_index_, p_B2P2_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frame1_index_, p_B1P1_,
                  frame2_index_, p_B2P2_, x, y);
  }
}
}  // namespace multibody
}  // namespace drake
