#include "drake/multibody/inverse_kinematics/position_cost.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {

PositionCost::PositionCost(const MultibodyPlant<double>* const plant,
                           const Frame<double>& frameA,
                           const Eigen::Ref<const Eigen::Vector3d>& p_AP,
                           const Frame<double>& frameB,
                           const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                           const Eigen::Ref<const Eigen::Matrix3d>& C,
                           systems::Context<double>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      plant_double_(plant),
      frameA_index_(frameA.index()),
      p_AP_{p_AP},
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      C_{C},
      context_double_{plant_context},
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
  if (plant_context == nullptr)
    throw std::invalid_argument(
        "PositionCost(): plant_context is nullptr.");
}

PositionCost::PositionCost(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AP,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Eigen::Ref<const Eigen::Matrix3d>& C,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      plant_double_(nullptr),
      frameA_index_(frameA.index()),
      p_AP_{p_AP},
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ},
      C_{C},
      context_double_{nullptr},
      plant_autodiff_(plant),
      context_autodiff_(plant_context) {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
}

PositionCost::~PositionCost() = default;

namespace {

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameA_index, const Eigen::Vector3d& p_AP,
                   const FrameIndex frameB_index, const Eigen::Vector3d& p_BQ,
                   const Eigen::Matrix3d& C,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameA = plant.get_frame(frameA_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  Vector3<T> p_AQ;
  plant.CalcPointsPositions(*context, frameB, p_BQ.cast<T>(), frameA,
                            &p_AQ);
  const Vector3<T> err = p_AQ - p_AP;
  if constexpr (std::is_same_v<T, S>) {
    *y = err.transpose() * C * err;
  } else {
    static_assert(std::is_same_v<T, double>);
    static_assert(std::is_same_v<S, AutoDiffXd>);
    Eigen::Matrix3Xd Jq_v_ABq(3, plant.num_positions());
    plant.CalcJacobianTranslationalVelocity(*context,
                                            JacobianWrtVariable::kQDot, frameB,
                                            p_BQ, frameA, frameA, &Jq_v_ABq);
    const Vector3<S> err_ad =
        math::InitializeAutoDiff(err, Jq_v_ABq * math::ExtractGradient(x));
    *y = err_ad.transpose() * C * err_ad;
  }
}

}  // namespace

void PositionCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, p_AP_,
                  frameB_index_, p_BQ_, C_, x, y);
  }
}

void PositionCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  if (!use_autodiff()) {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, p_AP_,
                  frameB_index_, p_BQ_, C_, x, y);
  } else {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameA_index_, p_AP_,
                  frameB_index_, p_BQ_, C_, x, y);
  }
}

void PositionCost::DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
                          VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "PositionCost::DoEval() does not work for symbolic variables.");
}

}  // namespace multibody
}  // namespace drake
