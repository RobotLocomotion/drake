#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::NormalizeVector;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
GazeTargetConstraint::GazeTargetConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle,
    systems::Context<double>* plant_context)
    : solvers::Constraint(
          2, RefFromPtrOrThrow(plant).num_positions(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      plant_double_{plant},
      frameA_index_{frameA.index()},
      frameB_index_{frameB.index()},
      p_AS_{p_AS},
      n_A_{NormalizeVector(n_A)},
      p_BT_{p_BT},
      cone_half_angle_{cone_half_angle},
      cos_cone_half_angle_{std::cos(cone_half_angle_)},
      context_double_{plant_context},
      plant_autodiff_{nullptr},
      context_autodiff_{nullptr} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (cone_half_angle < 0 || cone_half_angle > M_PI_2) {
    throw std::invalid_argument(
        "GazeTargetConstraint: cone_half_angle should be within [0, pi/2]");
  }
}

GazeTargetConstraint::GazeTargetConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(
          2, RefFromPtrOrThrow(plant).num_positions(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      plant_double_{nullptr},
      frameA_index_{frameA.index()},
      frameB_index_{frameB.index()},
      p_AS_{p_AS},
      n_A_{NormalizeVector(n_A)},
      p_BT_{p_BT},
      cone_half_angle_{cone_half_angle},
      cos_cone_half_angle_{std::cos(cone_half_angle_)},
      context_double_{nullptr},
      plant_autodiff_{plant},
      context_autodiff_{plant_context} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (cone_half_angle < 0 || cone_half_angle > M_PI_2) {
    throw std::invalid_argument(
        "GazeTargetConstraint: cone_half_angle should be within [0, pi/2]");
  }
}

void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frameA,
    const Frame<double>& frameB, const Eigen::Vector3d& p_BT,
    const Eigen::Vector3d& n_A,
    const Eigen::Vector3d& cos_cone_half_angle_squared_times_p,
    const double& p_dot_n, const Eigen::Vector2d& g,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  // The constraint values are
  //   g(q)  = ⎡ p · n_unit_A                                 ⎤
  //           ⎣(p · n_unit_A)² - (cosθ)²p · p⎦
  // where p is an abbreviation for p_ST_A(q). The partial derivative of g,
  // ∂g/∂q, is therefore given by
  //   ∂g/∂q = ⎡n_unit_Aᵀ                               ⎤ ∂p/∂q,
  //           ⎣2 [(p · n_unit_A) n_unit_A -  (cosθ)²p]ᵀ⎦
  // Position of target point T measured and expressed in A frame.
  Eigen::MatrixXd Jq_V_ABt(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frameB,
                                    p_BT, frameA, frameA, &Jq_V_ABt);
  // Jq_p = Jq_v_ABt = ∂p/∂q.
  const Matrix3X<double> Jq_p = Jq_V_ABt.bottomRows<3>();
  // J_g_p = ∂g/∂p.
  const Eigen::Matrix<double, 2, 3> Jp_g =
      (Eigen::Matrix<double, 2, 3>() << n_A.transpose(),
       2 * (p_dot_n * n_A - cos_cone_half_angle_squared_times_p).transpose())
          .finished();

  *y = math::InitializeAutoDiff(g, Jp_g * Jq_p * math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameA_index, const FrameIndex frameB_index,
                   const Eigen::Vector3d& p_AS, const Eigen::Vector3d& n_A,
                   const Eigen::Vector3d p_BT, double cos_cone_half_angle,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameA = plant.get_frame(frameA_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  // Position of target point T measured and expressed in A frame.
  Vector3<T> p_AT;
  plant.CalcPointsPositions(*context, frameB, p_BT.cast<T>(), frameA, &p_AT);
  const Vector3<T> p_ST_A = p_AT - p_AS;
  const T p_dot_n = p_ST_A.dot(n_A);
  const Vector3<T> cos_cone_half_angle_squared_times_p =
      std::pow(cos_cone_half_angle, 2) * p_ST_A;
  // Constraint values, g.
  const Vector2<T> g{
      p_dot_n,
      p_dot_n * p_dot_n - cos_cone_half_angle_squared_times_p.dot(p_ST_A)};
  if constexpr (std::is_same_v<T, S>) {
    *y = g;
  } else {
    EvalConstraintGradient(*context, plant, frameA, frameB, p_BT, n_A,
                           cos_cone_half_angle_squared_times_p, p_dot_n, g, x,
                           y);
  }
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(math::InitializeAutoDiff(x), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  p_AS_, n_A_, p_BT_, cos_cone_half_angle_, x, y);
  }
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                  AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameA_index_,
                  frameB_index_, p_AS_, n_A_, p_BT_, cos_cone_half_angle_, x,
                  y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  p_AS_, n_A_, p_BT_, cos_cone_half_angle_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
