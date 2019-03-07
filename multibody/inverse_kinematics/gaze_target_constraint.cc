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
    const MultibodyPlant<double>* const plant,
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle,
    systems::Context<double>* context)
    : solvers::Constraint(
          2, RefFromPtrOrThrow(plant).num_positions(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      plant_{RefFromPtrOrThrow(plant)},
      frameA_index_{frameA.index()},
      frameB_index_{frameB.index()},
      p_AS_{p_AS},
      n_A_{NormalizeVector(n_A)},
      p_BT_{p_BT},
      cone_half_angle_{cone_half_angle},
      cos_cone_half_angle_{std::cos(cone_half_angle_)},
      context_{context} {
  if (context == nullptr) throw std::invalid_argument("context is nullptr.");
  if (cone_half_angle < 0 || cone_half_angle > M_PI_2) {
    throw std::invalid_argument(
        "GazeTargetConstraint: cone_half_angle should be within [0, pi/2]");
  }
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  // TODO(avalenzu): Re-work to avoid round-trip through AutoDiffXd (#10205).
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                  AutoDiffVecXd* y) const {
  // The constraint values are
  //   g(q)  = ⎡ p · n_unit_A                                 ⎤
  //           ⎣(p · n_unit_A)² - (cosθ)²p · p⎦
  // where p is an abbreviation for p_ST_A(q). The partial derivative of g,
  // ∂g/∂q, is therefore given by
  //   ∂g/∂q = ⎡n_unit_Aᵀ                               ⎤ ∂p/∂q,
  //           ⎣2 [(p · n_unit_A) n_unit_A -  (cosθ)²p]ᵀ⎦
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  const Frame<double>& frameA = plant_.get_frame(frameA_index_);
  const Frame<double>& frameB = plant_.get_frame(frameB_index_);
  // Position of target point T measured and expressed in A frame.
  Vector3<double> p_AT;
  plant_.CalcPointsPositions(*context_, frameB, p_BT_, frameA, &p_AT);
  Eigen::MatrixXd Jq_V_ABt(6, plant_.num_positions());
  plant_.CalcJacobianSpatialVelocity(*context_,
                                     JacobianWrtVariable::kQDot, frameB,
                                     p_BT_, frameA, frameA, &Jq_V_ABt);
  // Jq_p = Jq_v_ABt = ∂p/∂q.
  const Matrix3X<double> Jq_p = Jq_V_ABt.bottomRows<3>();
  const Vector3<double> p_ST_A = p_AT - p_AS_;
  const double p_dot_n = p_ST_A.dot(n_A_);
  const Vector3<double> cos_cone_half_angle_squared_times_p =
      std::pow(cos_cone_half_angle_, 2) * p_ST_A;
  // Constraint values, g.
  const Vector2<double> g{
      p_dot_n,
      p_dot_n * p_dot_n - cos_cone_half_angle_squared_times_p.dot(p_ST_A)};
  // J_g_p = ∂g/∂p.
  const Eigen::Matrix<double, 2, 3> Jp_g =
      (Eigen::Matrix<double, 2, 3>() << n_A_.transpose(),
       2 * (p_dot_n * n_A_ - cos_cone_half_angle_squared_times_p).transpose())
          .finished();

  *y = math::initializeAutoDiffGivenGradientMatrix(
      g, Jp_g * Jq_p * math::autoDiffToGradientMatrix(x));
}

}  // namespace multibody
}  // namespace drake
