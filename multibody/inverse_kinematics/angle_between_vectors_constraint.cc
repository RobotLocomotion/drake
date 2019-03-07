#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::NormalizeVector;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const MultibodyPlant<double>* const plant,
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& a_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& b_B,
    double angle_lower, double angle_upper, systems::Context<double>* context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(std::cos(angle_upper)),
                          Vector1d(std::cos(angle_lower))),
      plant_(RefFromPtrOrThrow(plant)),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      a_unit_A_(NormalizeVector(a_A)),
      b_unit_B_(NormalizeVector(b_B)),
      context_(context) {
  if (context == nullptr) throw std::invalid_argument("context is nullptr.");
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::invalid_argument(
        "AngleBetweenVectorsConstraint: should satisfy 0 <= angle_lower <= "
        "angle_upper <= pi");
  }
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  // TODO(avalenzu): Re-work to avoid round-trip through AutoDiffXd (#10205).
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  // The constraint function is
  //   g(q) = a_unit_Aᵀ * R_AB(q) * b_unit_B.
  // To derive the Jacobian of g w.r.t. q, ∂g/∂q, we first differentiate w.r.t.
  // time to obtain
  //   ġ = a_unit_Aᵀ Ṙ_AB b_unit_B,
  // where we have dropped the dependence on q for readability. Next we expand
  // Ṙ_AB to yield
  //   ġ =  a_unit_Aᵀ ω̂_AB R_AB b_unit_B,
  // where ω̂_AB is the skew-symmetric, cross-product matrix such that
  // (ω̂_AB) r = ω_AB × r; ω_AB, r ∈ ℝ³. Applying R_AB to b_unit_B gives the
  // following triple product
  //   ġ = a_unit_A ⋅ (ω_AB × b_unit_A),
  // which can then be rearranged to yield
  //   ġ =  b_unit_A ⋅ (a_unit_A × ω_AB)  (circular shift)
  //     = (b_unit_A × a_unit_A) ⋅ ω_AB   (swap operators).
  // The angular velocity of B relative to A can be written as
  //   ω_AB = Jq_w_AB(q) q̇,
  // where Jq_w_AB is the Jacobian of ω_AB with respect to q̇. Therefore,
  //   ġ = (b_unit_A × a_unit_A)ᵀ Jq_w_AB q̇.
  // By the chain rule, ġ = ∂g/∂q q̇. Comparing this with the previous equation,
  // we see that
  //   ∂g/∂q = (b_unit_A × a_unit_A)ᵀ Jq_w_AB.
  y->resize(1);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  const Frame<double>& frameA = plant_.get_frame(frameA_index_);
  const Frame<double>& frameB = plant_.get_frame(frameB_index_);
  const Matrix3<double> R_AB =
      plant_.CalcRelativeTransform(*context_, frameA, frameB).linear();
  Eigen::MatrixXd Jq_V_AB(6, plant_.num_positions());
  plant_.CalcJacobianSpatialVelocity(
      *context_, JacobianWrtVariable::kQDot, frameB,
      Eigen::Vector3d::Zero() /* p_BQ */, frameA, frameA, &Jq_V_AB);
  const Eigen::Vector3d b_unit_A = R_AB * b_unit_B_;
  *y = math::initializeAutoDiffGivenGradientMatrix(
      a_unit_A_.transpose() * b_unit_A, b_unit_A.cross(a_unit_A_).transpose() *
                                            Jq_V_AB.topRows<3>() *
                                            math::autoDiffToGradientMatrix(x));
}
}  // namespace multibody
}  // namespace drake
