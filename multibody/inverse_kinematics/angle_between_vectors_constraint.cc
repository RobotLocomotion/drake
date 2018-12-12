#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::NormalizeVector;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const multibody_plant::MultibodyPlant<double>& plant,
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& na_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& nb_B,
    double angle_lower, double angle_upper, systems::Context<double>* context)
    : solvers::Constraint(1, plant.num_positions(),
                          Vector1d(std::cos(angle_upper)),
                          Vector1d(std::cos(angle_lower))),
      plant_(plant),
      frameA_(frameA),
      na_unit_A_(NormalizeVector(na_A)),
      frameB_(frameB),
      nb_unit_B_(NormalizeVector(nb_B)),
      context_(context) {
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::invalid_argument(
        "AngleBetweenVectorsConstraint: should satisfy 0 <= angle_lower <= "
        "angle_upper <= pi");
  }
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  // The constraint function is
  //   g(q) = na_unit_Aᵀ * R_AB(q) * nb_unit_B.
  // To derive the Jacobian of g w.r.t. q, ∂g/∂q, we first differentiate w.r.t.
  // time to obtain
  //   ġ = na_unit_Aᵀ Ṙ_AB nb_unit_B,
  // where we have dropped the dependence on q for readability. Next we expand
  // Ṙ_AB to yield
  //   ġ =  na_unit_Aᵀ ω̂_AB R_AB nb_unit_B,
  // where ω̂_AB is the skew-symmetric, cross-product matrix such that
  // ω̂_AB r = ω_AB × r ∀ r ∈ ℝ³. Applying R_AB to nb_unit_B gives the following
  // triple product
  //   ġ = na_unit_A ⋅ (ω_AB × nb_unit_A),
  // which can then be rearranged to yield
  //   ġ =  nb_unit_A ⋅ (na_unit_A × ω_AB)  (circular shift)
  //     = (nb_unit_A × na_unit_A) ⋅ ω_AB   (swap operators).
  // The angular velocity of B relative to A can be written as
  //   ω_AB = Jq_w_AB(q) q̇,
  // where Jq_w_AB is the Jacobian of ω_AB with respect to q̇. Therefore,
  //   ġ = (nb_unit_A × na_unit_A)ᵀ Jq_w_AB q̇.
  // By the chain rule, ġ = ∂g/∂q q̇. Comparing this with the previous equation,
  // we see that
  //   ∂g/∂q = (nb_unit_A × na_unit_A)ᵀ Jq_w_AB.
  y->resize(1);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  const Matrix3<double> R_AB =
      plant_.tree().CalcRelativeTransform(*context_, frameA_, frameB_).linear();
  Eigen::MatrixXd Jq_V_AB(6, plant_.num_positions());
  plant_.tree().CalcJacobianSpatialVelocity(
      *context_, JacobianWrtVariable::kQDot, frameB_,
      Eigen::Vector3d::Zero() /* p_BQ */, frameA_, frameA_, &Jq_V_AB);
  const Eigen::Vector3d nb_unit_A = R_AB * nb_unit_B_;
  *y = math::initializeAutoDiffGivenGradientMatrix(
      na_unit_A_.transpose() * nb_unit_A,
      nb_unit_A.cross(na_unit_A_).transpose() * Jq_V_AB.topRows<3>());
}
}  // namespace multibody
}  // namespace drake
