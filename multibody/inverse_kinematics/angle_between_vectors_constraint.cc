#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::NormalizeVector;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& a_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& b_B, double angle_lower,
    double angle_upper, systems::Context<double>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(std::cos(angle_upper)),
                          Vector1d(std::cos(angle_lower))),
      plant_double_((plant)),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      a_unit_A_(NormalizeVector(a_A)),
      b_unit_B_(NormalizeVector(b_B)),
      context_double_(plant_context),
      plant_autodiff_{nullptr},
      context_autodiff_{nullptr} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::invalid_argument(
        "AngleBetweenVectorsConstraint: should satisfy 0 <= angle_lower <= "
        "angle_upper <= pi");
  }
}

AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& a_A,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& b_B, double angle_lower,
    double angle_upper, systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(std::cos(angle_upper)),
                          Vector1d(std::cos(angle_lower))),
      plant_double_(nullptr),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      a_unit_A_(NormalizeVector(a_A)),
      b_unit_B_(NormalizeVector(b_B)),
      context_double_(nullptr),
      plant_autodiff_{plant},
      context_autodiff_{plant_context} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::invalid_argument(
        "AngleBetweenVectorsConstraint: should satisfy 0 <= angle_lower <= "
        "angle_upper <= pi");
  }
}

void EvalConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frameA,
    const Frame<double>& frameB, const Eigen::Vector3d& a_unit_A,
    const Eigen::Vector3d& b_unit_B, const math::RotationMatrix<double>& R_AB,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  // The constraint function is
  //   g(q) = a_unit_Aᵀ * R_AB(q) * b_unit_B.
  // To derive the Jacobian of g w.r.t. q, ∂g/∂q, we first differentiate
  // w.r.t. time to obtain
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
  // By the chain rule, ġ = ∂g/∂q q̇. Comparing this with the previous
  // equation, we see that
  //   ∂g/∂q = (b_unit_A × a_unit_A)ᵀ Jq_w_AB.
  Eigen::MatrixXd Jq_V_AB(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(context, JacobianWrtVariable::kQDot, frameB,
                                    Eigen::Vector3d::Zero() /* p_BQ */, frameA,
                                    frameA, &Jq_V_AB);
  const Eigen::Vector3d b_unit_A = R_AB * b_unit_B;
  *y = math::InitializeAutoDiff(
      a_unit_A.transpose() * b_unit_A,
      b_unit_A.cross(a_unit_A).transpose() * Jq_V_AB.topRows<3>() *
          math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frameA_index, const FrameIndex frameB_index,
                   const Eigen::Vector3d& a_unit_A,
                   const Eigen::Vector3d& b_unit_B,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameA = plant.get_frame(frameA_index);
  const Frame<T>& frameB = plant.get_frame(frameB_index);
  const math::RotationMatrix<T> R_AB =
      plant.CalcRelativeRotationMatrix(*context, frameA, frameB);

  // Note: The expression below has quantities with different scalar types.
  // The cast from `double` to `T` preserves derivative or symbolic information
  // in R_AB (if it exists).
  const Vector3<T> b_unit_A = R_AB * b_unit_B.cast<T>();
  *y = a_unit_A.transpose() * b_unit_A;
  if constexpr (!std::is_same_v<T, S>) {
    EvalConstraintGradient(*context, plant, frameA, frameB, a_unit_A, b_unit_B,
                           R_AB, x, y);
  }
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(math::InitializeAutoDiff(x), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  a_unit_A_, b_unit_B_, x, y);
  }
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameA_index_,
                  frameB_index_, a_unit_A_, b_unit_B_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameA_index_, frameB_index_,
                  a_unit_A_, b_unit_B_, x, y);
  }
}
}  // namespace multibody
}  // namespace drake
