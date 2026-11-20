#include "drake/multibody/optimization/spatial_velocity_constraint.h"

#include <limits>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/math/autodiff_gradient.h"
#include "drake/math/differentiable_norm.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

namespace drake {
namespace multibody {

using Eigen::VectorXd;
using internal::RefFromPtrOrThrow;
using internal::UpdateContextPositionsAndVelocities;

namespace {

VectorXd MakeLowerBounds(
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_lower,
    const std::optional<SpatialVelocityConstraint::AngularVelocityBounds>&
        w_AC_bounds) {
  if (w_AC_bounds) {
    VectorXd lower(5);
    lower.head<3>() = v_AC_lower;
    DRAKE_THROW_UNLESS(w_AC_bounds->magnitude_lower >= 0);
    lower[3] = std::pow(w_AC_bounds->magnitude_lower, 2);
    lower[4] = std::cos(w_AC_bounds->theta_bound);
    return lower;
  }
  return v_AC_lower;
}

VectorXd MakeUpperBounds(
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_upper,
    const std::optional<SpatialVelocityConstraint::AngularVelocityBounds>&
        w_AC_bounds) {
  if (w_AC_bounds) {
    VectorXd upper(5);
    upper.head<3>() = v_AC_upper;
    DRAKE_THROW_UNLESS(w_AC_bounds->magnitude_lower <=
                       w_AC_bounds->magnitude_upper);
    upper[3] = std::pow(w_AC_bounds->magnitude_upper, 2);
    upper[4] = 1;
    return upper;
  }
  return v_AC_upper;
}

template <typename T, typename U>
const MultibodyPlant<T>* MaybeMatchScalar(const MultibodyPlant<U>* instance) {
  if constexpr (std::is_same_v<T, U>) {
    return instance;
  } else {
    return nullptr;
  }
}

template <typename T, typename U>
systems::Context<T>* MaybeMatchScalar(systems::Context<U>* instance) {
  if constexpr (std::is_same_v<T, U>) {
    return instance;
  } else {
    return nullptr;
  }
}

// Sets y[3] = |w_AC|² and y[4] = cos(θ).
template <typename T>
void SetAngularVelocityRows(const Vector3<T>& w_AC,
                            const Eigen::Vector3d& reference_direction,
                            VectorX<T>* y) {
  using std::sqrt;
  const T squared_norm = w_AC.squaredNorm();
  (*y)[3] = squared_norm;
  const double kEps = std::numeric_limits<double>::epsilon();
  (*y)[4] =
      w_AC.dot(reference_direction) / sqrt(squared_norm + 100 * kEps * kEps);
}

// Evaluates the constraint with the plant's own scalar type.
template <typename T>
void EvalSameScalar(const MultibodyPlant<T>& plant,
                    systems::Context<T>* context, const Frame<T>& frameA,
                    const Frame<T>& frameB, const Eigen::Vector3d& p_BC,
                    const std::optional<Eigen::Vector3d>& reference_direction,
                    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) {
  UpdateContextPositionsAndVelocities(context, plant, x);

  // v_AC_A = v_AB_A + v_BC_A + w_AB_A x p_BC_A
  // Compute v_AB_A and w_AB_A.
  const SpatialVelocity<T> V_AB_A =
      frameB.CalcSpatialVelocity(*context, frameA, frameA);
  // v_BC_A = 0, because C is fixed in B.
  // p_BC_A = R_AB * p_BC.
  const math::RotationMatrix<T> R_AB =
      frameB.CalcRotationMatrix(*context, frameA);
  const Vector3<T> p_BC_A = R_AB * p_BC.cast<T>();
  y->template head<3>() =
      V_AB_A.translational() + V_AB_A.rotational().cross(p_BC_A);

  if (reference_direction) {
    // w_AC = w_AB_A + w_BC_A, but w_BC_A = 0 because C is fixed in B.
    SetAngularVelocityRows<T>(V_AB_A.rotational(), *reference_direction, y);
  }
}

// Evaluates the constraint with a double plant. V_AC_A = Jv_V_AC_A⋅v, so
// ∂V_AC_A/∂v = Jv_V_AC_A and
// ∂V_AC_A/∂q = [D₁(Jv_V_AC_A)⋅v, …, Dₙ(Jv_V_AC_A)⋅v]⋅N⁺(q).
void EvalClosedFormGradient(
    const MultibodyPlant<double>& plant, systems::Context<double>* context,
    const Frame<double>& frameA, const Frame<double>& frameB,
    const Eigen::Vector3d& p_BC,
    const std::optional<Eigen::Vector3d>& reference_direction,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  const VectorXd x_value = math::ExtractValue(x);
  UpdateContextPositionsAndVelocities(context, plant, x_value);
  const int nq = plant.num_positions();
  const int nv = plant.num_velocities();
  const auto v = x_value.tail(nv);

  Eigen::MatrixXd Jv_V_AC_A(6, nv);
  plant.CalcJacobianSpatialVelocity(*context, JacobianWrtVariable::kV, frameB,
                                    p_BC, frameA, frameA, &Jv_V_AC_A);
  std::vector<Matrix6X<double>> HJv_V_AC_A;
  plant.CalcHessianSpatialVelocity(*context, JacobianWrtVariable::kV, frameB,
                                   p_BC, frameA, frameA, &HJv_V_AC_A);

  Matrix6X<double> dV_dx(6, nq + nv);
  if (plant.IsVelocityEqualToQDot()) {
    for (int i = 0; i < nv; ++i) {
      dV_dx.col(i) = HJv_V_AC_A[i] * v;
    }
  } else {
    Matrix6X<double> HJv_V_AC_A_times_v(6, nv);
    for (int i = 0; i < nv; ++i) {
      HJv_V_AC_A_times_v.col(i) = HJv_V_AC_A[i] * v;
    }
    dV_dx.leftCols(nq) =
        HJv_V_AC_A_times_v * plant.MakeQDotToVelocityMap(*context);
  }
  dV_dx.rightCols(nv) = Jv_V_AC_A;

  const Vector6<AutoDiffXd> V_AC_A = math::InitializeAutoDiff(
      Vector6<double>(Jv_V_AC_A * v), dV_dx * math::ExtractGradient(x));
  y->head<3>() = V_AC_A.tail<3>();
  if (reference_direction) {
    SetAngularVelocityRows<AutoDiffXd>(V_AC_A.head<3>(), *reference_direction,
                                       y);
  }
}

}  // namespace

SpatialVelocityConstraint::~SpatialVelocityConstraint() = default;

SpatialVelocityConstraint::SpatialVelocityConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_lower,
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_upper,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BCo,
    systems::Context<AutoDiffXd>* plant_context,
    const std::optional<AngularVelocityBounds>& w_AC_bounds)
    : SpatialVelocityConstraint(plant, frameA, v_AC_lower, v_AC_upper, frameB,
                                p_BCo, plant_context, w_AC_bounds, 0) {}

SpatialVelocityConstraint::SpatialVelocityConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_lower,
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_upper,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BCo,
    systems::Context<double>* plant_context,
    const std::optional<AngularVelocityBounds>& w_AC_bounds)
    : SpatialVelocityConstraint(plant, frameA, v_AC_lower, v_AC_upper, frameB,
                                p_BCo, plant_context, w_AC_bounds, 0) {}

template <typename T>
SpatialVelocityConstraint::SpatialVelocityConstraint(
    const MultibodyPlant<T>* const plant, const Frame<T>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_lower,
    const Eigen::Ref<const Eigen::Vector3d>& v_AC_upper, const Frame<T>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BCo,
    systems::Context<T>* plant_context,
    const std::optional<AngularVelocityBounds>& w_AC_bounds, int)
    : solvers::Constraint(w_AC_bounds ? 5 : 3,
                          RefFromPtrOrThrow(plant).num_multibody_states(),
                          MakeLowerBounds(v_AC_lower, w_AC_bounds),
                          MakeUpperBounds(v_AC_upper, w_AC_bounds)),
      plant_double_(MaybeMatchScalar<double>(plant)),
      context_double_(MaybeMatchScalar<double>(plant_context)),
      plant_autodiff_(MaybeMatchScalar<AutoDiffXd>(plant)),
      context_autodiff_(MaybeMatchScalar<AutoDiffXd>(plant_context)),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      p_BC_{p_BCo} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (w_AC_bounds) {
    w_AC_reference_direction_ = w_AC_bounds->reference_direction.normalized();
  }
}

void SpatialVelocityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  if (plant_double_ != nullptr) {
    y->resize(this->num_constraints());
    EvalSameScalar<double>(*plant_double_, context_double_,
                           plant_double_->get_frame(frameA_index_),
                           plant_double_->get_frame(frameB_index_), p_BC_,
                           w_AC_reference_direction_, x, y);
  } else {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  }
}

void SpatialVelocityConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  y->resize(this->num_constraints());
  if (plant_double_ != nullptr) {
    EvalClosedFormGradient(*plant_double_, context_double_,
                           plant_double_->get_frame(frameA_index_),
                           plant_double_->get_frame(frameB_index_), p_BC_,
                           w_AC_reference_direction_, x, y);
  } else {
    EvalSameScalar<AutoDiffXd>(*plant_autodiff_, context_autodiff_,
                               plant_autodiff_->get_frame(frameA_index_),
                               plant_autodiff_->get_frame(frameB_index_), p_BC_,
                               w_AC_reference_direction_, x, y);
  }
}

}  // namespace multibody
}  // namespace drake
