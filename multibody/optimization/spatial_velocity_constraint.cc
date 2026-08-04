#include "drake/multibody/optimization/spatial_velocity_constraint.h"

#include <utility>

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
    : solvers::Constraint(w_AC_bounds ? 5 : 3,
                          RefFromPtrOrThrow(plant).num_multibody_states(),
                          MakeLowerBounds(v_AC_lower, w_AC_bounds),
                          MakeUpperBounds(v_AC_upper, w_AC_bounds)),
      plant_(plant),
      context_(plant_context),
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
  AutoDiffVecXd y_t;
  Eval(x.cast<AutoDiffXd>(), &y_t);
  *y = math::ExtractValue(y_t);
}

void SpatialVelocityConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  y->resize(this->num_constraints());
  UpdateContextPositionsAndVelocities(context_, *plant_, x);
  const Frame<AutoDiffXd>& frameA = plant_->get_frame(frameA_index_);
  const Frame<AutoDiffXd>& frameB = plant_->get_frame(frameB_index_);

  // v_AC_A = v_AB_A + v_BC_A + w_AB_A x p_BC_A
  // Compute v_AB_A and w_AB_A.
  const SpatialVelocity<AutoDiffXd> V_AB_A =
      frameB.CalcSpatialVelocity(*context_, frameA, frameA);
  // v_BC_A = 0, because C is fixed in B.
  // p_BC_A = R_AB * p_BC.
  const math::RotationMatrix<AutoDiffXd> R_AB =
      frameB.CalcRotationMatrix(*context_, frameA);
  const Vector3<AutoDiffXd> p_BC_A = R_AB * p_BC_.cast<AutoDiffXd>();
  y->head<3>() = V_AB_A.translational() + V_AB_A.rotational().cross(p_BC_A);

  if (w_AC_reference_direction_) {
    DRAKE_ASSERT(this->num_constraints() == 5);
    // w_AC = w_AB_A + w_BC_A, but w_BC_A = 0 because C is fixed in B.
    Vector3<AutoDiffXd> w_AC = V_AB_A.rotational();
    AutoDiffXd squared_norm = w_AC.squaredNorm();
    (*y)[3] = squared_norm;
    const double kEps = std::numeric_limits<double>::epsilon();
    // cos(θ)
    (*y)[4] = w_AC.dot(*w_AC_reference_direction_) /
              sqrt(squared_norm + 100 * kEps * kEps);
  }
}

}  // namespace multibody
}  // namespace drake
