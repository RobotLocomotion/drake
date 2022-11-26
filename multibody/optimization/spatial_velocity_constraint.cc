#include "drake/multibody/optimization/spatial_velocity_constraint.h"

#include <utility>

#include "drake/math/differentiable_norm.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

namespace drake {
namespace multibody {

using internal::RefFromPtrOrThrow;
using internal::UpdateContextPositionsAndVelocities;
using Eigen::VectorXd;

namespace {

VectorXd MakeLowerBounds(
    const Eigen::Ref<const Eigen::Vector3d>& v_AQ_lower,
    const std::optional<SpatialVelocityConstraint::AngularVelocityBounds>&
        w_AQ_bounds) {
  if (w_AQ_bounds) {
    VectorXd lower(5);
    lower.head<3>() = v_AQ_lower;
    DRAKE_THROW_UNLESS(w_AQ_bounds->magnitude_lower >= 0);
    lower[3] = w_AQ_bounds->magnitude_lower;
    lower[4] = 0;  // the angle difference will always be positive.
    return lower;
  }
  return v_AQ_lower;
}

VectorXd MakeUpperBounds(
    const Eigen::Ref<const Eigen::Vector3d>& v_AQ_upper,
    const std::optional<SpatialVelocityConstraint::AngularVelocityBounds>&
        w_AQ_bounds) {
  if (w_AQ_bounds) {
    VectorXd upper(5);
    upper.head<3>() = v_AQ_upper;
    upper[3] = w_AQ_bounds->magnitude_upper;
    upper[4] = w_AQ_bounds->theta_bound;
    return upper;
  }
  return v_AQ_upper;
}

}  // namespace

SpatialVelocityConstraint::SpatialVelocityConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& v_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& v_AQ_upper,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    systems::Context<AutoDiffXd>* plant_context,
    const std::optional<AngularVelocityBounds>& w_AQ_bounds)
    : solvers::Constraint(w_AQ_bounds ? 5 : 3,
                          RefFromPtrOrThrow(plant).num_multibody_states(),
                          MakeLowerBounds(v_AQ_lower, w_AQ_bounds),
                          MakeUpperBounds(v_AQ_upper, w_AQ_bounds)),
      plant_(plant),
      context_(plant_context),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      p_BQ_{p_BQ} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  if (w_AQ_bounds) {
    w_AQ_nominal_direction_ =
        w_AQ_bounds->nominal_direction.normalized();
  }
}

void SpatialVelocityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(x.cast<AutoDiffXd>(), &y_t);
  *y = math::ExtractValue(y_t);
}

void SpatialVelocityConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(this->num_constraints());
  UpdateContextPositionsAndVelocities(context_, *plant_, x);
  const Frame<AutoDiffXd>& frameA = plant_->get_frame(frameA_index_);
  const Frame<AutoDiffXd>& frameB = plant_->get_frame(frameB_index_);

  // v_AQ_A = v_AB_A + v_BQ_A + w_AB_A x p_BQ_A
  // Compute v_AB_A and w_AB_A.
  const SpatialVelocity<AutoDiffXd> V_AB_A =
      frameB.CalcSpatialVelocity(*context_, frameA, frameA);
  // v_BQ_A = 0, because Q is fixed in B.
  // p_BQ_A = R_AB * p_BQ.
  const math::RotationMatrix<AutoDiffXd> R_AB =
      frameB.CalcRotationMatrix(*context_, frameA);
  const Vector3<AutoDiffXd> p_BQ_A = R_AB * p_BQ_.cast<AutoDiffXd>();
  y->head<3>() = V_AB_A.translational() + V_AB_A.rotational().cross(p_BQ_A);

  log()->info("V_AB_A = {}", V_AB_A);
  log()->info("V_AB_W = {}", frameB.CalcSpatialVelocity(*context_, frameA,
                                                        plant_->world_frame()));
  log()->info("V_AQ_A = {}", V_AB_A.Shift(p_BQ_A));
  log()->info("p_BQ_A = {}", math::ExtractValue(p_BQ_A).transpose());
  log()->info("v_AQ_A = {}", math::ExtractValue(y->head<3>()).transpose());

  if (w_AQ_nominal_direction_) {
    DRAKE_ASSERT(this->num_constraints() == 5);
    // w_AQ = w_AB_A + w_BQ_A, but w_BQ_A = 0 because Q is fixed in B.
    Vector3<AutoDiffXd> w_AQ = V_AB_A.rotational();
    AutoDiffXd norm = math::DifferentiableNorm(w_AQ);
    (*y)[3] = norm;
    AutoDiffXd cos_theta = w_AQ.dot(*w_AQ_nominal_direction_) / norm;
    using std::acos;
    (*y)[4] = acos(cos_theta);
  }
}

}  // namespace multibody
}  // namespace drake
