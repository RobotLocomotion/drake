#include "drake/multibody/constraint_wrappers.h"

namespace drake {
namespace systems {
namespace plants {

SingleTimeKinematicConstraintWrapper::SingleTimeKinematicConstraintWrapper(
    const SingleTimeKinematicConstraint* rigid_body_constraint,
    KinematicsCacheHelper<double>* kin_helper)
    : Constraint(rigid_body_constraint->getNumConstraint(nullptr),
                 rigid_body_constraint->getRobotPointer()->get_num_positions()),
      rigid_body_constraint_(rigid_body_constraint),
      kin_helper_(kin_helper) {
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  rigid_body_constraint->bounds(nullptr, lower_bound, upper_bound);
  set_bounds(lower_bound, upper_bound);
}

SingleTimeKinematicConstraintWrapper::~SingleTimeKinematicConstraintWrapper() {}

void SingleTimeKinematicConstraintWrapper::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::VectorXd* y) const {
  auto& kinsol = kin_helper_->UpdateKinematics(
      q, rigid_body_constraint_->getRobotPointer());
  Eigen::MatrixXd dy;
  rigid_body_constraint_->eval(nullptr, kinsol, *y, dy);
}

void SingleTimeKinematicConstraintWrapper::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& tq, AutoDiffVecXd* ty) const {
  Eigen::VectorXd q = drake::math::autoDiffToValueMatrix(tq);
  auto& kinsol = kin_helper_->UpdateKinematics(
      q, rigid_body_constraint_->getRobotPointer());
  Eigen::VectorXd y;
  Eigen::MatrixXd dy;
  rigid_body_constraint_->eval(nullptr, kinsol, y, dy);
  math::initializeAutoDiffGivenGradientMatrix(
      y, (dy * drake::math::autoDiffToGradientMatrix(tq)).eval(), *ty);
}

void SingleTimeKinematicConstraintWrapper::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "SingleTimeKinematicConstraintWrapper does not support symbolic "
      "evaluation.");
}

void QuasiStaticConstraintWrapper::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::VectorXd* y) const {
  auto& kinsol = kin_helper_->UpdateKinematics(
      q.head(rigid_body_constraint_->getRobotPointer()->get_num_positions()),
      rigid_body_constraint_->getRobotPointer());
  auto weights = q.tail(rigid_body_constraint_->getNumWeights());
  Eigen::MatrixXd dy;
  rigid_body_constraint_->eval(nullptr, kinsol, weights.data(), *y, dy);
}

QuasiStaticConstraintWrapper::QuasiStaticConstraintWrapper(
    const QuasiStaticConstraint* rigid_body_constraint,
    KinematicsCacheHelper<double>* kin_helper)
    : Constraint(rigid_body_constraint->getNumConstraint(nullptr) - 1,
                 rigid_body_constraint->getRobotPointer()->get_num_positions() +
                     rigid_body_constraint->getNumWeights()),
      rigid_body_constraint_(rigid_body_constraint),
      kin_helper_(kin_helper) {
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  rigid_body_constraint->bounds(nullptr, lower_bound, upper_bound);
  set_bounds(lower_bound, upper_bound);
}

void QuasiStaticConstraintWrapper::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "SingleTimeKinematicConstraintWrapper does not support symbolic "
      "evaluation.");
}

QuasiStaticConstraintWrapper::~QuasiStaticConstraintWrapper() {}

void QuasiStaticConstraintWrapper::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& tq, AutoDiffVecXd* ty) const {
  Eigen::VectorXd q = drake::math::autoDiffToValueMatrix(tq);
  auto& kinsol = kin_helper_->UpdateKinematics(
      q.head(rigid_body_constraint_->getRobotPointer()->get_num_positions()),
      rigid_body_constraint_->getRobotPointer());

  Eigen::VectorXd y;
  Eigen::MatrixXd dy;
  auto weights = q.tail(rigid_body_constraint_->getNumWeights());
  rigid_body_constraint_->eval(nullptr, kinsol, weights.data(), y, dy);
  y.conservativeResize(num_constraints());
  drake::math::initializeAutoDiffGivenGradientMatrix(
      y, (dy * drake::math::autoDiffToGradientMatrix(tq)).eval(), *ty);
}


}  // namespace plants
}  // namespace systems
}  // namespace drake
