#include "drake/multibody/constraint_wrappers.h"

namespace drake {
namespace systems {
namespace plants {

template <typename T>
KinematicsCacheHelper<T>::KinematicsCacheHelper(
    const std::vector<std::unique_ptr<RigidBody<T>>>& bodies) :
    kinsol_(RigidBodyTree<T>::CreateKinematicsCacheFromBodiesVector(bodies)) {
}

template <typename Scalar>
KinematicsCache<Scalar>& KinematicsCacheHelper<Scalar>::UpdateKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const RigidBodyTree<double>* tree) {
  if ((q.size() != last_q_.size()) || (q != last_q_) ||
      (tree != last_tree_)) {
    last_q_ = q;
    last_tree_ = tree;
    kinsol_.initialize(q);
    tree->doKinematics(kinsol_);
  }
  return kinsol_;
}

SingleTimeKinematicConstraintWrapper::SingleTimeKinematicConstraintWrapper(
    const SingleTimeKinematicConstraint* rigid_body_constraint,
    KinematicsCacheHelper<double>* kin_helper)
    : Constraint(rigid_body_constraint->getNumConstraint(nullptr)),
      rigid_body_constraint_(rigid_body_constraint),
      kin_helper_(kin_helper) {
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  rigid_body_constraint->bounds(nullptr, lower_bound, upper_bound);
  set_bounds(lower_bound, upper_bound);
}

SingleTimeKinematicConstraintWrapper::~SingleTimeKinematicConstraintWrapper() {}

void SingleTimeKinematicConstraintWrapper::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::VectorXd& y) const {
  auto& kinsol = kin_helper_->UpdateKinematics(
      q, rigid_body_constraint_->getRobotPointer());
  Eigen::MatrixXd dy;
  rigid_body_constraint_->eval(nullptr, kinsol, y, dy);
}

void SingleTimeKinematicConstraintWrapper::Eval(
    const Eigen::Ref<const TaylorVecXd>& tq, TaylorVecXd& ty) const {
  Eigen::VectorXd q = drake::math::autoDiffToValueMatrix(tq);
  auto& kinsol = kin_helper_->UpdateKinematics(
      q, rigid_body_constraint_->getRobotPointer());
  Eigen::VectorXd y;
  Eigen::MatrixXd dy;
  rigid_body_constraint_->eval(nullptr, kinsol, y, dy);
  math::initializeAutoDiffGivenGradientMatrix(
      y, (dy * drake::math::autoDiffToGradientMatrix(tq)).eval(), ty);
}

void QuasiStaticConstraintWrapper::Eval(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    Eigen::VectorXd& y) const {
  auto& kinsol = kin_helper_->UpdateKinematics(
      q.head(
          rigid_body_constraint_->getRobotPointer()->get_num_positions()),
      rigid_body_constraint_->getRobotPointer());
  auto weights = q.tail(rigid_body_constraint_->getNumWeights());
  Eigen::MatrixXd dy;
  rigid_body_constraint_->eval(nullptr, kinsol, weights.data(), y, dy);
}

QuasiStaticConstraintWrapper::QuasiStaticConstraintWrapper(
    const QuasiStaticConstraint* rigid_body_constraint,
    KinematicsCacheHelper<double>* kin_helper)
    : Constraint(rigid_body_constraint->getNumConstraint(nullptr) - 1),
      rigid_body_constraint_(rigid_body_constraint),
      kin_helper_(kin_helper) {
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  rigid_body_constraint->bounds(nullptr, lower_bound, upper_bound);
  set_bounds(lower_bound, upper_bound);
}

QuasiStaticConstraintWrapper::~QuasiStaticConstraintWrapper() {}

void QuasiStaticConstraintWrapper::Eval(
    const Eigen::Ref<const TaylorVecXd>& tq, TaylorVecXd& ty) const {
  Eigen::VectorXd q = drake::math::autoDiffToValueMatrix(tq);
  auto& kinsol = kin_helper_->UpdateKinematics(
      q.head(
          rigid_body_constraint_->getRobotPointer()->get_num_positions()),
      rigid_body_constraint_->getRobotPointer());

  Eigen::VectorXd y;
  Eigen::MatrixXd dy;
  auto weights = q.tail(rigid_body_constraint_->getNumWeights());
  rigid_body_constraint_->eval(nullptr, kinsol, weights.data(), y, dy);
  y.conservativeResize(num_constraints());
  drake::math::initializeAutoDiffGivenGradientMatrix(
      y, (dy * drake::math::autoDiffToGradientMatrix(tq)).eval(), ty);
}

// Explicitly instantiates on the most common scalar types.
template class KinematicsCacheHelper<double>;

}  // namespace plants
}  // namespace systems
}  // namespace drake
