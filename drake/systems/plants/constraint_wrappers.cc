#include "ConstraintWrappers.h"

namespace drake {
namespace systems {
namespace plants {

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

}
}
}
