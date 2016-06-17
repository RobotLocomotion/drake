#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/core/Gradient.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace Drake {
namespace systems {
namespace plants {

/// Helper class to avoid recalculating a kinematics cache which is
/// going to be used repeatedly by multiple other classes.
template <typename Scalar>
class KinematicsCacheHelper {
 public:
  KinematicsCacheHelper(
      const std::vector<std::unique_ptr<RigidBody> >& bodies)
      : kinsol_(bodies) {}

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const RigidBodyTree* tree) {
    if ((q.size() != last_q_.size()) || (q != last_q_) ||
        (tree != last_tree_)) {
      last_q_ = q;
      last_tree_ = tree;
      kinsol_.initialize(q);
      tree->doKinematics(kinsol_);
    }
    return kinsol_;
  }

 private:
  Eigen::VectorXd last_q_;
  const RigidBodyTree* last_tree_;
  KinematicsCache<Scalar> kinsol_;
};

class SingleTimeKinematicConstraintWrapper :
      public drake::solvers::Constraint {
 public:
  /// All pointers are aliased for the lifetime of the wrapper.
  SingleTimeKinematicConstraintWrapper(
      const SingleTimeKinematicConstraint* rigid_body_constraint,
      KinematicsCacheHelper<double>* kin_helper)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr)),
        rigid_body_constraint_(rigid_body_constraint),
        kin_helper_(kin_helper) {
    rigid_body_constraint->bounds(nullptr, lower_bound_, upper_bound_);
  }
  ~SingleTimeKinematicConstraintWrapper() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& q,
                    Eigen::VectorXd& y) const override {
    auto& kinsol = kin_helper_->UpdateKinematics(
        q, rigid_body_constraint_->getRobotPointer());
    Eigen::MatrixXd dy;
    rigid_body_constraint_->eval(nullptr, kinsol, y, dy);
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& tq,
                    TaylorVecXd& ty) const override {
    Eigen::VectorXd q = autoDiffToValueMatrix(tq);
    auto& kinsol = kin_helper_->UpdateKinematics(
        q, rigid_body_constraint_->getRobotPointer());
    Eigen::VectorXd y;
    Eigen::MatrixXd dy;
    rigid_body_constraint_->eval(nullptr, kinsol, y, dy);
    initializeAutoDiffGivenGradientMatrix(
        y, (dy * autoDiffToGradientMatrix(tq)).eval(), ty);
  }

 private:
  const SingleTimeKinematicConstraint* rigid_body_constraint_;
  mutable KinematicsCacheHelper<double>* kin_helper_;
};

class QuasiStaticConstraintWrapper :
      public drake::solvers::Constraint {
 public:
  /// All pointers are aliased for the lifetime of the wrapper.  Also,
  /// the wrapped QuasiStaticConstraint claims to have three
  /// constraints, but the third was handled differently in the
  /// original SNOPT implementation, which we won't try to reproduce
  /// here.
  QuasiStaticConstraintWrapper(
      const QuasiStaticConstraint* rigid_body_constraint,
      KinematicsCacheHelper<double>* kin_helper)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr) - 1),
        rigid_body_constraint_(rigid_body_constraint),
        kin_helper_(kin_helper) {
    rigid_body_constraint->bounds(nullptr, lower_bound_, upper_bound_);
  }
  virtual ~QuasiStaticConstraintWrapper() {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& q,
            Eigen::VectorXd& y) const override {
    auto& kinsol = kin_helper_->UpdateKinematics(
        q.head(
            rigid_body_constraint_->getRobotPointer()->number_of_positions()),
            rigid_body_constraint_->getRobotPointer());
    auto weights = q.tail(rigid_body_constraint_->getNumWeights());
    Eigen::MatrixXd dy;
    rigid_body_constraint_->eval(nullptr, kinsol, weights.data(), y, dy);
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& tq,
            TaylorVecXd& ty) const override {
    Eigen::VectorXd q = autoDiffToValueMatrix(tq);
    auto& kinsol = kin_helper_->UpdateKinematics(
        q.head(
            rigid_body_constraint_->getRobotPointer()->number_of_positions()),
        rigid_body_constraint_->getRobotPointer());

    Eigen::VectorXd y;
    Eigen::MatrixXd dy;
    auto weights = q.tail(rigid_body_constraint_->getNumWeights());
    rigid_body_constraint_->eval(nullptr, kinsol, weights.data(), y, dy);
    y.conservativeResize(num_constraints());
    initializeAutoDiffGivenGradientMatrix(
        y, (dy * autoDiffToGradientMatrix(tq)).eval(), ty);
  }

 private:
  const QuasiStaticConstraint* rigid_body_constraint_;
  mutable KinematicsCacheHelper<double>* kin_helper_;
};

}
}
}
