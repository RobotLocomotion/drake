#ifndef DRAKE_SYSTEMS_PLANTS_CONSTRAINTWRAPPERS_H_
#define DRAKE_SYSTEMS_PLANTS_CONSTRAINTWRAPPERS_H_

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

class SingleTimeKinematicConstraintWrapper : public Constraint {
 public:
  SingleTimeKinematicConstraintWrapper(
      const std::shared_ptr<SingleTimeKinematicConstraint>& rigid_body_constraint)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr)),
        rigid_body_constraint_shared_(rigid_body_constraint),
        rigid_body_constraint_(rigid_body_constraint.get()),
        kinsol_(rigid_body_constraint->getRobotPointer()->bodies) {
    rigid_body_constraint->bounds(nullptr, lower_bound_, upper_bound_);
  }

  // Don't take partial ownership of the constraint, instead alias it
  // for the lifetime of the wrapper.
  SingleTimeKinematicConstraintWrapper(
      const SingleTimeKinematicConstraint* rigid_body_constraint)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr)),
        rigid_body_constraint_(rigid_body_constraint),
        kinsol_(rigid_body_constraint->getRobotPointer()->bodies) {
    rigid_body_constraint->bounds(nullptr, lower_bound_, upper_bound_);
  }
  ~SingleTimeKinematicConstraintWrapper() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& q,
                    Eigen::VectorXd& y) const override {
    kinsol_.initialize(q);
    rigid_body_constraint_->getRobotPointer()->doKinematics(kinsol_);
    Eigen::MatrixXd dy;
    rigid_body_constraint_->eval(nullptr, kinsol_, y, dy);
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& tq,
                    TaylorVecXd& ty) const override {
    Eigen::VectorXd q = autoDiffToValueMatrix(tq);
    kinsol_.initialize(q);
    rigid_body_constraint_->getRobotPointer()->doKinematics(kinsol_);
    Eigen::VectorXd y;
    Eigen::MatrixXd dy;
    rigid_body_constraint_->eval(nullptr, kinsol_, y, dy);
    initializeAutoDiffGivenGradientMatrix(
        y, (dy * autoDiffToGradientMatrix(tq)).eval(), ty);
  }

 private:
  std::shared_ptr<SingleTimeKinematicConstraint> rigid_body_constraint_shared_;
  const SingleTimeKinematicConstraint* rigid_body_constraint_;
  mutable KinematicsCache<double> kinsol_;
};

class QuasiStaticConstraintWrapper : public Constraint {
 public:
  // Don't take partial ownership of the constraint, instead alias it
  // for the lifetime of the wrapper.  Also, the wrapped
  // QuasiStaticConstraint claims to have three constraints, but the
  // third was handled differently in the original SNOPT
  // implementation, which we won't try to reproduce here.
  QuasiStaticConstraintWrapper(
      const QuasiStaticConstraint* rigid_body_constraint)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr) - 1),
        rigid_body_constraint_(rigid_body_constraint),
        kinsol_(rigid_body_constraint->getRobotPointer()->bodies) {
    rigid_body_constraint->bounds(nullptr, lower_bound_, upper_bound_);
  }
  virtual ~QuasiStaticConstraintWrapper() {}

  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& q,
                    Eigen::VectorXd& y) const override {
    kinsol_.initialize(q.head(
        rigid_body_constraint_->getRobotPointer()->num_positions));
    rigid_body_constraint_->getRobotPointer()->doKinematics(kinsol_);
    auto weights = q.tail(rigid_body_constraint_->getNumWeights());
    Eigen::MatrixXd dy;
    rigid_body_constraint_->eval(nullptr, kinsol_, weights.data(), y, dy);
  }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& tq,
                    TaylorVecXd& ty) const override {
    Eigen::VectorXd q = autoDiffToValueMatrix(tq);
    kinsol_.initialize(q.head(
        rigid_body_constraint_->getRobotPointer()->num_positions));
    rigid_body_constraint_->getRobotPointer()->doKinematics(kinsol_);

    Eigen::VectorXd y;
    Eigen::MatrixXd dy;
    auto weights = q.tail(rigid_body_constraint_->getNumWeights());
    rigid_body_constraint_->eval(nullptr, kinsol_, weights.data(), y, dy);
    y.conservativeResize(num_constraints());
    initializeAutoDiffGivenGradientMatrix(
        y, (dy * autoDiffToGradientMatrix(tq)).eval(), ty);
  }

 private:
  const QuasiStaticConstraint* rigid_body_constraint_;
  // TODO(sam.creasey) share the kinematics cache between wrappers
  mutable KinematicsCache<double> kinsol_;
};

}
}
}

#endif // DRAKE_SYSTEMS_PLANTS_CONSTRAINT_CONSTRAINTWRAPPERS_H_
