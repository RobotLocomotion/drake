#ifndef DRAKE_SYSTEMS_PLANTS_CONSTRAINTWRAPPERS_H_
#define DRAKE_SYSTEMS_PLANTS_CONSTRAINTWRAPPERS_H_

#include <memory>

#include <Eigen/Core>

#include "drake/core/Gradient.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/KinematicsCache.h"

namespace Drake {
namespace systems {
namespace plants {

class SingleTimeKinematicConstraintWrapper : public Constraint {
 public:
  SingleTimeKinematicConstraintWrapper(
      const std::shared_ptr<SingleTimeKinematicConstraint>& rigid_body_constraint)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr)),
        rigid_body_constraint(rigid_body_constraint),
        kinsol(rigid_body_constraint->getRobotPointer()->bodies) {
    rigid_body_constraint->bounds(nullptr, lower_bound_, upper_bound_);
  }
  ~SingleTimeKinematicConstraintWrapper() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& q,
                    Eigen::VectorXd& y) const override {
    kinsol.initialize(q);
    rigid_body_constraint->getRobotPointer()->doKinematics(kinsol);
    Eigen::MatrixXd dy;
    rigid_body_constraint->eval(nullptr, kinsol, y, dy);
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& tq,
                    TaylorVecXd& ty) const override {
    kinsol.initialize(autoDiffToValueMatrix(tq));
    rigid_body_constraint->getRobotPointer()->doKinematics(kinsol);
    Eigen::VectorXd y;
    Eigen::MatrixXd dy;
    rigid_body_constraint->eval(nullptr, kinsol, y, dy);
    initializeAutoDiffGivenGradientMatrix(
        y, (dy * autoDiffToGradientMatrix(tq)).eval(), ty);
  }

 private:
  std::shared_ptr<SingleTimeKinematicConstraint> rigid_body_constraint;
  mutable KinematicsCache<double> kinsol;
};

}
}
}

#endif // DRAKE_SYSTEMS_PLANTS_CONSTRAINT_CONSTRAINTWRAPPERS_H_
