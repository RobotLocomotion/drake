#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace systems {
namespace plants {

/// Helper class to avoid recalculating a kinematics cache which is
/// going to be used repeatedly by multiple other classes.
template <typename Scalar>
class KinematicsCacheHelper {
 public:
  explicit KinematicsCacheHelper(
      const std::vector<std::unique_ptr<RigidBody<Scalar>>>& bodies);

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const RigidBodyTree<double>* tree);

 private:
  Eigen::VectorXd last_q_;
  const RigidBodyTree<double>* last_tree_;
  KinematicsCache<Scalar> kinsol_;
};

class SingleTimeKinematicConstraintWrapper : public drake::solvers::Constraint {
 public:
  /// All pointers are aliased for the lifetime of the wrapper.
  SingleTimeKinematicConstraintWrapper(
      const SingleTimeKinematicConstraint* rigid_body_constraint,
      KinematicsCacheHelper<double>* kin_helper);

  ~SingleTimeKinematicConstraintWrapper() override;

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &q,
              Eigen::VectorXd &y) const override;

  void DoEval(const Eigen::Ref<const TaylorVecXd> &tq,
              TaylorVecXd &ty) const override;

 private:
  const SingleTimeKinematicConstraint* rigid_body_constraint_;
  mutable KinematicsCacheHelper<double>* kin_helper_;
};

class QuasiStaticConstraintWrapper : public drake::solvers::Constraint {
 public:
  /// All pointers are aliased for the lifetime of the wrapper.  Also,
  /// the wrapped QuasiStaticConstraint claims to have three
  /// constraints, but the third was handled differently in the
  /// original SNOPT implementation, which we won't try to reproduce
  /// here.
  QuasiStaticConstraintWrapper(
      const QuasiStaticConstraint* rigid_body_constraint,
      KinematicsCacheHelper<double>* kin_helper);

  virtual ~QuasiStaticConstraintWrapper();

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &q,
              Eigen::VectorXd &y) const override;

  void DoEval(const Eigen::Ref<const TaylorVecXd> &tq,
              TaylorVecXd &ty) const override;

 private:
  const QuasiStaticConstraint* rigid_body_constraint_;
  mutable KinematicsCacheHelper<double>* kin_helper_;
};

}  // namespace plants
}  // namespace systems
}  // namespace drake
