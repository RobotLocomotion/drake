#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {
namespace plants {

/// Helper class to avoid recalculating a kinematics cache which is
/// going to be used repeatedly by multiple other classes.
/// TODO(amcastro-tri): DRAKE_EXPORT is used here in the header instead of at
/// the place of explicit instantiation. Explore a solution for this problem as
/// described in #3940.
template <typename Scalar>
class DRAKE_EXPORT KinematicsCacheHelper {
 public:
  explicit KinematicsCacheHelper(
      const std::vector<std::unique_ptr<RigidBody>>& bodies);

  KinematicsCache<Scalar>& UpdateKinematics(
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const RigidBodyTree<double>* tree);

 private:
  Eigen::VectorXd last_q_;
  const RigidBodyTree<double>* last_tree_;
  KinematicsCache<Scalar> kinsol_;
};

class DRAKE_EXPORT SingleTimeKinematicConstraintWrapper :
      public drake::solvers::Constraint {
 public:
  /// All pointers are aliased for the lifetime of the wrapper.
  SingleTimeKinematicConstraintWrapper(
      const SingleTimeKinematicConstraint* rigid_body_constraint,
      KinematicsCacheHelper<double>* kin_helper);

  ~SingleTimeKinematicConstraintWrapper() override;

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& q,
            Eigen::VectorXd& y) const override;

  void Eval(const Eigen::Ref<const TaylorVecXd>& tq,
            TaylorVecXd& ty) const override;

 private:
  const SingleTimeKinematicConstraint* rigid_body_constraint_;
  mutable KinematicsCacheHelper<double>* kin_helper_;
};

class DRAKE_EXPORT QuasiStaticConstraintWrapper :
      public drake::solvers::Constraint {
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

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& q,
            Eigen::VectorXd& y) const override;

  void Eval(const Eigen::Ref<const TaylorVecXd>& tq,
            TaylorVecXd& ty) const override;

 private:
  const QuasiStaticConstraint* rigid_body_constraint_;
  mutable KinematicsCacheHelper<double>* kin_helper_;
};

}  // namespace plants
}  // namespace systems
}  // namespace drake
