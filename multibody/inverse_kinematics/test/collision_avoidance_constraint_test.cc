#include "drake/multibody/inverse_kinematics/collision_avoidance_constraint.h"

#include <limits>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
TEST_F(TwoFreeSpheresConstraintTest, CollisionAvoidanceConstraint) {
  const double minimal_distance(0.1);
  const CollisionAvoidanceConstraint constraint(
      *two_spheres_plant_, minimal_distance, plant_context_autodiff_);
  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Vector1d(minimal_distance)));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(),
                      Vector1d(std::numeric_limits<double>::infinity())));

  const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
  const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
  const Eigen::Vector3d sphere1_position(0.2, 1.2, 0.3);
  const Eigen::Vector3d sphere2_position(1.2, -0.4, 2.3);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(sphere1_quaternion), sphere1_position,
      QuaternionToVectorWxyz(sphere2_quaternion), sphere2_position;
  Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff = math::initializeAutoDiff(q);
  dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(plant_context_autodiff_)
      ->get_mutable_positions() = q_autodiff;
  AutoDiffVecXd y(1);
  constraint.Eval(q_autodiff, &y);

  // TODO(hongkai.dai): change this check once we add the penalty term to
  // evaluate CollisionAvoidanceConstraint.
  const double tol{1E-6};
  EXPECT_NEAR(
      y(0).value(),
      (sphere1_position - sphere2_position).norm() - radius1_ - radius2_, tol);
}
}
}  // namespace multibody
}  // namespace drake
