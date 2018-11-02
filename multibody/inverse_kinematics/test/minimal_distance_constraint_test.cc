#include "drake/multibody/inverse_kinematics/minimal_distance_constraint.h"

#include <limits>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
TEST_F(TwoFreeSpheresTest, MinimalDistanceConstraint) {
  const double minimal_distance(0.1);
  const MinimalDistanceConstraint constraint(*plant_autodiff_, minimal_distance,
                                             plant_context_autodiff_);
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
  Eigen::VectorXd y_double(1);
  constraint.Eval(q, &y_double);
  Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y(1);
  constraint.Eval(q_autodiff, &y);

  // TODO(hongkai.dai): change this check once we add the penalty term to
  // evaluate MinimalDistanceConstraint.
  const double tol{1E-6};
  const double distance_expected =
      (sphere1_position - sphere2_position).norm() - radius1_ - radius2_;
  EXPECT_NEAR(y_double(0), distance_expected, tol);
  EXPECT_NEAR(y(0).value(), distance_expected, tol);
}

GTEST_TEST(MinimalDistanceConstraintTest,
           MultibodyPlantWithouthGeometrySource) {
  auto plant = ConstructTwoFreeBodiesPlant<AutoDiffXd>();
  auto context = plant->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimalDistanceConstraint(*plant, 0.1, context.get()),
      std::invalid_argument,
      "MinimalDistanceConstraint: MultibodyPlant has not registered its "
      "geometry source with SceneGraph yet.");
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
