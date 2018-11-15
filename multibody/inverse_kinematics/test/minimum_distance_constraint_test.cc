#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
// The penalty function explained in section II.C of Whole-body Motion planning
// with Centroidal dynamics and full kinematics by Hongkai Dai, Andres
// Valenzuela and Russ Tedrake.
template <typename T>
T Penalty(const T& distance, double minimum_distance) {
  if (distance >= minimum_distance) {
    return 0;
  } else {
    const T x = distance / minimum_distance - 1;
    using std::exp;
    const T penalty = -x * exp(1.0 / x);
    return penalty;
  }
}

TEST_F(TwoFreeSpheresTest, MinimumDistanceConstraint) {
  const double minimum_distance(0.1);
  const MinimumDistanceConstraint constraint(*plant_autodiff_, minimum_distance,
                                             plant_context_autodiff_);
  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(0)));

  const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
  const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
  // distance larger than minimum_distance;
  Eigen::Vector3d sphere1_position(0.2, 1.2, 0.3);
  Eigen::Vector3d sphere2_position(1.2, -0.4, 2.3);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(sphere1_quaternion), sphere1_position,
      QuaternionToVectorWxyz(sphere2_quaternion), sphere2_position;
  Eigen::VectorXd y_double(1);
  constraint.Eval(q, &y_double);
  Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y(1);
  constraint.Eval(q_autodiff, &y);

  EXPECT_EQ(y_double(0), 0);
  EXPECT_EQ(y(0).value(), 0);
  EXPECT_TRUE(CompareMatrices(y(0).derivatives(),
                              Eigen::Matrix<double, 14, 1>::Zero()));

  // distance smaller than the minimum_distance;
  sphere1_position << 0.1, 0.2, 0.3;
  sphere2_position << 0.11, 0.21, 0.31;
  q << QuaternionToVectorWxyz(sphere1_quaternion), sphere1_position,
      QuaternionToVectorWxyz(sphere2_quaternion), sphere2_position;
  const double distance_expected =
      (sphere1_position - sphere2_position).norm() - radius1_ - radius2_;
  constraint.Eval(q, &y_double);
  const double penalty_expected = Penalty(distance_expected, minimum_distance);
  const double tol{1E-5};
  EXPECT_NEAR(y_double(0), penalty_expected, tol);
  const Eigen::Vector3d distance_normal =
      (sphere1_position - sphere2_position).normalized();
  Eigen::RowVectorXd ddistance_dq(14);
  ddistance_dq << Eigen::RowVector4d::Zero(), distance_normal.transpose(),
      Eigen::RowVector4d::Zero(), -distance_normal.transpose();
  const auto distance_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(distance_expected), ddistance_dq);
  const auto penalty_autodiff = Penalty(distance_autodiff(0), minimum_distance);
  q_autodiff = math::initializeAutoDiff(q);
  constraint.Eval(q_autodiff, &y);
  EXPECT_NEAR(y(0).value(), y_double(0), 1E-7);
  // The gradient tolerance is very big, because the closest point computed
  // from generaic EPA is not accurate. Hence the gradient of the signed
  // distance to the generalized position (ddistance_dq) isn't accurate inside
  // MinimumDistanceConstraint.Eval()
  const double gradient_tol = 0.03;
  EXPECT_TRUE(CompareMatrices(y(0).derivatives(),
                              penalty_autodiff.derivatives(), gradient_tol));
}

GTEST_TEST(MinimumDistanceConstraintTest,
           MultibodyPlantWithouthGeometrySource) {
  auto plant = ConstructTwoFreeBodiesPlant<AutoDiffXd>();
  auto context = plant->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(*plant, 0.1, context.get()),
      std::invalid_argument,
      "MinimumDistanceConstraint: MultibodyPlant has not registered its "
      "geometry source with SceneGraph yet.");
}

TEST_F(TwoFreeSpheresTest, NonpositiveMinimumDistance) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(*plant_autodiff_, 0, plant_context_autodiff_),
      std::invalid_argument,
      "MinimumDistanceConstraint: minimum_distance should be positive.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(*plant_autodiff_, -0.1,
                                plant_context_autodiff_),
      std::invalid_argument,
      "MinimumDistanceConstraint: minimum_distance should be positive.");
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
