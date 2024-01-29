#include "drake/geometry/optimization/geodesic_convexity.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/vpolytope.h"

namespace drake {
namespace geometry {
namespace optimization {

using geometry::optimization::Hyperrectangle;
using geometry::optimization::Intersection;
using geometry::optimization::VPolytope;

GTEST_TEST(GeodesicConvexityTest, PartitionConvexSetAPI) {
  Eigen::Matrix<double, 2, 4> vertices;
  // clang-format off
  vertices << 0.0, 4.0, 0.0, 4.0,
              0.0, 0.0, 4.0, 4.0;
  // clang-format on
  VPolytope v(vertices);
  std::vector<int> continuous_revolute_joints{0, 1};
  const double epsilon = 1e-5;

  // Test the function overload that takes in multiple convex sets.
  ConvexSets sets = PartitionConvexSet(MakeConvexSets(v, v),
                                       continuous_revolute_joints, epsilon);
  EXPECT_EQ(sets.size(), 8);
  for (const auto& set : sets) {
    EXPECT_TRUE(
        CheckIfSatisfiesConvexityRadius(*set, continuous_revolute_joints));
  }

  // List of convex sets must have at least one entry.
  EXPECT_THROW(PartitionConvexSet(MakeConvexSets(), continuous_revolute_joints),
               std::exception);

  // List of convex sets must not have null pointers.
  ConvexSets contains_nullptr;
  contains_nullptr.push_back(copyable_unique_ptr<ConvexSet>(nullptr));
  EXPECT_THROW(PartitionConvexSet(contains_nullptr, continuous_revolute_joints),
               std::exception);

  // List of convex sets must not sets which are unbounded on dimensions
  // corresponding to continuous revolute joints.
  Eigen::Matrix<double, 1, 2> A;
  A << 1, 0;
  Eigen::VectorXd b(1);
  b << 0;
  HPolyhedron unbounded_hpolyhedron(A, b);
  EXPECT_THROW(
      PartitionConvexSet(unbounded_hpolyhedron, continuous_revolute_joints),
      std::exception);
  EXPECT_THROW(PartitionConvexSet(MakeConvexSets(unbounded_hpolyhedron),
                                  continuous_revolute_joints),
               std::exception);

  // But unbounded dimensions are allowed on dimensions not corresponding to
  // continuous revolute joints.
  EXPECT_NO_THROW(
      PartitionConvexSet(unbounded_hpolyhedron, std::vector<int>{}));
  EXPECT_NO_THROW(PartitionConvexSet(MakeConvexSets(unbounded_hpolyhedron),
                                     std::vector<int>{}));

  // List of convex sets must have matching ambient dimension.
  VPolytope v_wrong_dimension(Eigen::MatrixXd::Zero(3, 1));
  DRAKE_EXPECT_THROWS_MESSAGE(
      PartitionConvexSet(MakeConvexSets(v, v_wrong_dimension),
                         continuous_revolute_joints),
      ".*convex_sets\\[i\\]->ambient_dimension.*");
  EXPECT_THROW(PartitionConvexSet(MakeConvexSets(v, v_wrong_dimension),
                                  continuous_revolute_joints),
               std::exception);

  // Epsilon must be strictly positive.
  EXPECT_THROW(PartitionConvexSet(v, continuous_revolute_joints, 0.0),
               std::exception);

  // Epsilon must be less than π.
  EXPECT_THROW(PartitionConvexSet(v, continuous_revolute_joints, M_PI),
               std::exception);
}

bool PointInASet(const ConvexSets& sets, const Eigen::VectorXd& point,
                 const double kTol) {
  for (int i = 0; i < ssize(sets); ++i) {
    if (sets[i]->PointInSet(point, kTol)) {
      return true;
    }
  }
  return false;
}

GTEST_TEST(GeodesicConvexityTest, PartitionConvexSet1) {
  // Example 1: partition along both dimensions.
  Eigen::Matrix<double, 2, 4> vertices;
  // clang-format off
  vertices << 0.0, 4.0, 0.0, 4.0,
              0.0, 0.0, 4.0, 4.0;
  // clang-format on
  VPolytope v(vertices);
  std::vector<int> continuous_revolute_joints{0, 1};
  const double epsilon = 1e-5;

  EXPECT_TRUE(CheckIfSatisfiesConvexityRadius(v, std::vector<int>{}));
  EXPECT_FALSE(CheckIfSatisfiesConvexityRadius(v, std::vector<int>{0}));
  EXPECT_FALSE(CheckIfSatisfiesConvexityRadius(v, std::vector<int>{1}));
  EXPECT_FALSE(CheckIfSatisfiesConvexityRadius(v, continuous_revolute_joints));

  ConvexSets sets = PartitionConvexSet(v, continuous_revolute_joints, epsilon);
  EXPECT_EQ(sets.size(), 4);
  for (const auto& set : sets) {
    EXPECT_TRUE(
        CheckIfSatisfiesConvexityRadius(*set, continuous_revolute_joints));
  }

  const double kTol = 1e-11;
  EXPECT_TRUE(PointInASet(sets, Eigen::Vector2d(0.0, 0.0), kTol));
  EXPECT_TRUE(PointInASet(sets, Eigen::Vector2d(4.0, 0.0), kTol));
  EXPECT_TRUE(PointInASet(sets, Eigen::Vector2d(0.0, 4.0), kTol));
  EXPECT_TRUE(PointInASet(sets, Eigen::Vector2d(4.0, 4.0), kTol));
  for (int i = 0; i < ssize(sets); ++i) {
    EXPECT_TRUE(sets[i]->PointInSet(
        Eigen::Vector2d(M_PI - 2.0 * epsilon, M_PI - 2.0 * epsilon), kTol));
  }
}

GTEST_TEST(GeodesicConvexityTest, PartitionConvexSet2) {
  // Example 2: Partition along one dimension.
  Eigen::Matrix<double, 2, 4> vertices2;
  // clang-format off
  vertices2 << 0.0, 2.0, 0.0, 2.0,
               0.0, 0.0, 4.0, 4.0;
  // clang-format on
  VPolytope v2(vertices2);
  std::vector<int> continuous_revolute_joints{0, 1};
  const double epsilon = 1e-5;
  ConvexSets sets = PartitionConvexSet(v2, continuous_revolute_joints, epsilon);
  EXPECT_EQ(sets.size(), 2);
  for (const auto& set : sets) {
    EXPECT_TRUE(
        CheckIfSatisfiesConvexityRadius(*set, continuous_revolute_joints));
  }

  // Check that sets overlap by epsilon.
  const auto maybe_intersection_bbox =
      Hyperrectangle::MaybeCalcAxisAlignedBoundingBox(
          Intersection(*sets[0], *sets[1]));
  ASSERT_TRUE(maybe_intersection_bbox.has_value());
  const Hyperrectangle intersection_bbox = maybe_intersection_bbox.value();
  for (int i = 0; i < intersection_bbox.ambient_dimension(); ++i) {
    // Due to solver numerics, the overlap may be slightly less than the given
    // epsilon value. So we instead require that the overlap be at least
    // epsilon - delta, where delta is a small constant.
    const double delta = 1e-7;
    DRAKE_DEMAND(delta < epsilon);
    EXPECT_TRUE(intersection_bbox.ub()[i] - intersection_bbox.lb()[i] >=
                epsilon - delta);
  }
}

GTEST_TEST(GeodesicConvexityTest, PartitionConvexSet3) {
  // Example 3: partition along no dimensions.
  Eigen::Matrix<double, 2, 4> vertices3;
  // clang-format off
  vertices3 << 0.0, 2.0, 0.0, 2.0,
               0.0, 0.0, 2.0, 2.0;
  // clang-format on
  VPolytope v3(vertices3);
  std::vector<int> continuous_revolute_joints{0, 1};
  const double epsilon = 1e-5;
  ConvexSets sets = PartitionConvexSet(v3, continuous_revolute_joints, epsilon);
  EXPECT_EQ(sets.size(), 1);
  for (const auto& set : sets) {
    EXPECT_TRUE(
        CheckIfSatisfiesConvexityRadius(*set, continuous_revolute_joints));
  }
}

GTEST_TEST(GeodesicConvexityTest, PartitionConvexSet4) {
  // Example 4: partition along one dimension, with not all joints continuous
  // revolute.
  Eigen::Matrix<double, 2, 4> vertices;
  // clang-format off
  vertices << 0.0, 4.0, 0.0, 4.0,
              0.0, 0.0, 4.0, 4.0;
  // clang-format on
  VPolytope v(vertices);
  std::vector<int> continuous_revolute_joints{0, 1};
  std::vector<int> only_one_continuous_revolute_joint{1};

  ConvexSets sets = PartitionConvexSet(v, only_one_continuous_revolute_joint);
  EXPECT_EQ(sets.size(), 2);
  for (const auto& set : sets) {
    EXPECT_TRUE(CheckIfSatisfiesConvexityRadius(
        *set, only_one_continuous_revolute_joint));
  }

  bool all_satisfy = true;
  for (const auto& set : sets) {
    all_satisfy = all_satisfy && CheckIfSatisfiesConvexityRadius(
                                     *set, continuous_revolute_joints);
  }
  EXPECT_FALSE(all_satisfy);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
