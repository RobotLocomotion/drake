#include "drake/geometry/optimization/geodesic_convexity.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
using geometry::optimization::internal::ComputeOffsetContinuousRevoluteJoints;
using geometry::optimization::internal::GetMinimumAndMaximumValueAlongDimension;

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

GTEST_TEST(GeodesicConvexityTest, GetMinimumAndMaximumValueAlongDimension) {
  Eigen::Matrix<double, 2, 4> vertices;
  // clang-format off
  vertices << -2.0, 2.0, -2.0, 2.0,
               0.0, 0.0,  4.0, 4.0;
  // clang-format on
  VPolytope v(vertices);
  const double kTol = 1e-11;

  std::pair<double, double> min_max;
  min_max = GetMinimumAndMaximumValueAlongDimension(v, 0);
  EXPECT_NEAR(min_max.first, -2.0, kTol);
  EXPECT_NEAR(min_max.second, 2.0, kTol);
  min_max = GetMinimumAndMaximumValueAlongDimension(v, 1);
  EXPECT_NEAR(min_max.first, 0.0, kTol);
  EXPECT_NEAR(min_max.second, 4.0, kTol);

  std::vector<std::pair<double, double>> min_maxs =
      GetMinimumAndMaximumValueAlongDimension(v, std::vector<int>{0, 1});
  ASSERT_EQ(min_maxs.size(), 2);
  EXPECT_NEAR(min_maxs[0].first, -2.0, kTol);
  EXPECT_NEAR(min_maxs[0].second, 2.0, kTol);
  EXPECT_NEAR(min_maxs[1].first, 0.0, kTol);
  EXPECT_NEAR(min_maxs[1].second, 4.0, kTol);
}

GTEST_TEST(GeodesicConvexityTest, ComputeOffset) {
  // 1D example.
  Eigen::Matrix<double, 1, 2> points1;
  Eigen::Matrix<double, 1, 2> points2;
  Eigen::Matrix<double, 1, 2> points3;
  points1 << 0, 3;
  points2 << 2.5, 5.5;
  points3 << 5, 8;
  const VPolytope v1(points1);
  const VPolytope v2(points2);
  const VPolytope v3(points3);
  std::vector<int> continuous_revolute_joints{0};

  const auto v1_bbox =
      GetMinimumAndMaximumValueAlongDimension(v1, continuous_revolute_joints);
  const auto v2_bbox =
      GetMinimumAndMaximumValueAlongDimension(v2, continuous_revolute_joints);
  const auto v3_bbox =
      GetMinimumAndMaximumValueAlongDimension(v3, continuous_revolute_joints);

  const int num_positions = 1;
  const double kTol = 1e-9;

  Eigen::VectorXd offset;
  offset = ComputeOffsetContinuousRevoluteJoints(
      num_positions, continuous_revolute_joints, v1_bbox, v2_bbox);
  EXPECT_TRUE(CompareMatrices(offset, Vector1d(0.0), kTol));
  offset = ComputeOffsetContinuousRevoluteJoints(
      num_positions, continuous_revolute_joints, v2_bbox, v1_bbox);
  EXPECT_TRUE(CompareMatrices(offset, Vector1d(0.0), kTol));
  offset = ComputeOffsetContinuousRevoluteJoints(
      num_positions, continuous_revolute_joints, v1_bbox, v3_bbox);
  EXPECT_TRUE(CompareMatrices(offset, Vector1d(2 * M_PI), kTol));
  offset = ComputeOffsetContinuousRevoluteJoints(
      num_positions, continuous_revolute_joints, v3_bbox, v1_bbox);
  EXPECT_TRUE(CompareMatrices(offset, Vector1d(-2 * M_PI), kTol));
  offset = ComputeOffsetContinuousRevoluteJoints(
      num_positions, continuous_revolute_joints, v2_bbox, v3_bbox);
  EXPECT_TRUE(CompareMatrices(offset, Vector1d(0.0), kTol));
  offset = ComputeOffsetContinuousRevoluteJoints(
      num_positions, continuous_revolute_joints, v3_bbox, v2_bbox);
  EXPECT_TRUE(CompareMatrices(offset, Vector1d(0.0), kTol));
}

GTEST_TEST(GeodesicConvexityTest, CalcPairwiseIntersectionsAPI) {
  VPolytope v(Vector1d(1));
  ConvexSets sets = MakeConvexSets(v);
  EXPECT_THROW(CalcPairwiseIntersections({}, std::vector<int>{}),
               std::exception);
  EXPECT_THROW(CalcPairwiseIntersections(sets, {}, std::vector<int>{}),
               std::exception);
  EXPECT_THROW(CalcPairwiseIntersections({}, sets, std::vector<int>{}),
               std::exception);
}

GTEST_TEST(GeodesicConvexityTest, CalcPairwiseIntersections1) {
  Hyperrectangle h1(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0));
  Hyperrectangle h2(Eigen::Vector2d(0.5, 0.5 + 2 * M_PI),
                    Eigen::Vector2d(1.5, 1.5 + 2 * M_PI));
  Hyperrectangle h3(Eigen::Vector2d(8 * M_PI, 0.0),
                    Eigen::Vector2d(1.0 + 8 * M_PI, 1.0));
  Hyperrectangle h4(Eigen::Vector2d(2 * M_PI, -4 * M_PI),
                    Eigen::Vector2d(1.0 + 2 * M_PI, 1.0 - 4 * M_PI));
  ConvexSets sets_A = MakeConvexSets(h1, h2);
  ConvexSets sets_B = MakeConvexSets(h3, h4);
  // Let's check the expected value of each pair of intersections.
  // h1 <--> h3: would intersect if h1 is translated by (4, 0) * 2π
  // h1 <--> h4: would intersect if h1 is translated by (1, -4) * 2π
  // h2 <--> h3: would intersect if h2 is translated by (4, -1) * 2π
  // h2 <--> h4: would intersect if h2 is translated by (1, -5) * 2π
  // Without continuous revolute joints, the intersection is empty.
  EXPECT_EQ(
      CalcPairwiseIntersections(sets_A, sets_B, std::vector<int>{}).size(), 0);
  // With only first joint continuous, the only intersection is h1 <--> h3.
  EXPECT_EQ(
      CalcPairwiseIntersections(sets_A, sets_B, std::vector<int>{0}).size(), 1);
  // With only second joint continuous, the intersection is empty.
  EXPECT_EQ(
      CalcPairwiseIntersections(sets_A, sets_B, std::vector<int>{1}).size(), 0);
  // With all joints continuous, all possible pairs exist.
  const auto intersection_edges =
      CalcPairwiseIntersections(sets_A, sets_B, std::vector<int>{0, 1});
  EXPECT_EQ(intersection_edges.size(), 4);
  for (const auto& [index_a, index_b, offset] : intersection_edges) {
    // Verify all centers are integer multiples of 2π apart.
    const auto offset_mod_2π = offset.array() / (2 * M_PI);
    for (int i = 0; i < offset_mod_2π.size(); ++i) {
      EXPECT_NEAR(offset_mod_2π[i], std::round(offset_mod_2π[i]), 1e-9);
    }
    // The center of the first rectangle + offset should be inside the second
    // rectangle.
    const auto* h_a =
        dynamic_cast<const Hyperrectangle*>(sets_A[index_a].get());
    DRAKE_DEMAND(h_a != nullptr);
    EXPECT_TRUE(sets_B[index_b]->PointInSet(h_a->Center() + offset, 1e-6));
  }
}

GTEST_TEST(GeodesicConvexityTest, CalcPairwiseIntersections2) {
  // Test the self intersections overload.
  Hyperrectangle h1(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0));
  Hyperrectangle h2(Eigen::Vector2d(0.5, 0.5 + 2 * M_PI),
                    Eigen::Vector2d(1.5, 1.5 + 2 * M_PI));
  Hyperrectangle h3(Eigen::Vector2d(8 * M_PI, 0.0),
                    Eigen::Vector2d(1.0 + 8 * M_PI, 1.0));
  ConvexSets sets = MakeConvexSets(h1, h2, h3);
  // Let's check the expected value of each pair of intersections.
  // h1 <--> h2: would intersect if h1 is translated by (0, 1) * 2π
  // h1 <--> h3: would intersect if h1 is translated by (4, 0) * 2π
  // h2 <--> h3: would intersect if h2 is translated by (4, -1) * 2π
  // Without continuous revolute joints, the intersection is empty.
  auto intersections_none = CalcPairwiseIntersections(sets, std::vector<int>{});
  EXPECT_EQ(intersections_none.size(), 0);
  // With only first joint continuous, the only intersection is h1 <--> h3.
  auto intersections_zero =
      CalcPairwiseIntersections(sets, std::vector<int>{0});
  EXPECT_EQ(intersections_zero.size(), 2);
  EXPECT_EQ(std::get<0>(intersections_zero[0]), 0);
  EXPECT_EQ(std::get<1>(intersections_zero[0]), 2);
  EXPECT_TRUE(CompareMatrices(std::get<2>(intersections_zero[0]),
                              Eigen::Vector2d(8 * M_PI, 0.0), 1e-9));
  // With all joints continuous, all pairs of intersections are non-empty.
  auto intersections_all =
      CalcPairwiseIntersections(sets, std::vector<int>{0, 1});
  EXPECT_EQ(intersections_all.size(), 6);
  for (const auto& [index_1, index_2, offset] : intersections_all) {
    // Verify all centers are integer multiples of 2π apart.
    const auto offset_mod_2π = offset.array() / (2 * M_PI);
    for (int i = 0; i < offset_mod_2π.size(); ++i) {
      EXPECT_NEAR(offset_mod_2π[i], std::round(offset_mod_2π[i]), 1e-9);
    }
    // The center of the first rectangle + offset should be inside the second
    // rectangle.
    const auto* h_first =
        dynamic_cast<const Hyperrectangle*>(sets[index_1].get());
    DRAKE_DEMAND(h_first != nullptr);
    EXPECT_TRUE(sets[index_2]->PointInSet(h_first->Center() + offset, 1e-6));
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
