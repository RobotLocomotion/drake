#include "drake/geometry/optimization/geodesic_convexity.h"

#include <utility>
#include <vector>

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

GTEST_TEST(GeodesicConvexityTest, ComputePairwiseIntersectionsAPI) {
  VPolytope v(Vector1d(1));
  ConvexSets sets = MakeConvexSets(v);
  EXPECT_THROW(ComputePairwiseIntersections({}, std::vector<int>{}),
               std::exception);
  EXPECT_THROW(ComputePairwiseIntersections(sets, {}, std::vector<int>{}),
               std::exception);
  EXPECT_THROW(ComputePairwiseIntersections({}, sets, std::vector<int>{}),
               std::exception);

  // The AABBs must be the right size.
  Hyperrectangle bbox(Vector1d(1), Vector1d(1));
  std::vector<Hyperrectangle> bad_bboxes;
  bad_bboxes.push_back(bbox);
  bad_bboxes.push_back(bbox);
  std::vector<Hyperrectangle> good_bboxes;
  good_bboxes.push_back(bbox);
  EXPECT_THROW(
      ComputePairwiseIntersections(sets, std::vector<int>{}, bad_bboxes),
      std::exception);
  EXPECT_THROW(ComputePairwiseIntersections(sets, sets, std::vector<int>{},
                                            bad_bboxes, good_bboxes),
               std::exception);
  EXPECT_THROW(ComputePairwiseIntersections(sets, sets, std::vector<int>{},
                                            good_bboxes, bad_bboxes),
               std::exception);

  // The ambient dimensions must line up correctly.
  Hyperrectangle wrong_dim_bbox_1(Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  Hyperrectangle wrong_dim_bbox_2(Eigen::Vector2d(2, 2), Eigen::Vector2d(3, 3));
  std::vector<Hyperrectangle> wrong_dim_bboxes_1;
  wrong_dim_bboxes_1.push_back(wrong_dim_bbox_1);
  std::vector<Hyperrectangle> wrong_dim_bboxes_2;
  wrong_dim_bboxes_2.push_back(wrong_dim_bbox_2);
  // If all bounding boxes agreed on dimension, but did not match the convex
  // sets, no intersection checks would be performed. This checks that an
  // error should still be thrown.
  EXPECT_THROW(
      ComputePairwiseIntersections(sets, sets, std::vector<int>{},
                                   wrong_dim_bboxes_1, wrong_dim_bboxes_2),
      std::exception);
}

GTEST_TEST(GeodesicConvexityTest, ComputePairwiseIntersections1) {
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

  // Cross-check that our BUILD file allows parallelism.
  DRAKE_DEMAND(Parallelism::Max().num_threads() > 1);

  for (auto parallelism : {Parallelism::None(), Parallelism::Max()}) {
    auto [edges_none, offsets_none] =
        ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{},
                                     true /* preprocess_bbox */, parallelism);
    auto [edges_zero, offsets_zero] =
        ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{0},
                                     true /* preprocess_bbox */, parallelism);
    auto [edges_one, offsets_one] =
        ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{1},
                                     true /* preprocess_bbox */, parallelism);
    auto [edges_both, offsets_both] =
        ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{0, 1},
                                     true /* preprocess_bbox */, parallelism);

    EXPECT_EQ(edges_none.size(), offsets_none.size());
    EXPECT_EQ(edges_zero.size(), offsets_zero.size());
    EXPECT_EQ(edges_one.size(), offsets_one.size());
    EXPECT_EQ(edges_both.size(), offsets_both.size());

    // Without continuous revolute joints, the intersection is empty.
    EXPECT_EQ(edges_none.size(), 0);
    // With only first joint continuous, the only intersection is h1 <--> h3.
    EXPECT_EQ(edges_zero.size(), 1);
    // With only second joint continuous, the intersection is empty.
    EXPECT_EQ(edges_one.size(), 0);
    // With all joints continuous, all possible pairs exist.
    EXPECT_EQ(edges_both.size(), 4);

    // All results should be the same if the bounding boxes are provided.
    std::vector<Hyperrectangle> bboxes_A = {h1, h2};
    std::vector<Hyperrectangle> bboxes_B = {h3, h4};
    EXPECT_EQ(ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{},
                                           bboxes_A, bboxes_B, parallelism)
                  .first.size(),
              0);
    EXPECT_EQ(ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{0},
                                           bboxes_A, bboxes_B, parallelism)
                  .first.size(),
              1);
    EXPECT_EQ(ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{1},
                                           bboxes_A, bboxes_B, parallelism)
                  .first.size(),
              0);
    const auto [other_intersection_edges, other_intersection_edge_offsets] =
        ComputePairwiseIntersections(sets_A, sets_B, std::vector<int>{0, 1},
                                     bboxes_A, bboxes_B);
    ASSERT_EQ(other_intersection_edges.size(),
              other_intersection_edge_offsets.size());
    ASSERT_EQ(other_intersection_edges.size(), 4);
    const auto& intersection_edges = edges_both;
    const auto& intersection_edge_offsets = offsets_both;
    // We use assert because we need the lengths of intersection_edges and
    // other_intersection_edges to be the same for this next check.
    for (int i = 0; i < ssize(intersection_edges); ++i) {
      // Check that each entry is the same. The first two elements of the tuple
      // are integers, so we can compare directly. The last entry is an
      // Eigen::VectorXd, so we have to use CompareMatrices.
      EXPECT_EQ(intersection_edges[i].first, other_intersection_edges[i].first);
      EXPECT_EQ(intersection_edges[i].second,
                other_intersection_edges[i].second);
      EXPECT_TRUE(CompareMatrices(intersection_edge_offsets[i],
                                  other_intersection_edge_offsets[i], 1e-15));
    }

    for (int i = 0; i < ssize(intersection_edges); ++i) {
      const int index_a = intersection_edges[i].first;
      const int index_b = intersection_edges[i].second;
      const Eigen::VectorXd& offset = intersection_edge_offsets[i];

      // Verify all centers are integer multiples of 2π apart.
      const auto offset_mod_2π = offset.array() / (2 * M_PI);
      for (int j = 0; j < offset_mod_2π.size(); ++j) {
        EXPECT_NEAR(offset_mod_2π[j], std::round(offset_mod_2π[j]), 1e-9);
      }
      // The center of the first rectangle + offset should be inside the second
      // rectangle.
      const auto* h_a =
          dynamic_cast<const Hyperrectangle*>(sets_A[index_a].get());
      DRAKE_DEMAND(h_a != nullptr);
      EXPECT_TRUE(sets_B[index_b]->PointInSet(h_a->Center() + offset, 1e-6));
    }
  }
}

GTEST_TEST(GeodesicConvexityTest, ComputePairwiseIntersections2) {
  // Test the self intersections overload.
  Hyperrectangle h1(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0));
  Hyperrectangle h2(Eigen::Vector2d(0.5, 0.5 + 2 * M_PI),
                    Eigen::Vector2d(1.5, 1.5 + 2 * M_PI));
  Hyperrectangle h3(Eigen::Vector2d(8 * M_PI, 0.0),
                    Eigen::Vector2d(1.0 + 8 * M_PI, 1.0));
  ConvexSets sets = MakeConvexSets(h1, h2, h3);

  // Cross-check that our BUILD file allows parallelism.
  DRAKE_DEMAND(Parallelism::Max().num_threads() > 1);

  for (auto parallelism : {Parallelism::None(), Parallelism::Max()}) {
    // Let's check the expected value of each pair of intersections.
    // h1 <--> h2: would intersect if h1 is translated by (0, 1) * 2π
    // h1 <--> h3: would intersect if h1 is translated by (4, 0) * 2π
    // h2 <--> h3: would intersect if h2 is translated by (4, -1) * 2π
    // Without continuous revolute joints, the intersection is empty.
    auto [intersections_none, offsets_none] = ComputePairwiseIntersections(
        sets, std::vector<int>{}, true /* preprocess_bbox */, parallelism);
    EXPECT_EQ(intersections_none.size(), offsets_none.size());
    EXPECT_EQ(intersections_none.size(), 0);
    // With only first joint continuous, the only intersection is h1 <--> h3.
    auto [intersections_zero, offsets_zero] = ComputePairwiseIntersections(
        sets, std::vector<int>{0}, true /* preprocess_bbox */, parallelism);
    EXPECT_EQ(intersections_zero.size(), offsets_zero.size());
    EXPECT_EQ(intersections_zero.size(), 2);
    EXPECT_EQ(intersections_zero[0].first, 0);
    EXPECT_EQ(intersections_zero[0].second, 2);
    EXPECT_TRUE(
        CompareMatrices(offsets_zero[0], Eigen::Vector2d(8 * M_PI, 0.0), 1e-9));
    // With all joints continuous, all pairs of intersections are non-empty.
    auto [intersections_both, offsets_both] = ComputePairwiseIntersections(
        sets, std::vector<int>{0, 1}, true /* preprocess_bbox */, parallelism);
    EXPECT_EQ(intersections_both.size(), offsets_both.size());
    EXPECT_EQ(intersections_both.size(), 6);
    for (int i = 0; i < ssize(intersections_both); ++i) {
      const int index_a = intersections_both[i].first;
      const int index_b = intersections_both[i].second;
      const Eigen::VectorXd& offset = offsets_both[i];
      // Verify all centers are integer multiples of 2π apart.
      const auto offset_mod_2π = offset.array() / (2 * M_PI);
      for (int j = 0; j < offset_mod_2π.size(); ++j) {
        EXPECT_NEAR(offset_mod_2π[j], std::round(offset_mod_2π[j]), 1e-9);
      }
      // The center of the first rectangle + offset should be inside the second
      // rectangle.
      const auto* h_first =
          dynamic_cast<const Hyperrectangle*>(sets[index_a].get());
      DRAKE_DEMAND(h_first != nullptr);
      EXPECT_TRUE(sets[index_b]->PointInSet(h_first->Center() + offset, 1e-6));
    }
  }
}

GTEST_TEST(GeodesicConvexityTest, ContainsNullptrTest) {
  Hyperrectangle h(Vector1d(0.0), Vector1d(1.0));
  ConvexSets sets_ok = MakeConvexSets(h, h);
  ConvexSets sets_nullptr =
      MakeConvexSets(h, copyable_unique_ptr<ConvexSet>(nullptr));
  DRAKE_EXPECT_THROWS_MESSAGE(
      ComputePairwiseIntersections(sets_nullptr, std::vector<int>{}, true,
                                   Parallelism::None()),
      ".*nullptr.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ComputePairwiseIntersections(sets_ok, sets_nullptr, std::vector<int>{},
                                   true, Parallelism::None()),
      ".*nullptr.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ComputePairwiseIntersections(sets_nullptr, sets_ok, std::vector<int>{},
                                   true, Parallelism::None()),
      ".*nullptr.*");
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
