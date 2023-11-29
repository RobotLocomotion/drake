#include "drake/planning/iris_from_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace planning {
namespace {
using Eigen::Vector2d;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::IrisOptions;

GTEST_TEST(IrisFromCliqueCover, BoxWithCornerObstaclesTest) {
  /* A 1x1 box domain with a 0.45x0.45 obstacle in each corner. The free space
   * is a cross composed to the union of
   * Box1 with vertices (0.45,0), (0.45, 1), (0.55, 0), (0.55, 1)
   * and
   * Box2 with vertices (0, 0.45), (0, 0.55), (1, 0.45), (1, 0.55)
   */
  const HPolyhedron domain =
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(1, 1));
  ConvexSets obstacles;
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(0.45, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0), Vector2d(1, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0.55), Vector2d(0.45, 1)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0.55), Vector2d(1, 1)));

  IrisFromCliqueCoverOptions options;
  options.num_builders = 1;
  options.num_points_per_coverage_check=100;
  options.num_points_per_visibility_round=10;
  std::vector<copyable_unique_ptr<HPolyhedron>> sets;
  std::cout << "entering iris from clique cover" << std::endl;
  IrisFromCliqueCover(obstacles, domain, options, &sets);

  std::cout << fmt::format("Got {} sets", sets.size()) << std::endl;

  //  const Eigen::Matrix<double, 2, 4> clique1{{0.45, 0.45, 0.55, 0.55},
  //                                            {0.0, 1.0, 0.0, 1.0}};
  //  copyable_unique_ptr<ConvexSet> set1_base =
  //  builder.BuildConvexSet(clique1); const HPolyhedron* set1 =
  //  dynamic_cast<const HPolyhedron*>(set1_base.get());
  //  // The convex hull of these points is approximately Box1.
  //  const Eigen::Matrix<double, 2, 4> test_points1{{0.46, 0.46, 0.54, 0.54},
  //                                                 {0.01, 0.99, 0.01, 0.99}};
  //  for (int i = 0; i < test_points1.cols(); ++i) {
  //    EXPECT_TRUE(set1->PointInSet(test_points1.col(i), 1e-6));
  //  }
  //
  //  const Eigen::Matrix<double, 2, 4> clique2{{0.0, 1.0, 0.0, 1.0},
  //                                            {0.45, 0.45, 0.55, 0.55}};
  //  copyable_unique_ptr<ConvexSet> set2_base =
  //  builder.BuildConvexSet(clique2); const HPolyhedron* set2 =
  //  dynamic_cast<const HPolyhedron*>(set2_base.get());
  //  // The convex hull of these points is approximately Box2.
  //  const Eigen::Matrix<double, 2, 4> test_points2{{0.01, 0.99, 0.01, 0.99},
  //                                                 {0.46, 0.46, 0.54, 0.54}};
  //  for (int i = 0; i < test_points2.cols(); ++i) {
  //    EXPECT_TRUE(set2->PointInSet(test_points2.col(i), 1e-6));
  //  }
}

}  // namespace
}  // namespace planning
}  // namespace drake