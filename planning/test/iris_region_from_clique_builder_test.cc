#include "drake/planning/iris_region_from_clique_builder.h"

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

GTEST_TEST(IrisRegionFromCliqueBuilder, TestCtorSettersAndGetters) {
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

  IrisOptions options{};
  // Change one of the default fields so we can be sure the options are set
  // properly.
  options.iteration_limit = 2;

  const double rank_tol_for_lowner_john_ellipse = 1e-6;

  IrisRegionFromCliqueBuilder builder{obstacles, domain, options,
                                      rank_tol_for_lowner_john_ellipse};
  EXPECT_EQ(obstacles.size(), builder.get_obstacles().size());
  EXPECT_EQ(domain.A(), builder.get_domain().A());
  EXPECT_EQ(domain.b(), builder.get_domain().b());
  EXPECT_EQ(options.iteration_limit, builder.get_options().iteration_limit);
  EXPECT_EQ(rank_tol_for_lowner_john_ellipse,
            builder.get_rank_tol_for_lowner_john_ellipse());

  const HPolyhedron domain2 =
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(1, 1));
  builder.set_domain(domain2);
  EXPECT_EQ(domain2.A(), builder.get_domain().A());

  ConvexSets obstacles2;
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(0.45, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0), Vector2d(1, 0.45)));
  builder.set_obstacles(obstacles2);
  EXPECT_EQ(obstacles2.size(), builder.get_obstacles().size());

  IrisOptions options2{};
  options2.iteration_limit = 4;
  builder.set_options(options2);
  EXPECT_EQ(options2.iteration_limit, builder.get_options().iteration_limit);

  const double rank_tol_for_lowner_john_ellipse2 = 1e-10;
  builder.set_rank_tol_for_lowner_john_ellipse(
      rank_tol_for_lowner_john_ellipse2);
  EXPECT_EQ(rank_tol_for_lowner_john_ellipse2,
            builder.get_rank_tol_for_lowner_john_ellipse());
}

GTEST_TEST(IrisRegionFromCliqueBuilder, TestCtorDefaults) {
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
  IrisRegionFromCliqueBuilder builder{obstacles, domain};
  EXPECT_EQ(builder.get_options().iteration_limit, 1);
  EXPECT_EQ(builder.get_rank_tol_for_lowner_john_ellipse(), 1e-6);
}

GTEST_TEST(IrisRegionFromCliqueBuilder, TestBuildConvexSet) {
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

  IrisRegionFromCliqueBuilder builder{obstacles, domain};

  const Eigen::Matrix<double, 2, 4> clique1{{0.45, 0.45, 0.55, 0.55},
                                            {0.0, 1.0, 0.0, 1.0}};
  copyable_unique_ptr<ConvexSet> set1_base = builder.BuildConvexSet(clique1);
  const HPolyhedron* set1 = dynamic_cast<const HPolyhedron*>(set1_base.get());
  // The convex hull of these points is approximately Box1.
  const Eigen::Matrix<double, 2, 4> test_points1{{0.46, 0.46, 0.54, 0.54},
                                                 {0.01, 0.99, 0.01, 0.99}};
  for (int i = 0; i < test_points1.cols(); ++i) {
    EXPECT_TRUE(set1->PointInSet(test_points1.col(i), 1e-6));
  }

  const Eigen::Matrix<double, 2, 4> clique2{{0.0, 1.0, 0.0, 1.0},
                                            {0.45, 0.45, 0.55, 0.55}};
  copyable_unique_ptr<ConvexSet> set2_base = builder.BuildConvexSet(clique2);
  const HPolyhedron* set2 = dynamic_cast<const HPolyhedron*>(set2_base.get());
  // The convex hull of these points is approximately Box2.
  const Eigen::Matrix<double, 2, 4> test_points2{{0.01, 0.99, 0.01, 0.99},
                                                 {0.46, 0.46, 0.54, 0.54}};
  for (int i = 0; i < test_points2.cols(); ++i) {
    EXPECT_TRUE(set2->PointInSet(test_points2.col(i), 1e-6));
  }
}

}  // namespace
}  // namespace planning
}  // namespace drake
