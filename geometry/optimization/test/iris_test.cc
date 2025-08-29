#include "drake/geometry/optimization/iris.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Matrix;
using Eigen::RowVector2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using math::RigidTransformd;

/* Confirms that this recovers the bounding box domain. */
GTEST_TEST(IrisTest, NoObstacles) {
  const ConvexSets obstacles;
  const Vector2d sample{.5, .5};
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);
  const HPolyhedron region = Iris(obstacles, sample, domain);

  // The algorithm converges with only the domain defining the region.
  EXPECT_TRUE(CompareMatrices(region.A(), domain.A()));
  EXPECT_TRUE(CompareMatrices(region.b(), domain.b()));
}

GTEST_TEST(IrisTest, NoObstacles2) {
  const ConvexSets obstacles;
  const Vector2d sample{.5, 0};
  // L1-ball.
  Matrix<double, 4, 2> A;
  // clang-format off
  A <<  1,  1,  // x+y ≤ 1
        1, -1,  // x-y ≤ 1
       -1,  1,  // -x+y ≤ 1
       -1, -1;  // -x-y ≤ 1
  // clang-format on
  Vector4d b = Vector4d::Ones();
  const HPolyhedron domain(A, b);
  const HPolyhedron region = Iris(obstacles, sample, domain);

  // The algorithm converges with only the domain defining the region.
  EXPECT_TRUE(CompareMatrices(region.A(), domain.A()));
  EXPECT_TRUE(CompareMatrices(region.b(), domain.b()));
}

/* Small obstacle in the bottom.
┌───────────────┐
│               │
│               │
│               │
│               │
│               │
│     ┌───┐     │
│     │   │     │
└─────┴───┴─────┘ */
GTEST_TEST(IrisTest, SmallBox) {
  ConvexSets obstacles;
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-.4, -1), Vector2d(.4, -.5)));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);

  const Vector2d sample{.45, -.95};  // low and to the right of the obstacle.
  HPolyhedron region = Iris(obstacles, sample, domain);
  EXPECT_EQ(region.b().size(), 5);  // 4 from bbox + 1 from obstacle.
  EXPECT_TRUE(region.PointInSet(Vector2d(0.3, 0)));      // above the box
  EXPECT_TRUE(region.PointInSet(Vector2d(0.99, 0.99)));  // top right corner
  EXPECT_FALSE(region.PointInSet(sample));

  // Setting the option keeps the sample in the set (but misses above the box).
  IrisOptions options;
  options.require_sample_point_is_contained = true;
  region = Iris(obstacles, sample, domain, options);
  EXPECT_FALSE(region.PointInSet(Vector2d(0.3, 0)));  // above the box
  EXPECT_TRUE(region.PointInSet(sample));
}

/* Unit ball inside the unit box; IRIS finds a region in the top corner. */
GTEST_TEST(IrisTest, BallInBox) {
  ConvexSets obstacles;
  obstacles.emplace_back(Hyperellipsoid::MakeUnitBall(2));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);

  const Vector2d sample{.95, .95};  // top right corner.
  const HPolyhedron region = Iris(obstacles, sample, domain);
  EXPECT_EQ(region.b().size(), 5);  // 4 from bbox + 1 from obstacle.
  // Closest point on the boundary of the circle to the sample point.
  const Vector2d query = Vector2d::Constant(1.0 / std::sqrt(2.0));
  EXPECT_TRUE(region.PointInSet(query + Vector2d::Constant(0.01)));
  EXPECT_FALSE(region.PointInSet(query - Vector2d::Constant(0.01)));
}

/* Box obstacles in all corners.
┌─────┬─┬─────┐
│     │ │     │
│     │ │     │
├─────┘ └─────┤
│             │
├─────┐ ┌─────┤
│     │ │     │
│     │ │     │
└─────┴─┴─────┘ */
GTEST_TEST(IrisTest, MultipleBoxes) {
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.1, .5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-.1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, .5), Vector2d(-.1, 1)));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);

  const Vector2d sample{0, 0};  // center of the bounding box.
  const HPolyhedron region = Iris(obstacles, sample, domain);
  EXPECT_EQ(region.b().size(), 8);  // 4 from bbox + 1 from each obstacle.

  // The ellipsoid will stretch in x, but the polytope will use the inner
  // corners to define the separating hyperplanes.  Check that the boundary
  // points on the x-axis are in, and the boundary points on the y-axis are
  // not.
  EXPECT_TRUE(region.PointInSet(Vector2d(.99, 0.0)));
  EXPECT_TRUE(region.PointInSet(Vector2d(-.99, 0.0)));
  EXPECT_FALSE(region.PointInSet(Vector2d(0.0, .99)));
  EXPECT_FALSE(region.PointInSet(Vector2d(0.0, -.99)));
}

GTEST_TEST(IrisTest, StartingEllipse) {
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.1, .5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-.1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, .5), Vector2d(-.1, 1)));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);

  const Vector2d sample{0, 0};  // center of the bounding box.
  IrisOptions options;
  // Use narrow ellipse that stretches along y-axis.
  Eigen::Matrix2d A;
  A << 0.1, 0, 0, 0.01;
  Hyperellipsoid E(A, sample);
  options.starting_ellipse = E;
  const HPolyhedron region = Iris(obstacles, sample, domain, options);
  EXPECT_EQ(region.b().size(), 8);  // 4 from bbox + 1 from each obstacle.

  // The initial ellipsoid will cause the region to expand in y, but the
  // polytope will use the inner corners to define the separating hyperplanes.
  // Check that the boundary points on the y-axis are in, and the boundary
  // points on the x-axis are not.
  EXPECT_TRUE(region.PointInSet(Vector2d(0.0, .99)));
  EXPECT_TRUE(region.PointInSet(Vector2d(0.0, -.99)));
  EXPECT_FALSE(region.PointInSet(Vector2d(.99, 0.0)));
  EXPECT_FALSE(region.PointInSet(Vector2d(-.99, 0.0)));
}

GTEST_TEST(IrisTest, BoundingRegion) {
  ConvexSets obstacles;
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(0.1, 0.5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-0.1, -0.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -0.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, 0.5), Vector2d(-0.1, 1)));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);

  const Vector2d sample{0, 0};  // center of the bounding box.
  IrisOptions options;

  const HPolyhedron region = Iris(obstacles, sample, domain, options);

  // Use a bounding_region that omits the obstacles to the left of the x-axis.
  options.bounding_region = HPolyhedron(RowVector2d(-1, 0), Vector1d(0.05));

  const HPolyhedron region_w_bounding =
      Iris(obstacles, sample, domain, options);

  EXPECT_EQ(region.b().size(), 8);  // 4 from `domain` and 1 from each obstacle.
  EXPECT_EQ(region_w_bounding.b().size(),
            7);  // 4 from `domain`, 1 from `options.bounding_region` and 2
                 // more from the right obstacles.

  // `region_w_bounding` should not contain points of y ≤ -0.05 since they're
  // outside its bounding region.
  EXPECT_TRUE(region.PointInSet(Vector2d(-0.1, 0)));
  EXPECT_FALSE(region_w_bounding.PointInSet(Vector2d(-0.1, 0)));

  // Points inside the bounding region and outside obstacles should be members
  // of both regions
  EXPECT_TRUE(region.PointInSet(Vector2d(0.99, 0)));
  EXPECT_TRUE(region_w_bounding.PointInSet(Vector2d(0.99, 0)));

  // Points inside obstacles should be excluded from both regions.
  EXPECT_FALSE(region.PointInSet(Vector2d(0.11, 0.51)));
  EXPECT_FALSE(region_w_bounding.PointInSet(Vector2d(0.11, 0.51)));
}

GTEST_TEST(IrisTest, BoundingRegion2) {
  const Vector2d sample{0, 0};

  // The domain is defined by -1 <= x <= 1, with no bounds on y.
  Eigen::MatrixXd domain_A(2, 2);
  Eigen::VectorXd domain_b(2);
  domain_A << 1, 0, -1, 0;
  domain_b << 1, 1;
  const HPolyhedron domain{domain_A, domain_b};

  // The additional bounding region is defined by -1 <= y <= 1, with no bounds
  // on x.
  Eigen::MatrixXd bounding_A(2, 2);
  Eigen::VectorXd bounding_b(2);
  bounding_A << 0, 1, 0, -1;
  bounding_b << 1, 1;

  IrisOptions options;
  options.bounding_region = HPolyhedron{bounding_A, bounding_b};

  // Check both with and without the options.verify_domain_boundedness flag.
  ConvexSets obstacles;
  std::vector<HPolyhedron> regions;
  regions.push_back(Iris(obstacles, sample, domain, options));
  options.verify_domain_boundedness = false;
  regions.push_back(Iris(obstacles, sample, domain, options));
  for (const auto& region : regions) {
    EXPECT_TRUE(region.PointInSet(Vector2d(0.99, 0.99)));
    EXPECT_TRUE(region.PointInSet(Vector2d(-0.99, 0.99)));
    EXPECT_TRUE(region.PointInSet(Vector2d(0.99, -0.99)));
    EXPECT_TRUE(region.PointInSet(Vector2d(-0.99, -0.99)));
  }
}

GTEST_TEST(IrisTest, TerminationConditions) {
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.1, .5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-.1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, .5), Vector2d(-.1, 1)));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);

  const Vector2d sample{0, 0};  // center of the bounding box.
  IrisOptions options;
  options.iteration_limit = 1;
  // Negative thresholds disable the termination condition.
  options.termination_threshold = -1;
  options.relative_termination_threshold = -1;
  const HPolyhedron iteration_region = Iris(obstacles, sample, domain, options);
  EXPECT_EQ(iteration_region.b().size(),
            8);  // 4 from bbox + 1 from each obstacle.

  options.iteration_limit = 100;
  options.termination_threshold = 0.1;
  options.relative_termination_threshold = -1;
  const HPolyhedron abs_volume_region =
      Iris(obstacles, sample, domain, options);
  EXPECT_EQ(abs_volume_region.b().size(),
            8);  // 4 from bbox + 1 from each obstacle.

  options.iteration_limit = 100;
  options.termination_threshold = -1;
  options.relative_termination_threshold = 0.1;
  const HPolyhedron rel_volume_region =
      Iris(obstacles, sample, domain, options);
  EXPECT_EQ(rel_volume_region.b().size(),
            8);  // 4 from bbox + 1 from each obstacle.
}

GTEST_TEST(IrisTest, TerminationFunc) {
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.1, .5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-.1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, .5), Vector2d(-.1, 1)));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(2);
  const Vector2d sample{0, 0};  // center of the bounding box.
  const Vector2d q1{0.15, -0.45};
  const Vector2d q2{-0.05, 0.75};
  IrisOptions options;
  options.iteration_limit = 100;
  options.termination_threshold = -1;
  SetEdgeContainmentTerminationCondition(&options, q1, q2, 1e-3);
  EXPECT_FALSE(options.termination_func(domain));
  const HPolyhedron region = Iris(obstacles, sample, domain, options);
  EXPECT_EQ(region.b().size(), 8);
  EXPECT_TRUE(region.PointInSet(q1));
  EXPECT_TRUE(region.PointInSet(q2));
  EXPECT_TRUE(region.PointInSet(sample));
  EXPECT_FALSE(options.termination_func(region));
  // What if we didn't set the termination condition?
  options.termination_func = nullptr;
  const HPolyhedron region_no_termination =
      Iris(obstacles, sample, domain, options);
  EXPECT_FALSE(region_no_termination.PointInSet(q1) &&
               region_no_termination.PointInSet(q2));
  // failure case when the domain is infeasible.
  const Vector2d q3{3.0, 0};
  SetEdgeContainmentTerminationCondition(&options, q1, q3, 1e-3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Iris(obstacles, sample, domain, options),
      ".* Please check the implementation of your termination_func.");
}

GTEST_TEST(IrisTest, BallInBoxNDims) {
  const int N = 8;
  ConvexSets obstacles;
  obstacles.emplace_back(Hyperellipsoid::MakeUnitBall(N));
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(N);

  const VectorXd sample = VectorXd::Constant(N, .95);  // top right corner.
  const HPolyhedron region = Iris(obstacles, sample, domain);
  EXPECT_EQ(region.b().size(),
            1 + domain.b().size());  // bbox + 1 from obstacle.
  // Closest point on the boundary of the circle to the sample point.
  const VectorXd query = VectorXd::Constant(N, 1.0 / std::sqrt(N));
  EXPECT_TRUE(region.PointInSet(query + VectorXd::Constant(N, 0.01)));
  EXPECT_FALSE(region.PointInSet(query - VectorXd::Constant(N, 0.01)));
}

// A previous version of the IRIS algorithm used containment of the point on an
// obstacle closest to the ellipse center to determine whether an additional
// hyperplane should be added to the region. This tests a case where the closest
// point on an obstacle is outside the region but a different part of the
// obstacle is not. This caused the old approach to yield a region that
// overlapped one of the obstacles.
GTEST_TEST(IrisTest, ClosestPointFailure) {
  Eigen::Vector3d radii(1.457, 1.2, 1.185);
  Eigen::MatrixXd centers(2, 3);
  // clang-format off
  centers << -4.42, -3.89, 0.42,
             -1.58, 1.79, -1.70;
  // clang-format on
  ConvexSets obstacles;
  ConvexSets shrunk_obstacles;
  for (int ii = 0; ii < radii.size(); ++ii) {
    obstacles.emplace_back(
        Hyperellipsoid::MakeHypersphere(radii[ii], centers.col(ii)));
    shrunk_obstacles.emplace_back(
        Hyperellipsoid::MakeHypersphere(0.99 * radii[ii], centers.col(ii)));
  }

  const HPolyhedron domain =
      HPolyhedron::MakeBox(Vector2d::Constant(-10.), Vector2d::Constant(10.));

  IrisOptions options;
  options.require_sample_point_is_contained = true;

  const Vector2d sample(-4.00, 0.25);
  const HPolyhedron region = Iris(obstacles, sample, domain, options);

  for (const auto& o : shrunk_obstacles) {
    EXPECT_FALSE(o->IntersectsWith(region));
  }
}

GTEST_TEST(IrisOptionsTest, Serialize) {
  IrisOptions options;
  options.require_sample_point_is_contained = false;
  options.iteration_limit = 25;
  options.termination_threshold = 1e-3;
  options.relative_termination_threshold = 0.01;
  options.configuration_space_margin = 0.05;
  options.num_collision_infeasible_samples = 8;
  options.num_additional_constraint_infeasible_samples = 3;
  options.random_seed = 789;
  const std::string yaml = yaml::SaveYamlString(options);
  const auto options2 = yaml::LoadYamlString<IrisOptions>(yaml);
  EXPECT_EQ(options.require_sample_point_is_contained,
            options2.require_sample_point_is_contained);
  EXPECT_EQ(options.iteration_limit, options2.iteration_limit);
  EXPECT_EQ(options.termination_threshold, options2.termination_threshold);
  EXPECT_EQ(options.relative_termination_threshold,
            options2.relative_termination_threshold);
  EXPECT_EQ(options.configuration_space_margin,
            options2.configuration_space_margin);
  EXPECT_EQ(options.num_collision_infeasible_samples,
            options2.num_collision_infeasible_samples);
  EXPECT_EQ(options.num_additional_constraint_infeasible_samples,
            options2.num_additional_constraint_infeasible_samples);
  EXPECT_EQ(options.random_seed, options2.random_seed);
}

GTEST_TEST(IrisOptionsTest, SetEdgeContainmentTerminationCondition) {
  IrisOptions options;
  const Vector2d x_1{0.0, 1.0};
  const Vector2d x_2{1.0, 3.0};
  const double epsilon = 1e-3;
  const double tol = 1e-9;
  // default options should not have starting_ellipse or termination_func.
  EXPECT_FALSE(options.starting_ellipse.has_value());
  EXPECT_FALSE(options.termination_func);
  SetEdgeContainmentTerminationCondition(&options, x_1, x_2, epsilon, tol);
  EXPECT_TRUE(options.starting_ellipse.has_value());
  EXPECT_TRUE(options.termination_func);
  const Hyperellipsoid& E = options.starting_ellipse.value();
  // The ellipse should contain both endpoints.
  EXPECT_TRUE(E.PointInSet(x_1, tol));
  EXPECT_TRUE(E.PointInSet(x_2, tol));
  // make a box that only contains up to half of the line segment. Must lead to
  // termination
  const auto half_box = HPolyhedron::MakeBox(Vector2d::Zero(), (x_1 + x_2) / 2);
  EXPECT_TRUE(options.termination_func(half_box));
  // Make a box that contains the line segment. Must not lead to termination
  const auto all_box = HPolyhedron::MakeBox(x_1, x_2);
  EXPECT_FALSE(options.termination_func(all_box));
}

class SceneGraphTester : public ::testing::Test {
 protected:
  void SetUp() { source_id_ = scene_graph_.RegisterSource("test"); }

  const QueryObject<double>& UpdateContextAndGetQueryObject() {
    context_ = scene_graph_.CreateDefaultContext();
    return scene_graph_.get_query_output_port().Eval<QueryObject<double>>(
        *context_);
  }

  GeometryId RegisterAnchoredShape(const Shape& shape,
                                   const RigidTransformd& X_WS) {
    auto instance = std::make_unique<GeometryInstance>(
        X_WS, shape.Clone(), "S" + std::to_string(shape_count_++));
    instance->set_proximity_properties(ProximityProperties());
    return scene_graph_.RegisterAnchoredGeometry(source_id_,
                                                 std::move(instance));
  }

  SceneGraph<double> scene_graph_;
  SourceId source_id_;
  int shape_count_{0};
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(SceneGraphTester, MakeSphere) {
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 0);

  RegisterAnchoredShape(Sphere(1.), RigidTransformd(Vector3d{1., 2., 3.}));
  query = UpdateContextAndGetQueryObject();
  obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 1);
  EXPECT_NE(dynamic_cast<const Hyperellipsoid*>(obstacles[0].get()), nullptr);

  // Confirm that I can have multiple obstacles.
  RegisterAnchoredShape(Sphere(1.), RigidTransformd(Vector3d{1., 2., 3.}));
  query = UpdateContextAndGetQueryObject();
  obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 2);
}

TEST_F(SceneGraphTester, MakeCylinder) {
  RegisterAnchoredShape(Cylinder(1., 2.),
                        RigidTransformd(Vector3d{1., 2., 3.}));
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 1);
  EXPECT_NE(dynamic_cast<const CartesianProduct*>(obstacles[0].get()), nullptr);
}

TEST_F(SceneGraphTester, MakeHalfSpace) {
  RegisterAnchoredShape(HalfSpace(), RigidTransformd(Vector3d{1., 2., 3.}));
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 1);
  EXPECT_NE(dynamic_cast<const HPolyhedron*>(obstacles[0].get()), nullptr);
}

TEST_F(SceneGraphTester, MakeBox) {
  RegisterAnchoredShape(Box(1., 2., 3.), RigidTransformd(Vector3d{1., 2., 3.}));
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 1);
  EXPECT_NE(dynamic_cast<const HPolyhedron*>(obstacles[0].get()), nullptr);
}

TEST_F(SceneGraphTester, MakeCapsule) {
  RegisterAnchoredShape(Capsule(1., 2.), RigidTransformd(Vector3d{1., 2., 3.}));
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 1);
  EXPECT_NE(dynamic_cast<const MinkowskiSum*>(obstacles[0].get()), nullptr);
}

TEST_F(SceneGraphTester, MakeEllipsoid) {
  RegisterAnchoredShape(Ellipsoid(1., 2., 3.),
                        RigidTransformd(Vector3d{1., 2., 3.}));
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 1);
  EXPECT_NE(dynamic_cast<const Hyperellipsoid*>(obstacles[0].get()), nullptr);
}

/* A SceneGraph 3D version of the multiple boxes test.  Viewed from the
positive z-axis down to the origin, we have:
┌─────┬─┬─────┐
│     │ │     │
│     │ │     │
├─────┘ └─────┤
│             │
├─────┐ ┌─────┤
│     │ │     │
│     │ │     │
└─────┴─┴─────┘
and each box spans the entire bounding box in z. */
TEST_F(SceneGraphTester, MultipleBoxes) {
  const HPolyhedron domain = HPolyhedron::MakeUnitBox(3);
  Box box(.9, .5, 2.0);
  RegisterAnchoredShape(box, RigidTransformd(Vector3d{.55, .75, 0.}));
  RegisterAnchoredShape(box, RigidTransformd(Vector3d{-.55, .75, 0.}));
  RegisterAnchoredShape(box, RigidTransformd(Vector3d{.55, -.75, 0.}));
  RegisterAnchoredShape(box, RigidTransformd(Vector3d{-.55, -.75, 0.}));
  auto query = UpdateContextAndGetQueryObject();
  auto obstacles = MakeIrisObstacles(query);
  EXPECT_EQ(obstacles.size(), 4);

  const Vector3d sample = Vector3d::Zero();  // center of the bounding box.
  const HPolyhedron region = Iris(obstacles, sample, domain);

  // As in the 2D version, the ellipsoid will stretch in x, but the polytope
  // will use the inner corners to define the separating hyperplanes.  Check
  // that the boundary points on the x-axis are in, and the boundary points on
  // the y-axis are not.
  EXPECT_TRUE(region.PointInSet(Vector3d(.99, 0.0, 0.0)));
  EXPECT_TRUE(region.PointInSet(Vector3d(-.99, 0.0, 0.0)));
  EXPECT_FALSE(region.PointInSet(Vector3d(0.0, .99, 0.0)));
  EXPECT_FALSE(region.PointInSet(Vector3d(0.0, -.99, 0.0)));

  // We also expect it to elongate in z.
  EXPECT_TRUE(region.PointInSet(Vector3d(0.0, 0.0, 0.99)));
  EXPECT_TRUE(region.PointInSet(Vector3d(0.0, 0.0, -0.99)));
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
