#include "drake/geometry/optimization/iris.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Matrix;
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

// One prismatic link with joint limits.  Iris should return the joint limits.
GTEST_TEST(DeprecatedIrisInConfigurationSpaceTest, JointLimits) {
  const std::string limits_urdf = R"(
<robot name="limits">
  <link name="movable">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)";

  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  multibody::Parser(&plant).AddModelFromString(limits_urdf, "urdf");
  plant.Finalize();
  auto diagram = builder.Build();

  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  auto context = diagram->CreateDefaultContext();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  HPolyhedron region = IrisInConfigurationSpace(
      plant, plant.GetMyContextFromRoot(*context), sample, options);
#pragma GCC diagnostic pop

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-5;
  const double qmin = -2.0, qmax = 2.0;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

namespace {

// Helper method for testing IrisInConfigurationSpace from a urdf string.
HPolyhedron IrisFromUrdf(const std::string urdf,
                         const Eigen::Ref<const Eigen::VectorXd>& sample,
                         const IrisOptions& options) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  multibody::Parser(&plant).AddModelFromString(urdf, "urdf");
  plant.Finalize();
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()), sample);
  return IrisInConfigurationSpace(plant, plant.GetMyContextFromRoot(*context),
                                  options);
}

}  // namespace

// One prismatic link with joint limits.  Iris should return the joint limits.
GTEST_TEST(IrisInConfigurationSpaceTest, JointLimits) {
  const std::string limits_urdf = R"(
<robot name="limits">
  <link name="movable">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)";

  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(limits_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-5;
  const double qmin = -2.0, qmax = 2.0;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

// Three boxes.  Two on the outside are fixed.  One in the middle on a prismatic
// joint.  The configuration space is a (convex) line segment q ∈ (−1,1).
GTEST_TEST(IrisInConfigurationSpaceTest, BoxesPrismatic) {
  const std::string boxes_urdf = R"(
<robot name="boxes">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="2 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-2 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="center">
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)";

  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(boxes_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-3;  // due to ibex's rel_eps_f.
  const double qmin = -1.0 + options.configuration_space_margin,
               qmax = 1.0 - options.configuration_space_margin;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

// Three spheres.  Two on the outside are fixed.  One in the middle on a
// prismatic joint.  The configuration space is a (convex) line segment q ∈
// (−1,1).
GTEST_TEST(IrisInConfigurationSpaceTest, SpheresPrismatic) {
  const std::string spheres_urdf = R"(
<robot name="spheres">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="2 0 0"/>
      <geometry><sphere radius=".5"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-2 0 0"/>
      <geometry><sphere radius=".5"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="center">
      <geometry><sphere radius=".5"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)";

  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(spheres_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-3;  // due to ibex's rel_eps_f.
  const double qmin = -1.0 + options.configuration_space_margin,
               qmax = 1.0 - options.configuration_space_margin;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

// A simple pendulum of length `l` with a sphere at the tip of radius `r`
// between two (fixed) walls at `w` from the origin.  The configuration space
// is a (convex) line segment q ∈ (sin⁻¹((-w+r)/l), sin((w-r)/l)).
GTEST_TEST(IrisInConfigurationSpaceTest, SinglePendulum) {
  const double l = 2.0;
  const double r = .5;
  const double w = 1.0;
  const std::string pendulum_urdf = fmt::format(
      R"(
<robot name="pendulum">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="pendulum">
    <collision name="ball">
      <origin rpy="0 0 0" xyz="0 0 -{l}"/>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
  </link>
  <joint name="pendulum" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="world"/>
    <child link="pendulum"/>
  </joint>
</robot>
)",
      fmt::arg("w_plus_one_half", w + .5), fmt::arg("l", l), fmt::arg("r", r));

  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(pendulum_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-3;  // due to ibex's rel_eps_f.
  const double qmin =
                   std::asin((-w + r) / l) + options.configuration_space_margin,
               qmax =
                   std::asin((w - r) / l) - options.configuration_space_margin;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

// A simple double pendulum with link lengths `l1` and `l2` with a sphere at the
// tip of radius `r` between two (fixed) walls at `w` from the origin.  The
// true configuration space is - w + r ≤ l₁s₁ + l₂s₁₊₂ ≤ w - r.  These regions
// are visualized at https://www.desmos.com/calculator/ff0hbnkqhm.
GTEST_TEST(IrisInConfigurationSpaceTest, DoublePendulum) {
  const double l1 = 2.0;
  const double l2 = 1.0;
  const double r = .5;
  const double w = 1.83;
  const std::string double_pendulum_urdf = fmt::format(
      R"(
<robot name="double_pendulum">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link2">
    <collision name="ball">
      <origin rpy="0 0 0" xyz="0 0 -{l2}"/>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -{l1}"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)",
      fmt::arg("w_plus_one_half", w + .5), fmt::arg("l1", l1),
      fmt::arg("l2", l2), fmt::arg("r", r));

  const Vector2d sample = Vector2d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(double_pendulum_urdf, sample, options);

  // Note: You may use this to plot the solution in the desmos graphing
  // calculator link above.  Just copy each equation in the printed formula into
  // a desmos cell.  The intersection is the computed region.
  // const Vector2<symbolic::Expression> xy{symbolic::Variable("x"),
  //                                       symbolic::Variable("y")};
  // std::cout << (region.A()*xy <= region.b()) << std::endl;

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 2.0);

  EXPECT_TRUE(region.PointInSet(Vector2d{.4, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{.5, 0.0}));
  EXPECT_TRUE(region.PointInSet(Vector2d{.3, .3}));
  EXPECT_FALSE(region.PointInSet(Vector2d{.4, .3}));
  EXPECT_TRUE(region.PointInSet(Vector2d{-.4, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{-.5, 0.0}));
  EXPECT_TRUE(region.PointInSet(Vector2d{-.3, -.3}));
  EXPECT_FALSE(region.PointInSet(Vector2d{-.4, -.3}));
}

// A block on a vertical track, free to rotate (in the plane) with width `w` and
// height `h`, plus a ground plane at z=0.  The true configuration space is
// min(q₀ ± .5w sin(q₁) ± .5h cos(q₁)) ≥ 0, where the min is over the ±.  These
// regions are visualized at https://www.desmos.com/calculator/ok5ckpa1kp.
GTEST_TEST(IrisInConfigurationSpaceTest, BlockOnGround) {
  const double w = 2.0;
  const double h = 1.0;
  const std::string block_urdf = fmt::format(
      R"(
<robot name="block">
  <link name="fixed">
    <collision name="ground">
      <origin rpy="0 0 0" xyz="0 0 -1"/>
      <geometry><box size="10 10 2"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="link1"/>
  <joint name="joint1" type="prismatic">
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="3.0"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link2">
    <collision name="block">
      <geometry><box size="{w} 1 {h}"/></geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)",
      fmt::arg("w", w), fmt::arg("h", h));

  const Vector2d sample{1.0, 0.0};
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(block_urdf, sample, options);

  // Note: You may use this to plot the solution in the desmos graphing
  // calculator link above.  Just copy each equation in the printed formula into
  // a desmos cell.  The intersection is the computed region.
  // const Vector2<symbolic::Expression> xy{symbolic::Variable("x"),
  //                                        symbolic::Variable("y")};
  // std::cout << (region.A()*xy <= region.b()) << std::endl;

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 2.0);
}

// A (somewhat contrived) example of a concave configuration-space obstacle
// (resulting in a convex configuration-space, which we approximate with
// polytopes):  A simple pendulum of length `l` with a sphere at the tip of
// radius `r` on a vertical track, plus a ground plane at z=0.  The
// configuration space is given by the joint limits and z + l*cos(theta) >= r.
// The region is visualized at https://www.desmos.com/calculator/flshvay78b.
// In addition to testing the convex space, this is a test for which Ibex finds
// counter-examples that Snopt misses.
GTEST_TEST(IrisInConfigurationSpaceTest, ConvexConfigurationSpace) {
  const double l = 1.5;
  const double r = 0.1;
  const std::string convex_urdf = fmt::format(
      R"(
<robot name="pendulum_on_vertical_track">
  <link name="fixed">
    <collision name="ground">
      <origin rpy="0 0 0" xyz="0 0 -1"/>
      <geometry><box size="10 10 2"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="cart">
  </link>
  <joint name="track" type="prismatic">
    <axis xyz="0 0 1"/>
    <limit lower="-{l}" upper="0"/>
    <parent link="world"/>
    <child link="cart"/>
  </joint>
  <link name="pendulum">
    <collision name="ball">
      <origin rpy="0 0 0" xyz="0 0 {l}"/>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
  </link>
  <joint name="pendulum" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="cart"/>
    <child link="pendulum"/>
  </joint>
</robot>
)",
      fmt::arg("l", l), fmt::arg("r", r));

  const Vector2d sample{1.0, 0.0};
  IrisOptions options;
  options.enable_ibex = true;
  HPolyhedron region = IrisFromUrdf(convex_urdf, sample, options);

  // Note: You may use this to plot the solution in the desmos graphing
  // calculator link above.  Just copy each equation in the printed formula into
  // a desmos cell.  The intersection is the computed region.
  // const Vector2<symbolic::Expression> xy{symbolic::Variable("x"),
  //                                        symbolic::Variable("y")};
  // std::cout << (region.A()*xy <= region.b()) << std::endl;

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 0.5);

  // Without Ibex, we find that SNOPT misses this point. It should be outside of
  // the configuration space (in collision).  The particular value was found by
  // visual inspection using the desmos plot.
  const double z_test = 0, theta_test = -1.55;
  EXPECT_FALSE(region.PointInSet(Vector2d{z_test, theta_test}));
  // Confirm that the pendulum is colliding with the wall with true kinematics:
  EXPECT_LE(z_test + l*std::cos(theta_test), r);
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
