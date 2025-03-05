#include "drake/planning/iris/iris_zo.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace planning {
namespace {

using common::MaybePauseForUser;
using Eigen::Vector2d;
using Eigen::VectorXd;
using geometry::Meshcat;
using geometry::Rgba;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::VPolytope;
using symbolic::Variable;

// Helper method for testing FastIris from a urdf string.
HPolyhedron IrisZoFromUrdf(const std::string urdf,
                           const Hyperellipsoid& starting_ellipsoid,
                           const IrisZoOptions& options,
                           const HPolyhedron* maybe_domain = nullptr) {
  CollisionCheckerParams params;
  RobotDiagramBuilder<double> builder(0.0);

  builder.parser().package_map().AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/package.xml"));
  params.robot_model_instances =
      builder.parser().AddModelsFromString(urdf, "urdf");

  auto plant_ptr = &(builder.plant());
  plant_ptr->Finalize();

  params.model = builder.Build();
  params.edge_step_size = 0.01;
  HPolyhedron domain =
      maybe_domain ? *maybe_domain
                   : HPolyhedron::MakeBox(plant_ptr->GetPositionLowerLimits(),
                                          plant_ptr->GetPositionUpperLimits());
  planning::SceneGraphCollisionChecker checker(std::move(params));
  return IrisZo(checker, starting_ellipsoid, domain, options);
}

// Reproduced from the IrisInConfigurationSpace unit tests.
// One prismatic link with joint limits.  Iris should return the joint limits.
GTEST_TEST(IrisZoTest, JointLimits) {
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
  Hyperellipsoid starting_ellipsoid =
      Hyperellipsoid::MakeHypersphere(1e-2, sample);
  IrisZoOptions options;
  options.verbose = true;

  options.set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return q;
      },
      /* parameterization_is_threadsafe */ false,
      /* parameterization_dimension */ 1);

  // Check that the parameterization was set correctly. (Note the non-default
  // value for parameterization_is_threadsafe.)
  EXPECT_EQ(options.get_parameterization_is_threadsafe(), false);
  ASSERT_TRUE(options.get_parameterization_dimension().has_value());
  EXPECT_EQ(options.get_parameterization_dimension().value(), 1);
  const Vector1d output = options.get_parameterization()(Vector1d(3.0));
  EXPECT_NEAR(output[0], 3.0, 1e-15);

  // Now set the parameterization with parameterization_is_threadsafe set to
  // true.
  options.set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return q;
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);

  HPolyhedron region = IrisZoFromUrdf(limits_urdf, starting_ellipsoid, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-5;
  const double qmin = -2.0, qmax = 2.0;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

// Reproduced from the IrisInConfigurationSpace unit tests.
// A simple double pendulum with link lengths `l1` and `l2` with a sphere at the
// tip of radius `r` between two (fixed) walls at `w` from the origin.  The
// true configuration space is - w + r ≤ l₁s₁ + l₂s₁₊₂ ≤ w - r.  These regions
// are visualized at https://www.desmos.com/calculator/ff0hbnkqhm.
GTEST_TEST(IrisZoTest, DoublePendulum) {
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
  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
  meshcat->Delete("face_pt");
  IrisZoOptions options;
  options.verbose = true;
  options.meshcat = meshcat;
  Hyperellipsoid starting_ellipsoid =
      Hyperellipsoid::MakeHypersphere(1e-2, sample);

  HPolyhedron region =
      IrisZoFromUrdf(double_pendulum_urdf, starting_ellipsoid, options);

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

  {
    meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                             -3.25, 3.25, -3.25, 3.25);
    meshcat->SetProperty("/Grid", "visible", true);
    Eigen::RowVectorXd theta2s =
        Eigen::RowVectorXd::LinSpaced(100, -1.57, 1.57);
    Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 2 * theta2s.size() + 1);
    const double c = -w + r;
    for (int i = 0; i < theta2s.size(); ++i) {
      const double a = l1 + l2 * std::cos(theta2s[i]),
                   b = l2 * std::sin(theta2s[i]);
      // wolfram solve a*sin(q) + b*cos(q) = c for q
      points(0, i) =
          2 * std::atan((std::sqrt(a * a + b * b - c * c) + a) / (b + c)) +
          M_PI;
      points(1, i) = theta2s[i];
      points(0, points.cols() - i - 2) =
          2 * std::atan((std::sqrt(a * a + b * b - c * c) + a) / (b - c)) -
          M_PI;
      points(1, points.cols() - i - 2) = theta2s[i];
    }
    points.col(points.cols() - 1) = points.col(0);
    meshcat->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    MaybePauseForUser();
  }

  // We now test an example of a region grown along a parameterization of the
  // space. We use the rational parameterization s=tan(θ/2), so our
  // parameterization function is θ=2arctan(s).
  options.set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return (2 * q.array().atan()).matrix();
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 2);

  // Check that the parameterization was set correctly.
  EXPECT_EQ(options.get_parameterization_is_threadsafe(), true);
  ASSERT_TRUE(options.get_parameterization_dimension().has_value());
  EXPECT_EQ(options.get_parameterization_dimension().value(), 2);
  const Vector2d output = options.get_parameterization()(Vector2d(0.0, 0.0));
  EXPECT_NEAR(output[0], 0.0, 1e-15);
  EXPECT_NEAR(output[1], 0.0, 1e-15);

  options.configuration_space_margin = 1e-4;
  const Vector2d sample2{0.0, 0.0};
  starting_ellipsoid = Hyperellipsoid::MakeHypersphere(1e-2, sample2);
  // This domain matches the joint limits under the transformation.
  HPolyhedron domain =
      HPolyhedron::MakeBox(Vector2d(-1.0, -1.0), Vector2d(1.0, 1.0));
  region = IrisZoFromUrdf(double_pendulum_urdf, starting_ellipsoid, options,
                          &domain);

  EXPECT_EQ(region.ambient_dimension(), 2);
  Vector2d region_query_point_1(-0.1, 0.3);
  Vector2d region_query_point_2(0.1, -0.3);
  EXPECT_TRUE(region.PointInSet(region_query_point_1));
  EXPECT_TRUE(region.PointInSet(region_query_point_2));

  {
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();

    // Region boundaries appear "curved" in the ambient space, so we use many
    // points per boundary segment to make a more faithful visualization.
    int n_points_per_edge = 10;
    Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(
        3, n_points_per_edge * vregion.vertices().cols() + 1);
    int next_point_index = 0;

    // Order vertices in counterclockwise order.
    Vector2d centroid = vregion.vertices().rowwise().mean();
    Eigen::Matrix2Xd centered = vregion.vertices().colwise() - centroid;
    VectorXd angles = centered.row(1).array().binaryExpr(
        centered.row(0).array(), [](double y, double x) {
          return std::atan2(y, x);
        });
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(
        vregion.vertices().cols(), 0, vregion.vertices().cols() - 1);
    std::sort(indices.data(), indices.data() + vregion.vertices().cols(),
              [&angles](int i1, int i2) {
                return angles(i1) < angles(i2);
              });
    Eigen::Matrix2Xd sorted_vertices = vregion.vertices()(Eigen::all, indices);

    for (int i1 = 0; i1 < sorted_vertices.cols(); ++i1) {
      int i2 = i1 + 1;
      if (i2 == sorted_vertices.cols()) {
        i2 = 0;
      }
      Vector2d q1 = sorted_vertices.col(i1);
      Vector2d q2 = sorted_vertices.col(i2);
      for (int j = 0; j < n_points_per_edge; ++j) {
        double t =
            static_cast<double>(j) / static_cast<double>(n_points_per_edge);
        Vector2d q = t * q2 + (1 - t) * q1;
        points.col(next_point_index).head(2) =
            options.get_parameterization()(q);
        ++next_point_index;
      }
    }
    points.topRightCorner(2, 1) =
        options.get_parameterization()(sorted_vertices.col(0));
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    meshcat->SetObject("Test point", Sphere(0.03), Rgba(1, 0, 0));

    Vector2d ambient_query_point =
        options.get_parameterization()(region_query_point_1);
    meshcat->SetTransform(
        "Test point", math::RigidTransform(Eigen::Vector3d(
                          ambient_query_point[0], ambient_query_point[1], 0)));

    MaybePauseForUser();
  }

  // Verify that we fail gracefully if the parameterization has the wrong output
  // dimension (even if we claim it outputs the correct dimension).
  options.set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return Vector1d(0.0);
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZoFromUrdf(double_pendulum_urdf, starting_ellipsoid, options,
                     &domain),
      ".*wrong dimension.*");
}

const char block_urdf[] = R"(
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
      <geometry><box size="2 1 1"/></geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)";

// Reproduced from the IrisInConfigurationSpace unit tests.
// A block on a vertical track, free to rotate (in the plane) with width `w` of
// 2 and height `h` of 1, plus a ground plane at z=0.  The true configuration
// space is min(q₀ ± .5w sin(q₁) ± .5h cos(q₁)) ≥ 0, where the min is over the
// ±. This region is also visualized at
// https://www.desmos.com/calculator/ok5ckpa1kp.
GTEST_TEST(IrisZoTest, BlockOnGround) {
  const Vector2d sample{1.0, 0.0};
  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
  meshcat->Delete("face_pt");
  IrisZoOptions options;
  options.verbose = true;
  options.meshcat = meshcat;
  Hyperellipsoid starting_ellipsoid =
      Hyperellipsoid::MakeHypersphere(1e-2, sample);
  HPolyhedron region = IrisZoFromUrdf(block_urdf, starting_ellipsoid, options);

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 2.0);

  {
    meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}), 0,
                             3.25, -3.25, 3.25);
    meshcat->SetProperty("/Grid", "visible", true);
    Eigen::RowVectorXd thetas = Eigen::RowVectorXd::LinSpaced(100, -M_PI, M_PI);
    const double w = 2, h = 1;
    Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 2 * thetas.size() + 1);
    for (int i = 0; i < thetas.size(); ++i) {
      const double a = 0.5 *
                       (-w * std::sin(thetas[i]) - h * std::cos(thetas[i])),
                   b = 0.5 *
                       (-w * std::sin(thetas[i]) + h * std::cos(thetas[i])),
                   c = 0.5 *
                       (+w * std::sin(thetas[i]) - h * std::cos(thetas[i])),
                   d = 0.5 *
                       (+w * std::sin(thetas[i]) + h * std::cos(thetas[i]));
      points(0, i) = std::max({a, b, c, d});
      points(1, i) = thetas[i];
      points(0, points.cols() - i - 2) = 3.0;
      points(1, points.cols() - i - 2) = thetas[i];
    }
    points.col(points.cols() - 1) = points.col(0);
    meshcat->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    MaybePauseForUser();
  }
}

struct IdentityConstraint {
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 2; }
  template <typename ScalarType>
  void eval(const Eigen::Ref<const VectorX<ScalarType>>& x,
            VectorX<ScalarType>* y) const {
    (*y) = x;
  }
};

// Reproduced from the IrisInConfigurationSpace unit tests.
// A (somewhat contrived) example of a concave configuration-space obstacle
// (resulting in a convex configuration-space, which we approximate with
// polytopes):  A simple pendulum of length `l` with a sphere at the tip of
// radius `r` on a vertical track, plus a ground plane at z=0.  The
// configuration space is given by the joint limits and z + l*cos(theta) >= r.
// The region is also visualized at
// https://www.desmos.com/calculator/flshvay78b. In addition to testing the
// convex space, this was originally a test for which Ibex found
// counter-examples that Snopt missed; now Snopt succeeds due to having
// options.num_collision_infeasible_samples > 1.
GTEST_TEST(IrisZoTest, ConvexConfigurationSpace) {
  const double l = 1.5;
  const double r = 0.1;

  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
  meshcat->Delete("face_pt");
  meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                           -3.25, 3.25, -3.25, 3.25);
  meshcat->SetProperty("/Grid", "visible", true);
  Eigen::RowVectorXd theta1s = Eigen::RowVectorXd::LinSpaced(100, -1.5, 1.5);
  Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 2 * theta1s.size());
  for (int i = 0; i < theta1s.size(); ++i) {
    points(0, i) = r - l * cos(theta1s[i]);
    points(1, i) = theta1s[i];
    points(0, points.cols() - i - 1) = 0;
    points(1, points.cols() - i - 1) = theta1s[i];
  }
  meshcat->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));

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

  const Vector2d sample{-0.5, 0.0};
  IrisZoOptions options;

  // This point should be outside of the configuration space (in collision).
  // The particular value was found by visual inspection using meshcat.
  const double z_test = 0, theta_test = -1.55;
  // Confirm that the pendulum is colliding with the wall with true kinematics:
  EXPECT_LE(z_test + l * std::cos(theta_test), r);

  // Turn on meshcat for addition debugging visualizations.
  // This example is truly adversarial for IRIS. After one iteration, the
  // maximum-volume inscribed ellipse is approximately centered in C-free. So
  // finding a counter-example in the bottom corner (near the test point) is
  // not only difficult because we need to sample in a corner of the polytope,
  // but because the objective is actually pulling the counter-example search
  // away from that corner. Open the meshcat visualization to step through the
  // details!
  options.meshcat = meshcat;
  options.verbose = true;
  Hyperellipsoid starting_ellipsoid =
      Hyperellipsoid::MakeHypersphere(1e-2, sample);
  HPolyhedron region = IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options);
  if (!region.PointInSet(Vector2d{z_test, theta_test})) {
    log()->info("Our test point is not in the set");
  }

  EXPECT_EQ(region.ambient_dimension(), 2);
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 0.5);

  {
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    meshcat->SetObject("Test point", Sphere(0.03), Rgba(1, 0, 0));
    meshcat->SetTransform("Test point", math::RigidTransform(Eigen::Vector3d(
                                            z_test, theta_test, 0)));

    MaybePauseForUser();
  }

  // Another version of the test, adding the additional constraint that
  // x <= -0.3.
  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(2, "q");
  Eigen::RowVectorXd a(2);
  a << 1, 0;
  double lb = -std::numeric_limits<double>::infinity();
  double ub = -0.3;
  prog.AddLinearConstraint(a, lb, ub, q);
  options.prog_with_additional_constraints = &prog;
  options.max_iterations = 1;
  options.max_iterations_separating_planes = 1;
  region = IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options);

  // Due to the configuration space margin, this point can never be in the
  // region.
  Vector2d query_point_not_in_set(-0.29, 0.0);
  Vector2d query_point_in_set(-0.31, 0.0);
  EXPECT_FALSE(region.PointInSet(query_point_not_in_set));
  EXPECT_TRUE(region.PointInSet(query_point_in_set));

  {
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    MaybePauseForUser();
  }

  // We also verify the code path when one of the additional constraints is not
  // threadsafe. We construct the constraint (-2, -0.5) <= (x, y) <= (0, 1.5) in
  // terms of the above struct IdentityConstraint, which is not tagged as
  // threadsafe.
  Eigen::VectorXd simple_constraint_lb = Eigen::Vector2d(-2.0, -0.5);
  Eigen::VectorXd simple_constraint_ub = Eigen::Vector2d(0.0, 1.5);
  std::shared_ptr<solvers::Constraint> simple_constraint =
      std::make_shared<solvers::EvaluatorConstraint<
          solvers::FunctionEvaluator<IdentityConstraint>>>(
          std::make_shared<solvers::FunctionEvaluator<IdentityConstraint>>(
              IdentityConstraint{}),
          simple_constraint_lb, simple_constraint_ub);
  prog.AddConstraint(simple_constraint, q);

  options.max_iterations = 3;
  options.max_iterations_separating_planes = 20;

  region = IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options);
  query_point_not_in_set = Vector2d(-1.0, -0.55);
  query_point_in_set = Vector2d(-1.0, -0.45);
  EXPECT_FALSE(region.PointInSet(query_point_not_in_set));
  EXPECT_TRUE(region.PointInSet(query_point_in_set));

  // If we have a containment point violating this constraint, IrisZo should
  // throw. Three points are needed to ensure the center of the starting
  // ellipsoid is within the convex hull of the points we must contain.
  Eigen::Matrix2Xd single_containment_point(2, 3);
  // clang-format off
  single_containment_point << -0.2, -0.6, -0.6,
                               0.0,  0.1, -0.1;
  // clang-format on
  options.containment_points = single_containment_point;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options),
      ".*containment points violates a constraint.*");
  options.containment_points = std::nullopt;

  // We now test an example of a region grown along a subspace.
  options.set_parameterization(
      [](const Vector1d& config) -> Vector2d {
        return Vector2d{config[0], 2 * config[0] + 1};
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);
  const Vector1d sample2{-0.5};
  starting_ellipsoid = Hyperellipsoid::MakeHypersphere(1e-2, sample2);
  // This domain matches the "x" dimension of C-space, so the region generated
  // will respect the joint limits.
  HPolyhedron domain = HPolyhedron::MakeBox(Vector1d(-1.5), Vector1d(0));

  // Since we have a parameterization, the prog with additional constraints will
  // have the wrong dimension. We expect an error message.
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options, &domain),
      ".*num_vars.*parameterized_dimension.*");

  // Reset the parameterization, and now generate the region.
  options.prog_with_additional_constraints = nullptr;
  region = IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options, &domain);

  EXPECT_EQ(region.ambient_dimension(), 1);
  Vector1d region_query_point_1(-0.75);
  Vector1d region_query_point_2(-0.1);
  EXPECT_TRUE(region.PointInSet(region_query_point_1));
  EXPECT_TRUE(region.PointInSet(region_query_point_2));

  {
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    for (int i = 0; i < vregion.vertices().cols(); ++i) {
      Vector2d point =
          options.get_parameterization()(vregion.vertices().col(i));
      points.col(i).head(2) = point;
      if (i == 0) {
        points.topRightCorner(2, 1) = point;
      }
    }
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    meshcat->SetObject("Test point", Sphere(0.03), Rgba(1, 0, 0));

    Vector2d ambient_query_point =
        options.get_parameterization()(region_query_point_1);
    meshcat->SetTransform(
        "Test point", math::RigidTransform(Eigen::Vector3d(
                          ambient_query_point[0], ambient_query_point[1], 0)));

    MaybePauseForUser();
  }
}
/* A movable sphere with fixed boxes in all corners.
┌───────────────┐
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
│       o       │
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
└───────────────┘ */
const char boxes_in_corners_urdf[] = R"(
<robot name="boxes">
  <link name="fixed">
    <collision name="top_left">
      <origin rpy="0 0 0" xyz="-1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="top_right">
      <origin rpy="0 0 0" xyz="1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_left">
      <origin rpy="0 0 0" xyz="-1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_right">
      <origin rpy="0 0 0" xyz="1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.01"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)";
GTEST_TEST(IrisZoTest, ForceContainmentPointsTest) {
  std::shared_ptr<Meshcat> meshcat;
  meshcat = geometry::GetTestEnvironmentMeshcat();
  meshcat->Delete("face_pt");
  meshcat->Delete("True C_free");
  meshcat->Delete("Test point");
  meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                           -3.25, 3.25, -3.25, 3.25);
  meshcat->SetProperty("/Grid", "visible", true);
  // Draw the true cspace.

  Eigen::Matrix3Xd env_points(3, 5);
  // clang-format off
        env_points << -2, 2,  2, -2, -2,
                        2, 2, -2, -2,  2,
                        0, 0,  0,  0,  0;
  // clang-format on
  meshcat->SetLine("Domain", env_points, 8.0, Rgba(0, 0, 0));
  Eigen::Matrix3Xd centers(3, 4);
  double c = 1.0;
  // clang-format off
        centers << -c, c,  c, -c,
                    c, c, -c, -c,
                    0, 0,  0,  0;
  // clang-format on
  Eigen::Matrix3Xd obs_points(3, 5);
  // Adding 0.01 offset to obstacles to acommodate for the radius of the
  // spherical robot.
  double s = 0.7 + 0.01;
  // clang-format off
        obs_points << -s, s,  s, -s, -s,
                        s, s, -s, -s, s,
                        s, 0,  0,  0,  0;
  // clang-format on
  for (int obstacle_idx = 0; obstacle_idx < 4; ++obstacle_idx) {
    Eigen::Matrix3Xd obstacle = obs_points;
    obstacle.colwise() += centers.col(obstacle_idx);
    meshcat->SetLine(fmt::format("/obstacles/obs_{}", obstacle_idx), obstacle,
                     8.0, Rgba(0, 0, 0));
  }
  const Vector2d sample{0.0, 0.0};
  Hyperellipsoid starting_ellipsoid =
      Hyperellipsoid::MakeHypersphere(1e-2, sample);

  // Test if we can force the containment of a two points. This test is
  // explicitly added to ensure that the procedure works when using fewer points
  // than a simplex are given.
  // First, we verify that IrisZo throws when a single containment point is
  // passed. In this case, the convex hull of the containment points does not
  // containt center of the starting ellipsoid.
  Eigen::Matrix2Xd single_containment_point(2, 1);
  single_containment_point << 0, 1;

  IrisZoOptions options;
  options.verbose = true;
  options.meshcat = meshcat;
  options.configuration_space_margin = 0.04;
  options.containment_points = single_containment_point;

  EXPECT_THROW(
      IrisZoFromUrdf(boxes_in_corners_urdf, starting_ellipsoid, options),
      std::runtime_error);

  // Now we test that it works for a two point VPolytope.
  Eigen::Matrix3Xd single_containment_point_and_ellipsoid_center(3, 2);
  // clang-format off
        single_containment_point_and_ellipsoid_center << 0, sample.x(),
                                                         1, sample.y(),
                                                         0, 0;
  // clang-format on
  options.containment_points =
      single_containment_point_and_ellipsoid_center.topRows(2);
  HPolyhedron region =
      IrisZoFromUrdf(boxes_in_corners_urdf, starting_ellipsoid, options);
  {
    Eigen::Vector3d point_to_draw = Eigen::Vector3d::Zero();
    std::string path = "cont_pt/0";
    options.meshcat->SetObject(path, Sphere(0.04),
                               geometry::Rgba(1, 0, 0.0, 1.0));
    point_to_draw(0) = single_containment_point(0, 0);
    point_to_draw(1) = single_containment_point(1, 0);
    options.meshcat->SetTransform(path,
                                  math::RigidTransform<double>(point_to_draw));
    EXPECT_TRUE(region.PointInSet(single_containment_point.col(0).head(2)));
    Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 20);

    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    MaybePauseForUser();
    meshcat->Delete("face_pt");
    meshcat->Delete(path);
  }

  Eigen::Matrix3Xd cont_points(3, 4);
  double xw, yw;
  xw = 0.4;
  yw = 0.28;
  // clang-format off
        cont_points << -xw, xw,  xw, -xw,
                        yw, yw, -yw, -yw,
                        0,  0,  0,  0;
  // clang-format on
  options.containment_points = cont_points.topRows(2);
  options.max_iterations_separating_planes = 100;
  options.max_iterations = -1;
  region = IrisZoFromUrdf(boxes_in_corners_urdf, starting_ellipsoid, options);
  EXPECT_EQ(region.ambient_dimension(), 2);
  {
    for (int pt_to_draw = 0; pt_to_draw < cont_points.cols(); ++pt_to_draw) {
      Eigen::Vector3d point_to_draw = Eigen::Vector3d::Zero();
      std::string path = fmt::format("cont_pt/{}", pt_to_draw);
      options.meshcat->SetObject(path, Sphere(0.04),
                                 geometry::Rgba(1, 0, 0.0, 1.0));
      point_to_draw(0) = cont_points(0, pt_to_draw);
      point_to_draw(1) = cont_points(1, pt_to_draw);
      options.meshcat->SetTransform(
          path, math::RigidTransform<double>(point_to_draw));
      EXPECT_TRUE(region.PointInSet(point_to_draw.head(2)));
    }
    Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 20);

    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    MaybePauseForUser();
  }
}

}  // namespace
}  // namespace planning
}  // namespace drake
