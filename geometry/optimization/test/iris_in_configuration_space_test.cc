#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Vector2d;
using symbolic::Variable;

const double kInf = std::numeric_limits<double>::infinity();

// Helper method for testing IrisInConfigurationSpace from a urdf string.
HPolyhedron IrisFromUrdf(const std::string urdf,
                         const Eigen::Ref<const Eigen::VectorXd>& sample,
                         const IrisOptions& options) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  multibody::Parser parser(&plant);
  parser.package_map().AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/package.xml"));
  parser.AddModelsFromString(urdf, "urdf");
  plant.Finalize();
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()), sample);
  return IrisInConfigurationSpace(plant, plant.GetMyContextFromRoot(*context),
                                  options);
}

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

const char boxes_urdf[] = R"""(
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
)""";

// Three boxes.  Two on the outside are fixed.  One in the middle on a prismatic
// joint.  The configuration space is a (convex) line segment q ∈ (−1,1).
GTEST_TEST(IrisInConfigurationSpaceTest, BoxesPrismatic) {
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

  DRAKE_EXPECT_THROWS_MESSAGE(IrisFromUrdf(boxes_urdf, Vector1d{1.1}, options),
                              "The seed point is in collision.*");
}

const char boxes_with_mesh_urdf[] = R"""(
<robot name="boxes">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="2.5 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-2.5 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="center">
      <!-- box size="2 2 2" -->
      <geometry>
        <mesh filename="package://box_model/meshes/box.obj">
          <drake:declare_convex/>
        </mesh>
      </geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)""";

// Three boxes.  Two on the outside are fixed.  One in the middle on a prismatic
// joint.  The configuration space is a (convex) line segment q ∈ (−1,1).
GTEST_TEST(IrisInConfigurationSpaceTest, BoxesWithMeshPrismatic) {
  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(boxes_with_mesh_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-3;  // due to ibex's rel_eps_f.
  const double qmin = -1.0 + options.configuration_space_margin,
               qmax = 1.0 - options.configuration_space_margin;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));

  DRAKE_EXPECT_THROWS_MESSAGE(IrisFromUrdf(boxes_urdf, Vector1d{1.1}, options),
                              "The seed point is in collision.*");
}

GTEST_TEST(IrisInConfigurationSpaceTest, ConfigurationObstacles) {
  const double kTol = 1e-3;  // due to ibex's rel_eps_f.
  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;

  // Configuration space obstacles less restrictive than task space obstacle.
  {
    ConvexSets obstacles;
    obstacles.emplace_back(HPolyhedron::MakeBox(Vector1d(1.5), Vector1d(3)));
    options.configuration_obstacles = obstacles;
    HPolyhedron region = IrisFromUrdf(boxes_urdf, sample, options);

    EXPECT_EQ(region.ambient_dimension(), 1);

    const double qmin = -1.0 + options.configuration_space_margin,
                 qmax = 1.0 - options.configuration_space_margin;
    EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
    EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
    EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
    EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
  }

  // Configuration space obstacle more restrictive than task space obstacles.
  {
    ConvexSets obstacles;
    obstacles.emplace_back(HPolyhedron::MakeBox(Vector1d(0.5), Vector1d(3)));
    options.configuration_obstacles = obstacles;
    HPolyhedron region = IrisFromUrdf(boxes_urdf, sample, options);

    EXPECT_EQ(region.ambient_dimension(), 1);

    const double qmin = -1.0 + options.configuration_space_margin, qmax = 0.5;
    EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
    EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
    EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
    EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
  }

  // Configuration space obstacles align with task space obstacle.
  {
    ConvexSets obstacles;
    obstacles.emplace_back(HPolyhedron::MakeBox(Vector1d(1), Vector1d(3)));
    options.configuration_obstacles = obstacles;
    HPolyhedron region = IrisFromUrdf(boxes_urdf, sample, options);

    EXPECT_EQ(region.ambient_dimension(), 1);

    const double qmin = -1.0 + options.configuration_space_margin,
                 qmax = 1.0 - options.configuration_space_margin;
    EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
    EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
    EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
    EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
  }

  // Sample is in configuration space obstacles.
  {
    ConvexSets obstacles;
    obstacles.emplace_back(HPolyhedron::MakeBox(Vector1d(0.5), Vector1d(3)));
    options.configuration_obstacles = obstacles;
    DRAKE_EXPECT_THROWS_MESSAGE(
        IrisFromUrdf(boxes_urdf, Vector1d(0.7), options),
        "The seed point is in configuration obstacle.*");
  }
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
// The region is visualized at https://www.desmos.com/calculator/flshvay78b. In
// addition to testing the convex space, this was originally a test for which
// Ibex found counter-examples that Snopt missed; now Snopt succeeds due to
// having options.num_collision_infeasible_samples > 1.
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

  const Vector2d sample{-0.5, 0.0};
  IrisOptions options;

  // This point should be outside of the configuration space (in collision).
  // The particular value was found by visual inspection using the desmos plot.
  const double z_test = 0, theta_test = -1.55;
  // Confirm that the pendulum is colliding with the wall with true kinematics:
  EXPECT_LE(z_test + l*std::cos(theta_test), r);

  // With num_collision_infeasible_samples == 1, we found that SNOPT misses this
  // point (on some platforms with some random seeds).

  options.num_collision_infeasible_samples = 5;
  HPolyhedron region = IrisFromUrdf(convex_urdf, sample, options);
  EXPECT_FALSE(region.PointInSet(Vector2d{z_test, theta_test}));

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 0.5);

  // Note: You may use this to plot the solution in the desmos graphing
  // calculator link above.  Just copy each equation in the printed formula into
  // a desmos cell.  The intersection is the computed region.
  // const Vector2<symbolic::Expression> xy{symbolic::Variable("x"),
  //                                        symbolic::Variable("y")};
  // std::cout << (region.A()*xy <= region.b()) << std::endl;

  // TODO(russt): Drop desmos and draw these in meshcat (as I did in the
  // DoublePendulumEndEffectorConstraints test below).
}

// Three boxes.  Two on the outside are fixed.  One in the middle on a prismatic
// joint.  Additional constraints are added via
// options.prog_with_additional_constraints:
//    q <= 0.5
//    sin(q) >= -1/sqrt(2)
// The (constrained) configuration space is a (convex) line segment
// q ∈ (−pi/4, .5).
GTEST_TEST(IrisInConfigurationSpaceTest, BoxesPrismaticPlusConstraints) {
  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;

  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables<1>("q");
  prog.AddBoundingBoxConstraint(-kInf, 0.5, q[0]);
  prog.AddConstraint(sin(q[0]), -1.0 / sqrt(2.0), kInf);
  options.prog_with_additional_constraints = &prog;

  HPolyhedron region = IrisFromUrdf(boxes_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 1);

  const double kTol = 1e-5;
  const double qmin = -M_PI / 4.0 + options.configuration_space_margin,
               qmax = 0.5;  // Note: no margin here because linear constraints
                            // are handled directly.
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + kTol}));
}

// This double pendulum doesn't have any collision geometry, but we'll add
// this end-effector position constraint via an InverseKinematics problem:
//    [-inf, -inf, -inf] <= p_WE <= [inf, inf, -1]
// Leading the configuration space (since both links are length 1):
//   cos(θ₁) + cos(θ₁ + θ₂) ≥ 1.
//
// Note that the zero lower and upper bounds in the y positions are included to
// test that equality constraints that still result in a configuration space
// region with an interior are supported.
GTEST_TEST(IrisInConfigurationSpaceTest, DoublePendulumEndEffectorConstraints) {
  const std::string double_pendulum_urdf = R"(
<robot name="double_pendulum">
  <link name="base"/>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link2"/>
  <joint name="joint2" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)";

  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  multibody::Parser(&plant).AddModelsFromString(double_pendulum_urdf, "urdf");
  plant.Finalize();
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(context.get());
  // Choose an initial sample that is near zero (but not exactly zero because
  // SNOPT fails to break the symmetry in the counter-example search).
  plant.SetPositions(&plant_context, Eigen::Vector2d(0.01, 0.01));

  multibody::InverseKinematics ik(plant, false);
  // Note: It is tempting to set the lower and upper bounds on the y-axis to
  // zero (after all, the point is always in the y=0 plane for all q). This does
  // work, but the numerics of the counter-example search are much worse and
  // the resulting IRIS region is much smaller because of it.
  ik.AddPositionConstraint(plant.GetFrameByName("link2"),
                           Eigen::Vector3d(0, 0, -1), plant.world_frame(),
                           Eigen::Vector3d(-kInf, -kInf, -kInf),
                           Eigen::Vector3d(kInf, kInf, -1));

  IrisOptions options;
  options.prog_with_additional_constraints = &ik.prog();
  options.num_additional_constraint_infeasible_samples = 10;

  HPolyhedron region = IrisInConfigurationSpace(
      plant, plant.GetMyContextFromRoot(*context), options);

  EXPECT_EQ(region.ambient_dimension(), 2);

  const double theta1 = 0.1;
  const double theta2_max = std::acos(1 - std::cos(theta1)) - theta1 -
                            options.configuration_space_margin;
  const double theta2_min = -std::acos(1 - std::cos(theta1)) - theta1 +
                            options.configuration_space_margin;

  // These tolerances are necessarily loose because we are approximating a
  // non-convex configuration space region with a polytope.
  const double kInnerTol = 0.1;
  const double kOuterTol = 0.1;
  EXPECT_TRUE(
      region.PointInSet(Eigen::Vector2d{theta1, theta2_min + kInnerTol}));
  EXPECT_TRUE(
      region.PointInSet(Eigen::Vector2d{theta1, theta2_max - kInnerTol}));
  EXPECT_FALSE(
      region.PointInSet(Eigen::Vector2d{theta1, theta2_min - kOuterTol}));
  EXPECT_FALSE(
      region.PointInSet(Eigen::Vector2d{theta1, theta2_max + kOuterTol}));

  {
    std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
    meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                            -3.25, 3.25, -3.25, 3.25);
    meshcat->SetProperty("/Grid", "visible", true);
    Eigen::RowVectorXd theta1s = Eigen::RowVectorXd::LinSpaced(100, -1.6, 1.6);
    Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 2 * theta1s.size());
    for (int i = 0; i < theta1s.size(); ++i) {
      points(0, i) = theta1s[i];
      points(1, i) =  std::acos(1 - std::cos(theta1s[i])) - theta1s[i];
      points(0, points.cols() - i - 1) = theta1s[i];
      points(1, points.cols() - i - 1) =
          -std::acos(1 - std::cos(theta1s[i])) - theta1s[i];
    }
    meshcat->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols()+1);
    points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
    points.topRightCorner(2, 1) = vregion.vertices().col(0);
    points.bottomRows<1>().setZero();
    meshcat->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

    meshcat->SetObject("Test point in (min)", Sphere(0.03), Rgba(0, 1, 0));
    meshcat->SetTransform("Test point in (min)",
                          math::RigidTransform(Eigen::Vector3d(
                              theta1, theta2_min + kInnerTol, 0)));
    meshcat->SetObject("Test point in (max)", Sphere(0.03), Rgba(0, 1, 0));
    meshcat->SetTransform("Test point in (max)",
                          math::RigidTransform(Eigen::Vector3d(
                              theta1, theta2_max - kInnerTol, 0)));
    meshcat->SetObject("Test point out (min)", Sphere(0.03), Rgba(1, 0, 0));
    meshcat->SetTransform("Test point out (min)",
                          math::RigidTransform(Eigen::Vector3d(
                              theta1, theta2_min - kOuterTol, 0)));
    meshcat->SetObject("Test point out (max)", Sphere(0.03), Rgba(1, 0, 0));
    meshcat->SetTransform("Test point out (max)",
                          math::RigidTransform(Eigen::Vector3d(
                              theta1, theta2_max + kOuterTol, 0)));

    // Note: This will not pause execution when running as a bazel test.
    std::cout << "[Press RETURN to continue]." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
