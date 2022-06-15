#include <gtest/gtest.h>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/ibex_solver.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Vector2d;

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
  if (!solvers::IbexSolver::is_available() ||
      !solvers::IbexSolver::is_enabled()) {
    // This test requires Ibex.
    return;
  }

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
