#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using common::MaybePauseForUser;
using Eigen::Vector2d;
using symbolic::Variable;

const double kInf = std::numeric_limits<double>::infinity();

// Helper method for testing IrisNp from a urdf string.
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
  return IrisNp(plant, plant.GetMyContextFromRoot(*context), options);
}

// One prismatic link with joint limits.  Iris should return the joint limits.
GTEST_TEST(IrisNpTest, JointLimits) {
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

// Two revolute joints, the first of which has limits [-1.0, 1.0], and the
// second of which is without limits. Iris should return the box
// [-1.0, 1.0] x [θ - π/2 + c, θ + π/2 - c], where c is the convexity radius
// stepback.
GTEST_TEST(IrisNpTest, ContinuousRevoluteJoint) {
  const std::string continuous_urdf = R"(
<robot name="limits">
  <link name="movable1">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <link name="movable2">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <axis xyz="1 0 0"/>
    <parent link="world"/>
    <child link="movable1"/>
    <limit lower="-1" upper="1"/>
  </joint>
  <joint name="joint2" type="continuous">
    <axis xyz="1 0 0"/>
    <parent link="movable1"/>
    <child link="movable2"/>
  </joint>
</robot>
)";

  const Vector2d sample = Vector2d::Zero();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(continuous_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 2);
  EXPECT_EQ(region.A().rows(), 4);

  const double kTol = 1e-5;
  double q1min = -1.0;
  double q1max = 1.0;
  EXPECT_TRUE(region.PointInSet(Vector2d{q1min + kTol, 0.0}));
  EXPECT_TRUE(region.PointInSet(Vector2d{q1max - kTol, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{q1min - kTol, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{q1max + kTol, 0.0}));

  double q2min = -M_PI_2 + options.convexity_radius_stepback;
  double q2max = M_PI_2 - options.convexity_radius_stepback;
  EXPECT_TRUE(region.PointInSet(Vector2d{0.0, q2min + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector2d{0.0, q2max - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector2d{0.0, q2min - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector2d{0.0, q2max + kTol}));

  // Negative convexity radius stepback is allowed.
  options.convexity_radius_stepback = -0.25;
  region = IrisFromUrdf(continuous_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 2);
  EXPECT_EQ(region.A().rows(), 4);

  q2min = -M_PI_2 + options.convexity_radius_stepback;
  q2max = M_PI_2 - options.convexity_radius_stepback;
  EXPECT_TRUE(region.PointInSet(Vector2d{0.0, q2min + kTol}));
  EXPECT_TRUE(region.PointInSet(Vector2d{0.0, q2max - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector2d{0.0, q2min - kTol}));
  EXPECT_FALSE(region.PointInSet(Vector2d{0.0, q2max + kTol}));

  // Convexity radius must be strictly less than π/2
  options.convexity_radius_stepback = M_PI_2;
  EXPECT_THROW(IrisFromUrdf(continuous_urdf, sample, options), std::exception);
}

// Check that IRIS correctly handles the continuous revolute component of a
// planar joint.
GTEST_TEST(IrisNpTest, PlanarJoint) {
  const std::string planar_urdf = R"(
<robot name="limits">
  <link name="movable">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="planar">
    <axis xyz="0 0 1"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)";

  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  multibody::Parser parser(&plant);
  parser.package_map().AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/package.xml"));
  parser.AddModelsFromString(planar_urdf, "urdf");

  plant.get_mutable_joint(multibody::JointIndex(0))
      .set_position_limits(Eigen::Vector3d{-1.0, -1.0, -kInf},
                           Eigen::Vector3d{1.0, 1.0, kInf});

  plant.Finalize();
  auto diagram = builder.Build();

  const Eigen::Vector3d sample = Eigen::Vector3d::Zero();
  auto context = diagram->CreateDefaultContext();
  plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()), sample);
  IrisOptions options;
  HPolyhedron region =
      IrisNp(plant, plant.GetMyContextFromRoot(*context), options);

  EXPECT_EQ(region.ambient_dimension(), 3);
  EXPECT_EQ(region.A().rows(), 6);

  const double kTol = 1e-5;
  double qmin = -M_PI_2 + options.convexity_radius_stepback;
  double qmax = M_PI_2 - options.convexity_radius_stepback;
  EXPECT_TRUE(region.PointInSet(Eigen::Vector3d{0.0, 0.0, qmin + kTol}));
  EXPECT_TRUE(region.PointInSet(Eigen::Vector3d{0.0, 0.0, qmax - kTol}));
  EXPECT_FALSE(region.PointInSet(Eigen::Vector3d{0.0, 0.0, qmin - kTol}));
  EXPECT_FALSE(region.PointInSet(Eigen::Vector3d{0.0, 0.0, qmax + kTol}));

  // If we don't set the position limits, then it should throw, since
  // the prismatic components of the planar joint are unbounded.
  DRAKE_EXPECT_THROWS_MESSAGE(IrisFromUrdf(planar_urdf, sample, options),
                              ".*position limits.*");

  // If we add an initial bounding region that bounds the planar degrees of
  // freedom, it shouldn't error.
  Eigen::MatrixXd A(4, 3);
  Eigen::VectorXd b(4);
  // clang-format off
  A <<  1,  0, 0,
       -1,  0, 0,
        0,  1, 0,
        0, -1, 0;
  // clang-format on
  b << 1, 1, 1, 1;
  options.bounding_region = HPolyhedron{A, b};
  EXPECT_NO_THROW(IrisFromUrdf(planar_urdf, sample, options));

  // It still shouldn't error if we disable the boundedness check.
  options.verify_domain_boundedness = false;
  EXPECT_NO_THROW(IrisFromUrdf(planar_urdf, sample, options));

  // If the initial bounding region doesn't actually bound the planar degrees of
  // freedom, it should error.
  Eigen::MatrixXd A2(1, 3);
  Eigen::VectorXd b2(1);
  A2 << 1, 0, 0;
  b2 << 1;
  options.bounding_region = HPolyhedron{A2, b2};
  options.verify_domain_boundedness = true;
  DRAKE_EXPECT_THROWS_MESSAGE(IrisFromUrdf(planar_urdf, sample, options),
                              ".*position limits.*");
}

// Check that IRIS correctly handles the continuous revolute component(s) of a
// roll-pitch-yaw floating joint.
GTEST_TEST(IrisNpTest, RpyFloatingJoint) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  const multibody::RigidBody<double>& body = plant.AddRigidBody("body");
  plant.AddJoint<multibody::RpyFloatingJoint>("joint", plant.world_body(), {},
                                              body, {});

  plant.get_mutable_joint(multibody::JointIndex(0))
      .set_position_limits(Vector6d(-kInf, -kInf, -kInf, -1.0, -1.0, -1.0),
                           Vector6d(kInf, kInf, kInf, 1.0, 1.0, 1.0));

  plant.Finalize();
  auto diagram = builder.Build();

  const Vector6d sample = Vector6d::Zero();
  auto context = diagram->CreateDefaultContext();
  plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()), sample);
  IrisOptions options;
  HPolyhedron region =
      IrisNp(plant, plant.GetMyContextFromRoot(*context), options);

  EXPECT_EQ(region.ambient_dimension(), 6);
  EXPECT_EQ(region.A().rows(), 12);

  const double kTol = 1e-5;
  double qmax = M_PI_2 - options.convexity_radius_stepback;
  for (const int i : std::vector<int>{-1, 1}) {
    for (const int j : std::vector<int>{-1, 1}) {
      for (const int k : std::vector<int>{-1, 1}) {
        Vector6d point_out(i * (qmax + kTol), j * (qmax + kTol),
                           k * (qmax + kTol), 0.0, 0.0, 0.0);
        Vector6d point_in(i * (qmax - kTol), j * (qmax - kTol),
                          k * (qmax - kTol), 0.0, 0.0, 0.0);
        EXPECT_TRUE(region.PointInSet(point_in));
        EXPECT_FALSE(region.PointInSet(point_out));
      }
    }
  }

  systems::DiagramBuilder<double> builder2;
  multibody::MultibodyPlant<double>& plant2 =
      multibody::AddMultibodyPlantSceneGraph(&builder2, 0.0);
  const multibody::RigidBody<double>& body2 = plant2.AddRigidBody("body");
  plant2.AddJoint<multibody::RpyFloatingJoint>("joint", plant2.world_body(), {},
                                               body2, {});

  plant2.Finalize();
  auto diagram2 = builder2.Build();

  // If we don't set the position limits, then it should throw, since
  // the prismatic components of the planar joint are unbounded.
  auto context2 = diagram2->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp(plant2, plant2.GetMyContextFromRoot(*context2), options),
      ".*position limits.*");
}

// Check that IRIS throws an intuitive error message if the user supplies a
// plant with a QuaternionFloatingJoint.
GTEST_TEST(IrisNpTest, QuaternionFloatingJoint) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  const multibody::RigidBody<double>& body = plant.AddRigidBody("body");
  plant.AddJoint<multibody::QuaternionFloatingJoint>(
      "joint", plant.world_body(), {}, body, {});

  plant.Finalize();
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  IrisOptions options;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp(plant, plant.GetMyContextFromRoot(*context), options),
      ".*not support QuaternionFloatingJoint.*");
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
GTEST_TEST(IrisNpTest, BoxesPrismatic) {
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

// Three boxes again, but the configuration-space margin is larger than 1/2 the
// gap.
GTEST_TEST(IrisNpTest, ConfigurationSpaceMargin) {
  const Vector1d sample = Vector1d::Zero();
  IrisOptions options;
  options.configuration_space_margin = 1.5;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisFromUrdf(boxes_urdf, sample, options),
      ".*is within options.configuration_space_margin of being infeasible.*");
}

const char boxes_with_mesh_urdf[] = R"""(
<robot xmlns:drake="http://drake.mit.edu" name="boxes">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="2.5 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-3 0 0"/>
      <geometry>
        <!-- This mesh is equivalent to <box size="2 2 2"/> -->
        <!-- Note: not declared convex -->
        <mesh filename="package://box_model/meshes/box.obj"/>
      </geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="center">
      <geometry>
        <!-- This mesh is equivalent to <box size="2 2 2"/> -->
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
// This also tests mesh geometry (both Convex and Mesh).
GTEST_TEST(IrisNpTest, BoxesWithMeshPrismatic) {
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

GTEST_TEST(IrisNpTest, ConfigurationObstacles) {
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

    // Test solver options
    // First change the counterexample solver options. We should get a different
    // IRIS region.
    options.solver_options.emplace(solvers::SolverOptions());
    options.solver_options->SetOption(solvers::IpoptSolver::id(), "max_iter",
                                      0);
    options.solver_options->SetOption(solvers::SnoptSolver::id(),
                                      "Major Iterations Limit", 0);
    const HPolyhedron region_no_counterexample =
        IrisFromUrdf(boxes_urdf, sample, options);
    EXPECT_FALSE(CompareMatrices(region.A(), region_no_counterexample.A()));
    options.solver_options = std::nullopt;
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

/* Box in 2D. A box is free to translate in x and y, and has joint limits
restricting the range in y and collision geometries restricting the range in x.
*/
const char boxes_in_2d_urdf[] = R"""(
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
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)""";

GTEST_TEST(IrisNpTest, ConfigurationObstaclesMultipleBoxes) {
  IrisOptions options;
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.1, .5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-.1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, .5), Vector2d(-.1, 1)));
  options.configuration_obstacles = obstacles;

  const Vector2d sample{0.8, 0};  // right corridor.
  HPolyhedron region = IrisFromUrdf(boxes_in_2d_urdf, sample, options);

  // The region will stretch in x.
  EXPECT_TRUE(region.PointInSet(Vector2d(.9, 0.0)));
  EXPECT_TRUE(region.PointInSet(Vector2d(-.9, 0.0)));
  EXPECT_FALSE(region.PointInSet(Vector2d(0.0, .9)));
  EXPECT_FALSE(region.PointInSet(Vector2d(0.0, -.9)));
}

/* Test if the starting ellipse is far away from the seed point, and the seed
point exits the polytope before the computations are over, then an error will be
thrown. */
GTEST_TEST(IrisNpTest, BadEllipseAndSample) {
  IrisOptions options;
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(0, 0), Vector2d(1, 1)));
  options.configuration_obstacles = obstacles;
  const Vector2d sample{-0.5, 0.5};
  const Vector2d ellipse_center{0.8, -0.2};  // ellipse includes collision
  Hyperellipsoid starting_ellipse =
      Hyperellipsoid::MakeHypersphere(0.1, ellipse_center);
  options.starting_ellipse = starting_ellipse;
  options.require_sample_point_is_contained = true;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisFromUrdf(boxes_in_2d_urdf, sample, options),
      ".*starting_ellipse not contain the seed point.*");
}

/* Same as boxes_in_2d_urdf, but without the first two collision geos.
The point is to have faster iris computations in the following test.
*/
const char boxes_in_2d_urdf_no_collisions[] = R"""(
<robot name="boxes">
  <link name="movable">
    <collision name="center">
      <geometry><box size="1 1 1"/></geometry>
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
    <limit lower="-1" upper="1"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)""";
/* Make termination function such that the region will contain q1, q2.
This is meant to generate Iris regions from edges.
*/
GTEST_TEST(IrisNpTest, TerminationFunc) {
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.1, .5), Vector2d(1, 1)));
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(-1, -1), Vector2d(-.1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(.1, -1), Vector2d(1, -.5)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(-1, .5), Vector2d(-.1, 1)));
  const Vector2d sample{0, 0};  // center of the bounding box.
  std::function<bool(const HPolyhedron&)> always_false =
      [&](const HPolyhedron&) {
        return false;
      };
  IrisOptions options;
  options.iteration_limit = 100;
  options.termination_threshold = -1;
  options.configuration_obstacles = obstacles;
  options.random_seed = 0;
  HPolyhedron without_termination =
      IrisFromUrdf(boxes_in_2d_urdf_no_collisions, sample, options);
  options.termination_func = always_false;
  HPolyhedron with_always_false =
      IrisFromUrdf(boxes_in_2d_urdf_no_collisions, sample, options);
  // Region with always false termination function should be the same as region
  // without the termination function
  EXPECT_TRUE(with_always_false.ContainedIn(without_termination, 1e-6));
  EXPECT_TRUE(without_termination.ContainedIn(with_always_false, 1e-6));
  // now we add a termination function that will make the region contain q1, q2
  const Vector2d q1{0.15, -0.45};
  const Vector2d q2{-0.05, 0.75};
  for (const auto& obstacle : obstacles) {
    EXPECT_FALSE(obstacle->PointInSet(q1));
    EXPECT_FALSE(obstacle->PointInSet(q2));
  }
  SetEdgeContainmentTerminationCondition(&options, q1, q2, 1e-3);
  // Test that the termination function works with constraints as well
  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(2, "q");
  prog.AddConstraint(q[0], -1, 0.16);
  options.prog_with_additional_constraints = &prog;
  HPolyhedron region_with_termination =
      IrisFromUrdf(boxes_in_2d_urdf_no_collisions, sample, options);
  EXPECT_TRUE(region_with_termination.PointInSet(q1));
  EXPECT_TRUE(region_with_termination.PointInSet(q2));
  // failure case when the provided edge is in collision
  const Vector2d q3{-0.85, 0.75};
  SetEdgeContainmentTerminationCondition(&options, q1, q3, 1e-3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisFromUrdf(boxes_in_2d_urdf_no_collisions, sample, options),
      ".*the termination function returned false on the computation of the "
      "initial region.*");
  // failure case when the provided edge is out of the domain
  const Vector2d q4{-0.85, 1.75};
  SetEdgeContainmentTerminationCondition(&options, q1, q4, 1e-3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisFromUrdf(boxes_in_2d_urdf_no_collisions, sample, options),
      ".*Please check the implementation of your termination_func.");
}

/* Box obstacles in one corner.
┌───────┬─────┐
│       │     │
│       │     │
│       └─────┤
│      *      │
│             │
│             │
│             │
└─────────────┘
We use only a single configuration obstacle, and verify the the computed
halfspace changes.
*/
GTEST_TEST(IrisNpTest, StartingEllipse) {
  const Vector2d sample{0.0, 0.0};
  IrisOptions options;
  options.iteration_limit = 1;
  options.num_collision_infeasible_samples = 0;
  ConvexSets obstacles;
  obstacles.emplace_back(VPolytope::MakeBox(Vector2d(.2, .2), Vector2d(1, 1)));
  options.configuration_obstacles = obstacles;
  HPolyhedron region = IrisFromUrdf(boxes_in_2d_urdf, sample, options);

  Eigen::Matrix2d A;
  A << 10, 0, 0, 1;
  options.starting_ellipse = Hyperellipsoid(A, sample);
  HPolyhedron region_w_ellipse =
      IrisFromUrdf(boxes_in_2d_urdf, sample, options);

  // Regions should have only one additional half space beyond the joint limits.
  EXPECT_EQ(region.b().size(), 5);
  EXPECT_EQ(region_w_ellipse.b().size(), 5);

  // last row of A is a scaling of [1, 1].
  EXPECT_NEAR(region.A()(4, 0), region.A()(4, 1), 1e-6);
  // last row of A is a scaling of [100, 1].
  EXPECT_NEAR(region_w_ellipse.A()(4, 0), 100 * region_w_ellipse.A()(4, 1),
              1e-6);
}

GTEST_TEST(IrisNpTest, BoundingRegion) {
  const Vector2d sample{0.0, 0.0};
  IrisOptions options;
  options.iteration_limit = 1;
  options.num_collision_infeasible_samples = 0;
  ConvexSets obstacles;
  obstacles.emplace_back(
      VPolytope::MakeBox(Vector2d(0.2, 0.2), Vector2d(1, 1)));
  options.configuration_obstacles = obstacles;
  HPolyhedron region = IrisFromUrdf(boxes_in_2d_urdf, sample, options);

  // Add a bounding region that halves the plant's joint limits.
  options.bounding_region =
      HPolyhedron::MakeBox(Vector2d(-1, -0.5), Vector2d(1, 0.5));
  HPolyhedron region_w_bounding =
      IrisFromUrdf(boxes_in_2d_urdf, sample, options);

  // `region` should have only one additional half space beyond the initial
  // polytope. `region_w_bounding` should have a further four half spaces since
  // its initial polytope will have been intersected with
  // `options.bounding_region`.
  EXPECT_EQ(region.b().size(), 5);
  EXPECT_EQ(region_w_bounding.b().size(), 9);

  // The point (-1.5, -0.5) is within the plant's joint limits but outside the
  // bounding region. It should be contained in region but not in
  // region_w_bounding.
  EXPECT_TRUE(region.PointInSet(Vector2d(-1.5, -0.5)));
  EXPECT_FALSE(region_w_bounding.PointInSet(Vector2d(-1.5, -0.5)));

  // A point closer to the origin should be in both regions.
  EXPECT_TRUE(region.PointInSet(Vector2d(-0.5, -0.25)));
  EXPECT_TRUE(region_w_bounding.PointInSet(Vector2d(-0.5, -0.25)));
}

// Three spheres.  Two on the outside are fixed.  One in the middle on a
// prismatic joint.  The configuration space is a (convex) line segment q ∈
// (−1,1).
GTEST_TEST(IrisNpTest, SpheresPrismatic) {
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
GTEST_TEST(IrisNpTest, SinglePendulum) {
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
GTEST_TEST(IrisNpTest, DoublePendulum) {
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
    std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
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

// A block on a vertical track, free to rotate (in the plane) with width `w` of
// 2 and height `h` of 1, plus a ground plane at z=0.  The true configuration
// space is min(q₀ ± .5w sin(q₁) ± .5h cos(q₁)) ≥ 0, where the min is over the
// ±. This region is also visualized at
// https://www.desmos.com/calculator/ok5ckpa1kp.
GTEST_TEST(IrisNpTest, BlockOnGround) {
  const Vector2d sample{1.0, 0.0};
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(block_urdf, sample, options);

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 2.0);

  {
    std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
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
GTEST_TEST(IrisNpTest, ConvexConfigurationSpace) {
  const double l = 1.5;
  const double r = 0.1;

  std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
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
  IrisOptions options;

  // This point should be outside of the configuration space (in collision).
  // The particular value was found by visual inspection using meshcat.
  const double z_test = 0, theta_test = -1.55;
  // Confirm that the pendulum is colliding with the wall with true kinematics:
  EXPECT_LE(z_test + l * std::cos(theta_test), r);

  // With num_collision_infeasible_samples == 1, we found that SNOPT misses this
  // point (on some platforms with some random seeds).
  options.num_collision_infeasible_samples = 3;

  // Turn on meshcat for addition debugging visualizations.
  // This example is truly adversarial for IRIS. After one iteration, the
  // maximum-volume inscribed ellipse is approximately centered in C-free. So
  // finding a counter-example in the bottom corner (near the test point) is
  // not only difficult because we need to sample in a corner of the polytope,
  // but because the objective is actually pulling the counter-example search
  // away from that corner. Open the meshcat visualization to step through the
  // details!
  options.meshcat = meshcat;

  HPolyhedron region = IrisFromUrdf(convex_urdf, sample, options);

  // TODO(russt): Expecting the test point to be outside the verified region is
  // too strong of a requirement right now. If we can improve the algorithm then
  // we should make this EXPECT_FALSE.
  if (!region.PointInSet(Vector2d{z_test, theta_test})) {
    log()->info("Our test point is not in the set");
  }

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
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

  // Confirm that mixing_steps has a tangible effect (this example is
  // particularly sensitive to the sampling), and obtains a smaller volume due
  // to non-uniform sampling with less mixing_steps.
  options.mixing_steps = 1;  // Smaller than the default.
  HPolyhedron region2 = IrisFromUrdf(convex_urdf, sample, options);
  constexpr double kTol = 1e-4;
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume() + kTol,
            region2.MaximumVolumeInscribedEllipsoid().Volume());
}

// Three boxes.  Two on the outside are fixed.  One in the middle on a prismatic
// joint.  Additional constraints are added via
// options.prog_with_additional_constraints:
//    q <= 0.5
//    sin(q) >= -1/sqrt(2)
// The (constrained) configuration space is a (convex) line segment
// q ∈ (−pi/4, .5).
GTEST_TEST(IrisNpTest, BoxesPrismaticPlusConstraints) {
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

  // Add an upper bound constraint on q
  const double q_ub = M_PI / 8;
  DRAKE_ASSERT(q_ub < qmax);  // otherwise test will be pointless
  prog.AddConstraint(sin(q[0]), -1.0 / sqrt(2.0), sin(q_ub));
  // Calc the upper bounded region
  HPolyhedron region_ub = IrisFromUrdf(boxes_urdf, sample, options);
  const double kMargin = options.configuration_space_margin;
  EXPECT_TRUE(region_ub.PointInSet(Vector1d{q_ub - kTol - kMargin}));
  EXPECT_FALSE(region_ub.PointInSet(Vector1d{q_ub + kTol - kMargin}));
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
GTEST_TEST(IrisNpTest, DoublePendulumEndEffectorConstraints) {
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
  // We required > 10 samples to pass the test on mac CI with ipopt.
  options.num_additional_constraint_infeasible_samples = 15;

  HPolyhedron region =
      IrisNp(plant, plant.GetMyContextFromRoot(*context), options);

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
      points(1, i) = std::acos(1 - std::cos(theta1s[i])) - theta1s[i];
      points(0, points.cols() - i - 1) = theta1s[i];
      points(1, points.cols() - i - 1) =
          -std::acos(1 - std::cos(theta1s[i])) - theta1s[i];
    }
    meshcat->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
    VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
    points.resize(3, vregion.vertices().cols() + 1);
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

    MaybePauseForUser();
  }
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
