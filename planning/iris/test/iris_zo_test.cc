#include "drake/planning/iris/iris_zo.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/planning/iris/test/iris_test_utilities.h"
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
TEST_F(JointLimits1D, JointLimitsBasic) {
  IrisZoOptions options;
  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

// In this test, we reconstruct the default identity parameterization in three
// different ways, to verify that all three methods work and produce the same
// results.
TEST_F(JointLimits1D, JointLimitsWithParameterization) {
  std::vector<IrisZoOptions> vector_of_options;
  vector_of_options.emplace_back();
  vector_of_options.back().set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return q;
      },
      /* parameterization_is_threadsafe */ false,
      /* parameterization_dimension */ 1);
  EXPECT_EQ(vector_of_options.back().get_parameterization_is_threadsafe(),
            false);

  // Now set the parameterization with parameterization_is_threadsafe set to
  // true.
  vector_of_options.emplace_back();
  vector_of_options.back().set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return q;
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);
  EXPECT_EQ(vector_of_options[1].get_parameterization_is_threadsafe(), true);

  // Now use an Expression parameterization.
  Eigen::VectorX<symbolic::Variable> variables(1);
  variables[0] = symbolic::Variable("q");
  Eigen::VectorX<symbolic::Expression> parameterization_expression(1);
  parameterization_expression[0] = symbolic::Expression(variables[0]);
  vector_of_options.emplace_back();
  vector_of_options.back().SetParameterizationFromExpression(
      parameterization_expression, variables);
  // Expression parameterizations are always threadsafe.
  EXPECT_EQ(vector_of_options.back().get_parameterization_is_threadsafe(),
            true);

  for (auto& options : vector_of_options) {
    options.verbose = true;

    // Check that the parameterization was set correctly.
    ASSERT_TRUE(options.get_parameterization_dimension().has_value());
    EXPECT_EQ(options.get_parameterization_dimension().value(), 1);
    const Vector1d output = options.get_parameterization()(Vector1d(3.0));
    EXPECT_NEAR(output[0], 3.0, 1e-15);

    HPolyhedron region =
        IrisZo(*checker_, starting_ellipsoid_, domain_, options);

    CheckRegion(region);
  }
}

// Now we test two cases of an Expression parameterization where an error should
// be thrown.
TEST_F(JointLimits1D, ParameterizationExpressionErrorChecks) {
  IrisZoOptions options;
  Eigen::VectorX<symbolic::Variable> variables(1);
  variables[0] = symbolic::Variable("q");

  // If the output dimension doesn't match the configuration space dimension, an
  // error will be thrown when IrisZo is called.
  Eigen::VectorX<symbolic::Expression>
      parameterization_expression_wrong_dimension(2);
  parameterization_expression_wrong_dimension[0] =
      symbolic::Expression(variables[0]);
  parameterization_expression_wrong_dimension[1] =
      symbolic::Expression(variables[0]);
  options.SetParameterizationFromExpression(
      parameterization_expression_wrong_dimension, variables);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*parameterization returned a point with the wrong dimension.*");

  // If the variables used in the parameterization don't match the variables
  // given in the second argument, an error is thrown immediately.
  symbolic::Variable extra_variable("oops");
  Eigen::VectorX<symbolic::Expression>
      parameterization_expression_extra_variable(1);
  parameterization_expression_extra_variable[0] = variables[0] + extra_variable;
  EXPECT_THROW(options.SetParameterizationFromExpression(
                   parameterization_expression_extra_variable, variables),
               std::exception);
  Eigen::VectorX<symbolic::Expression>
      parameterization_expression_missing_variable(1);
  parameterization_expression_missing_variable[0] = symbolic::Expression(1);
  EXPECT_THROW(options.SetParameterizationFromExpression(
                   parameterization_expression_missing_variable, variables),
               std::exception);
}

// Reproduced from the IrisInConfigurationSpace unit tests.
TEST_F(DoublePendulum, IrisZoTest) {
  IrisZoOptions options;
  options.verbose = true;

  meshcat_->Delete();
  options.meshcat = meshcat_;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);

  PlotEnvironmentAndRegion(region);
}

// Test growing a region for the double pendulum along a parameterization of the
// configuration space built from RationalForwardKinematics.
TEST_F(DoublePendulumRationalForwardKinematics,
       ParameterizationFromStaticConstructor) {
  IrisZoOptions options =
      IrisZoOptions::CreateWithRationalKinematicParameterization(
          &rational_kinematics_,
          /* q_star_val */ Vector2d::Zero());
  options.verbose = true;

  meshcat_->Delete();
  options.meshcat = meshcat_;

  // Check that the parameterization was set correctly.
  EXPECT_EQ(options.get_parameterization_is_threadsafe(), true);
  ASSERT_TRUE(options.get_parameterization_dimension().has_value());
  EXPECT_EQ(options.get_parameterization_dimension().value(), 2);

  CheckParameterization(options.get_parameterization());

  HPolyhedron region =
      IrisZo(*checker_, starting_ellipsoid_rational_forward_kinematics_,
             domain_rational_forward_kinematics_, options);

  CheckRegionRationalForwardKinematics(region);
  PlotEnvironmentAndRegionRationalForwardKinematics(
      region, options.get_parameterization(), region_query_point_1_);
}

// Verify that we can get the same behavior by using an Expression
// parameterization.
TEST_F(DoublePendulumRationalForwardKinematics,
       ParameterizationFromExpression) {
  IrisZoOptions options;
  Eigen::VectorX<symbolic::Variable> variables(2);
  variables[0] = symbolic::Variable("s1");
  variables[1] = symbolic::Variable("s2");

  // The parameterization is qᵢ = atan2(2sᵢ, 1-sᵢ²) for i=1,2.
  Eigen::VectorX<symbolic::Expression> parameterization_expression(2);
  for (int i = 0; i < 2; ++i) {
    parameterization_expression[i] =
        atan2(2 * variables[i], 1 - pow(variables[i], 2));
  }
  options.SetParameterizationFromExpression(parameterization_expression,
                                            variables);
  EXPECT_TRUE(options.get_parameterization_is_threadsafe());
  EXPECT_EQ(options.get_parameterization_dimension(), 2);

  CheckParameterization(options.get_parameterization());

  HPolyhedron region =
      IrisZo(*checker_, starting_ellipsoid_rational_forward_kinematics_,
             domain_rational_forward_kinematics_, options);
  CheckRegionRationalForwardKinematics(region);
}

TEST_F(DoublePendulumRationalForwardKinematics, BadParameterization) {
  // Verify that we fail gracefully if the parameterization has the wrong output
  // dimension (even if we claim it outputs the correct dimension).
  IrisZoOptions options;
  options.set_parameterization(
      [](const VectorXd& q) -> VectorXd {
        return Vector1d(0.0);
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_rational_forward_kinematics_,
             domain_rational_forward_kinematics_, options),
      ".*wrong dimension.*");
}

// Reproduced from the IrisInConfigurationSpace unit tests.
TEST_F(BlockOnGround, IrisZoTest) {
  IrisZoOptions options;
  options.verbose = true;
  meshcat_->Delete();
  options.meshcat = meshcat_;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

// Reproduced from the IrisInConfigurationSpace unit tests.
TEST_F(ConvexConfigurationSpace, IrisZoTest) {
  IrisZoOptions options;

  // Turn on meshcat for addition debugging visualizations.
  // This example is truly adversarial for IRIS. After one iteration, the
  // maximum-volume inscribed ellipse is approximately centered in C-free. So
  // finding a counter-example in the bottom corner (near the test point) is
  // not only difficult because we need to sample in a corner of the polytope,
  // but because the objective is actually pulling the counter-example search
  // away from that corner. Open the meshcat visualization to step through the
  // details!
  meshcat_->Delete();
  options.meshcat = meshcat_;
  options.verbose = true;
  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

// Verify that we throw a reasonable error when the initial point is in
// collision, and when the initial point violates an additional constraint.
TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, BadInitialEllipsoid) {
  IrisZoOptions options;
  options.prog_with_additional_constraints = &prog_;

  Hyperellipsoid ellipsoid_in_collision =
      Hyperellipsoid::MakeHypersphere(1e-2, Eigen::Vector2d(-1.0, 1.0));
  Hyperellipsoid ellipsoid_violates_constraint =
      Hyperellipsoid::MakeHypersphere(1e-2, Eigen::Vector2d(-0.1, 0.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, ellipsoid_in_collision, domain_, options),
      ".*Starting ellipsoid center.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, ellipsoid_violates_constraint, domain_, options),
      ".*Starting ellipsoid center.*");
}

TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, IrisZoTest) {
  IrisZoOptions options;
  options.prog_with_additional_constraints = &prog_;
  options.max_iterations = 1;
  options.max_iterations_separating_planes = 1;

  meshcat_->Delete();
  options.meshcat = meshcat_;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

TEST_F(ConvexConfigurationSpaceWithNotThreadsafeConstraint, IrisZoTest) {
  IrisZoOptions options;
  options.prog_with_additional_constraints = &prog_;
  options.max_iterations = 3;
  options.max_iterations_separating_planes = 20;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, BadContainmentPoint) {
  IrisZoOptions options;
  options.prog_with_additional_constraints = &prog_;

  // If we have a containment point violating this constraint, IrisZo should
  // throw. Three points are needed to ensure the center of the starting
  // ellipsoid is within the convex hull of the points we must contain.
  Eigen::Matrix2Xd containment_points(2, 3);
  // clang-format off
  containment_points << -0.2, -0.6, -0.6,
                         0.0,  0.1, -0.1;
  // clang-format on
  options.containment_points = containment_points;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*containment points violates a constraint.*");
}

// If the user specifies a parameterization and a MathematicalProgram containing
// additional constraints, their dimensions must match.
TEST_F(ConvexConfigurationSubspace, AdditionalConstraintsDimensionMismatch) {
  IrisZoOptions options;

  options.set_parameterization(
      [](const Vector1d& config) -> Vector2d {
        return Vector2d{config[0], 2 * config[0] + 1};
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  options.prog_with_additional_constraints = &prog;

  // Since we have a parameterization, the prog with additional constraints will
  // have the wrong dimension. We expect an error message.
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*num_vars.*parameterized_dimension.*");
}

TEST_F(ConvexConfigurationSubspace, FunctionParameterization) {
  IrisZoOptions options;
  options.set_parameterization(
      [](const Vector1d& config) -> Vector2d {
        return Vector2d{config[0], 2 * config[0] + 1};
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);

  meshcat_->Delete();
  PlotEnvironmentAndRegionSubspace(region, options.get_parameterization());
}

TEST_F(ConvexConfigurationSubspace, ExpressionParameterization) {
  // Finally, we test that the parameterization matches what we get when we
  // build it up manually using an Expression.
  Eigen::VectorX<symbolic::Variable> variables(1);
  variables[0] = symbolic::Variable("q");

  // The parameterization is (x, y) = (q, 2*q + 1)
  Eigen::VectorX<symbolic::Expression> parameterization_expression(2);
  parameterization_expression[0] = symbolic::Expression(variables[0]);
  parameterization_expression[1] = 2 * symbolic::Expression(variables[0]) + 1;

  IrisZoOptions options;
  options.SetParameterizationFromExpression(parameterization_expression,
                                            variables);
  EXPECT_TRUE(options.get_parameterization_is_threadsafe());
  EXPECT_EQ(options.get_parameterization_dimension(), 1);

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
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
