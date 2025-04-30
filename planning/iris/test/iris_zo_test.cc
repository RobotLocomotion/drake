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

  // Verify that we throw a reasonable error when the initial point is in
  // collision, and when the initial point violates an additional constraint.
  Hyperellipsoid ellipsoid_in_collision =
      Hyperellipsoid::MakeHypersphere(1e-2, Eigen::Vector2d(-1.0, 1.0));
  Hyperellipsoid ellipsoid_violates_constraint =
      Hyperellipsoid::MakeHypersphere(1e-2, Eigen::Vector2d(-0.1, 0.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZoFromUrdf(convex_urdf, ellipsoid_in_collision, options),
      ".*Starting ellipsoid center.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZoFromUrdf(convex_urdf, ellipsoid_violates_constraint, options),
      ".*Starting ellipsoid center.*");

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

  // Finally, we test that the parameterization matches what we get when we
  // build it up manually using an Expression.
  Eigen::VectorX<symbolic::Variable> variables(1);
  variables[0] = symbolic::Variable("q");

  // The parameterization is (x, y) = (q, 2*q + 1)
  Eigen::VectorX<symbolic::Expression> parameterization_expression(2);
  parameterization_expression[0] = symbolic::Expression(variables[0]);
  parameterization_expression[1] = 2 * symbolic::Expression(variables[0]) + 1;

  options.SetParameterizationFromExpression(parameterization_expression,
                                            variables);
  EXPECT_TRUE(options.get_parameterization_is_threadsafe());
  EXPECT_EQ(options.get_parameterization_dimension(), 1);
  region = IrisZoFromUrdf(convex_urdf, starting_ellipsoid, options, &domain);
  EXPECT_TRUE(region.PointInSet(region_query_point_1));
  EXPECT_TRUE(region.PointInSet(region_query_point_2));
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
