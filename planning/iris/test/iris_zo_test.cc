#include "drake/planning/iris/iris_zo.h"

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/planning/iris/iris_common.h"
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

GTEST_TEST(IrisZoOptionsTest, Serialize) {
  IrisZoOptions options;
  const std::string serialized = yaml::SaveYamlString(options);
  const auto deserialized = yaml::LoadYamlString<IrisZoOptions>(serialized);

  // Subfields accssed via the CommonSampledIrisOptions member
  // sampled_iris_options.
  EXPECT_EQ(deserialized.sampled_iris_options.num_particles,
            options.sampled_iris_options.num_particles);
  EXPECT_EQ(deserialized.sampled_iris_options.tau,
            options.sampled_iris_options.tau);
  EXPECT_EQ(deserialized.sampled_iris_options.delta,
            options.sampled_iris_options.delta);
  EXPECT_EQ(deserialized.sampled_iris_options.epsilon,
            options.sampled_iris_options.epsilon);
  EXPECT_EQ(deserialized.sampled_iris_options.containment_points,
            options.sampled_iris_options.containment_points);
  EXPECT_EQ(deserialized.sampled_iris_options.max_iterations,
            options.sampled_iris_options.max_iterations);
  EXPECT_EQ(deserialized.sampled_iris_options.max_iterations_separating_planes,
            options.sampled_iris_options.max_iterations_separating_planes);
  EXPECT_EQ(
      deserialized.sampled_iris_options.max_separating_planes_per_iteration,
      options.sampled_iris_options.max_separating_planes_per_iteration);
  EXPECT_EQ(deserialized.sampled_iris_options.verbose,
            options.sampled_iris_options.verbose);
  EXPECT_EQ(deserialized.sampled_iris_options.require_sample_point_is_contained,
            options.sampled_iris_options.require_sample_point_is_contained);
  EXPECT_EQ(deserialized.sampled_iris_options.configuration_space_margin,
            options.sampled_iris_options.configuration_space_margin);
  EXPECT_EQ(deserialized.sampled_iris_options.termination_threshold,
            options.sampled_iris_options.termination_threshold);
  EXPECT_EQ(deserialized.sampled_iris_options.relative_termination_threshold,
            options.sampled_iris_options.relative_termination_threshold);
  EXPECT_EQ(deserialized.sampled_iris_options.remove_all_collisions_possible,
            options.sampled_iris_options.remove_all_collisions_possible);
  EXPECT_EQ(deserialized.sampled_iris_options.random_seed,
            options.sampled_iris_options.random_seed);
  EXPECT_EQ(deserialized.sampled_iris_options.mixing_steps,
            options.sampled_iris_options.mixing_steps);
  EXPECT_EQ(deserialized.sampled_iris_options.sample_particles_in_parallel,
            options.sampled_iris_options.sample_particles_in_parallel);

  // The non-built-in types are not serialized.
  EXPECT_EQ(deserialized.bisection_steps, options.bisection_steps);
  EXPECT_EQ(deserialized.sampled_iris_options.meshcat, nullptr);
  EXPECT_EQ(deserialized.sampled_iris_options.prog_with_additional_constraints,
            nullptr);
}

// Reproduced from the IrisNp unit tests.
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
  vector_of_options.back().parameterization = IrisParameterizationFunction(
      [](const VectorXd& q) -> VectorXd {
        return q;
      },
      /* parameterization_is_threadsafe */ false,
      /* parameterization_dimension */ 1);
  EXPECT_EQ(vector_of_options.back()
                .parameterization.get_parameterization_is_threadsafe(),
            false);

  // Now set the parameterization with parameterization_is_threadsafe set to
  // true.
  vector_of_options.emplace_back();
  vector_of_options.back().parameterization = IrisParameterizationFunction(
      [](const VectorXd& q) -> VectorXd {
        return q;
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);
  EXPECT_EQ(vector_of_options[1]
                .parameterization.get_parameterization_is_threadsafe(),
            true);

  // Now use an Expression parameterization.
  Eigen::VectorX<symbolic::Variable> variables(1);
  variables[0] = symbolic::Variable("q");
  Eigen::VectorX<symbolic::Expression> parameterization_expression(1);
  parameterization_expression[0] = symbolic::Expression(variables[0]);
  vector_of_options.emplace_back();
  vector_of_options.back().parameterization =
      IrisParameterizationFunction(parameterization_expression, variables);
  // Expression parameterizations are always threadsafe.
  EXPECT_EQ(vector_of_options.back()
                .parameterization.get_parameterization_is_threadsafe(),
            true);

  for (auto& options : vector_of_options) {
    options.sampled_iris_options.verbose = true;

    // Check that the parameterization was set correctly.
    ASSERT_TRUE(
        options.parameterization.get_parameterization_dimension().has_value());
    EXPECT_EQ(options.parameterization.get_parameterization_dimension().value(),
              1);
    const Vector1d output =
        options.parameterization.get_parameterization_double()(Vector1d(3.0));
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
  options.parameterization = IrisParameterizationFunction(
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
  EXPECT_THROW(options.parameterization = IrisParameterizationFunction(
                   parameterization_expression_extra_variable, variables),
               std::exception);
  Eigen::VectorX<symbolic::Expression>
      parameterization_expression_missing_variable(1);
  parameterization_expression_missing_variable[0] = symbolic::Expression(1);
  EXPECT_THROW(IrisParameterizationFunction(
                   parameterization_expression_missing_variable, variables),
               std::exception);
}

// Reproduced from the IrisNp unit tests.
TEST_F(DoublePendulum, IrisZoTest) {
  IrisZoOptions options;
  options.sampled_iris_options.verbose = true;

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);

  PlotEnvironmentAndRegion(region);

  // Changing the sampling options should lead to a still-correct, but
  // slightly-different region.
  options.sampled_iris_options.sample_particles_in_parallel = true;
  HPolyhedron region2 =
      IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region2);
  EXPECT_FALSE(region.A().isApprox(region2.A(), 1e-10));
}

TEST_F(DoublePendulum, PostprocessRemoveCollisions) {
  IrisZoOptions options;

  // Deliberately set parameters so the initial region will pass the
  // probabilistic test.
  options.sampled_iris_options.tau = 0.01;
  options.sampled_iris_options.epsilon = 0.99;
  options.sampled_iris_options.delta = 0.99;
  options.sampled_iris_options.max_iterations = 1;
  options.sampled_iris_options.verbose = true;
  options.sampled_iris_options.remove_all_collisions_possible = false;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);

  Vector2d query_point(0.5, 0.0);
  EXPECT_FALSE(checker_->CheckConfigCollisionFree(query_point));
  EXPECT_TRUE(region.PointInSet(query_point));

  options.sampled_iris_options.remove_all_collisions_possible = true;
  region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  EXPECT_FALSE(region.PointInSet(query_point));
}

TEST_F(DoublePendulum, RelaxMargin) {
  IrisZoOptions options;

  // Deliberately set the configuration space margin to be very large, so that
  // the hyperplanes added will cut off the seed point and cause an error.
  options.sampled_iris_options.configuration_space_margin = 1e8;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*within sampled_iris_options\\.configuration_space_margin of being "
      "infeasible.*");

  options.sampled_iris_options.relax_margin = true;
  EXPECT_NO_THROW(IrisZo(*checker_, starting_ellipsoid_, domain_, options));
}

// Test growing a region for the double pendulum along a parameterization of the
// configuration space built from RationalForwardKinematics.
TEST_F(DoublePendulumRationalForwardKinematics,
       ParameterizationFromStaticConstructor) {
  IrisZoOptions options;
  options.parameterization =
      IrisParameterizationFunction(&rational_kinematics_,
                                   /* q_star_val */ Vector2d::Zero());
  options.sampled_iris_options.verbose = true;

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  // Check that the parameterization was set correctly.
  EXPECT_EQ(options.parameterization.get_parameterization_is_threadsafe(),
            true);
  ASSERT_TRUE(
      options.parameterization.get_parameterization_dimension().has_value());
  EXPECT_EQ(options.parameterization.get_parameterization_dimension().value(),
            2);

  CheckParameterization(options.parameterization.get_parameterization_double());

  HPolyhedron region =
      IrisZo(*checker_, starting_ellipsoid_rational_forward_kinematics_,
             domain_rational_forward_kinematics_, options);

  CheckRegionRationalForwardKinematics(region);
  PlotEnvironmentAndRegionRationalForwardKinematics(
      region, options.parameterization.get_parameterization_double(),
      region_query_point_1_);
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
  options.parameterization =
      IrisParameterizationFunction(parameterization_expression, variables);
  EXPECT_TRUE(options.parameterization.get_parameterization_is_threadsafe());
  EXPECT_EQ(options.parameterization.get_parameterization_dimension(), 2);

  CheckParameterization(options.parameterization.get_parameterization_double());

  HPolyhedron region =
      IrisZo(*checker_, starting_ellipsoid_rational_forward_kinematics_,
             domain_rational_forward_kinematics_, options);
  CheckRegionRationalForwardKinematics(region);
}

TEST_F(DoublePendulumRationalForwardKinematics, BadParameterization) {
  // Verify that we fail gracefully if the parameterization has the wrong output
  // dimension (even if we claim it outputs the correct dimension).
  IrisZoOptions options;
  options.parameterization = IrisParameterizationFunction(
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

// Reproduced from the IrisNp unit tests.
TEST_F(BlockOnGround, IrisZoTest) {
  IrisZoOptions options;
  options.sampled_iris_options.verbose = true;
  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

// Reproduced from the IrisNp unit tests.
TEST_F(ConvexConfigurationSpace, IrisZoTest) {
  IrisZoOptions options;

  options.sampled_iris_options.sample_particles_in_parallel = true;

  // Turn on meshcat for addition debugging visualizations.
  // This example is truly adversarial for IRIS. After one iteration, the
  // maximum-volume inscribed ellipse is approximately centered in C-free. So
  // finding a counter-example in the bottom corner (near the test point) is
  // not only difficult because we need to sample in a corner of the polytope,
  // but because the objective is actually pulling the counter-example search
  // away from that corner. Open the meshcat visualization to step through the
  // details!
  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;
  options.sampled_iris_options.verbose = true;
  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

// Verify that we throw a reasonable error when the initial point is in
// collision, and when the initial point violates an additional constraint.
TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, BadInitialEllipsoid) {
  IrisZoOptions options;
  options.sampled_iris_options.prog_with_additional_constraints = &prog_;

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
  options.sampled_iris_options.prog_with_additional_constraints = &prog_;
  options.sampled_iris_options.max_iterations = 1;
  options.sampled_iris_options.max_iterations_separating_planes = 1;

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

TEST_F(ConvexConfigurationSpaceWithNotThreadsafeConstraint, IrisZoTest) {
  IrisZoOptions options;
  options.sampled_iris_options.prog_with_additional_constraints = &prog_;
  options.sampled_iris_options.max_iterations = 3;
  options.sampled_iris_options.max_iterations_separating_planes = 20;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, BadContainmentPoint) {
  IrisZoOptions options;
  options.sampled_iris_options.prog_with_additional_constraints = &prog_;

  // If we have a containment point violating this constraint, IrisZo should
  // throw. Three points are needed to ensure the center of the starting
  // ellipsoid is within the convex hull of the points we must contain.
  Eigen::Matrix2Xd containment_points(2, 3);
  // clang-format off
  containment_points << -0.2, -0.6, -0.6,
                         0.0,  0.1, -0.1;
  // clang-format on
  options.sampled_iris_options.containment_points = containment_points;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*containment points violates a constraint.*");
}

// If the user specifies a parameterization and a MathematicalProgram containing
// additional constraints, their dimensions must match.
TEST_F(ConvexConfigurationSubspace, AdditionalConstraintsDimensionMismatch) {
  IrisZoOptions options;

  options.parameterization = IrisParameterizationFunction(
      [](const Vector1d& config) -> Vector2d {
        return Vector2d{config[0], 2 * config[0] + 1};
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  options.sampled_iris_options.prog_with_additional_constraints = &prog;

  // Since we have a parameterization, the prog with additional constraints will
  // have the wrong dimension. We expect an error message.
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*num_vars.*parameterized_dimension.*");
}

TEST_F(ConvexConfigurationSubspace, FunctionParameterization) {
  IrisZoOptions options;
  options.parameterization = IrisParameterizationFunction(
      [](const Vector1d& config) -> Vector2d {
        return Vector2d{config[0], 2 * config[0] + 1};
      },
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 1);

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);

  meshcat_->Delete();
  PlotEnvironmentAndRegionSubspace(
      region, options.parameterization.get_parameterization_double());

  // Also verify that we can have additional constraints, and they play nice
  // with the parameterization.
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1, "x");
  prog.AddLinearConstraint(Vector1d(1.0),
                           -std::numeric_limits<float>::infinity(), -0.25, x);
  options.sampled_iris_options.prog_with_additional_constraints = &prog;

  region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);

  Vector1d query_point_in(-0.3);
  Vector1d query_point_out(-0.2);

  EXPECT_TRUE(region.PointInSet(query_point_in));
  EXPECT_FALSE(region.PointInSet(query_point_out));

  // Verify that query_point_out is still collision free. (It just violates the
  // added constraint.)
  EXPECT_TRUE(checker_->CheckConfigCollisionFree(
      options.parameterization.get_parameterization_double()(query_point_out)));
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
  options.parameterization =
      IrisParameterizationFunction(parameterization_expression, variables);
  EXPECT_TRUE(options.parameterization.get_parameterization_is_threadsafe());
  EXPECT_EQ(options.parameterization.get_parameterization_dimension(), 1);

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

// First, we verify that IrisZo throws when a single containment point is
// passed. In this case, the convex hull of the containment points does not
// containt center of the starting ellipsoid.
TEST_F(FourCornersBoxes, SingleContainmentPoint) {
  Eigen::Matrix2Xd single_containment_point(2, 1);
  single_containment_point << 0, 1;

  IrisZoOptions options;
  options.sampled_iris_options.verbose = true;
  options.sampled_iris_options.configuration_space_margin = 0.04;
  options.sampled_iris_options.containment_points = single_containment_point;

  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisZo(*checker_, starting_ellipsoid_, domain_, options),
      ".*outside of the convex hull of the containment points.*");
}

// Test if we can force the containment of a two points (the ellipsoid center
// and one other point). This test is explicitly added to ensure that the
// procedure works when using fewer points than a simplex are given.
TEST_F(FourCornersBoxes, TwoContainmentPoints) {
  Eigen::Matrix2Xd containment_points(2, 2);
  // clang-format off
  containment_points << 0, starting_ellipsoid_.center().x(),
                        1, starting_ellipsoid_.center().y();
  // clang-format on
  IrisZoOptions options;
  options.sampled_iris_options.verbose = true;
  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;
  options.sampled_iris_options.configuration_space_margin = 0.04;
  options.sampled_iris_options.containment_points = containment_points;

  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);
  CheckRegionContainsPoints(region, containment_points);
  PlotEnvironmentAndRegion(region);
  PlotContainmentPoints(containment_points);
  MaybePauseForUser();
}

TEST_F(FourCornersBoxes, FourContainmentPoints) {
  Eigen::Matrix2Xd containment_points(2, 4);
  double xw, yw;
  xw = 0.4;
  yw = 0.28;
  // clang-format off
  containment_points << -xw, xw,  xw, -xw,
                         yw, yw, -yw, -yw;
  // clang-format on
  IrisZoOptions options;
  options.sampled_iris_options.verbose = true;
  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;
  options.sampled_iris_options.containment_points = containment_points;
  options.sampled_iris_options.max_iterations_separating_planes = 100;
  options.sampled_iris_options.max_iterations = -1;
  HPolyhedron region = IrisZo(*checker_, starting_ellipsoid_, domain_, options);

  CheckRegionContainsPoints(region, containment_points);
  PlotEnvironmentAndRegion(region);
  PlotContainmentPoints(containment_points);
  MaybePauseForUser();
}

}  // namespace
}  // namespace planning
}  // namespace drake
