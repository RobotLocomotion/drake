#include "drake/planning/iris/iris_np2.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/iris/iris_common.h"
#include "drake/planning/iris/test/iris_test_utilities.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/nlopt_solver.h"

namespace drake {
namespace planning {
namespace {

using common::MaybePauseForUser;
using Eigen::Vector2d;
using Eigen::VectorX;
using Eigen::VectorXd;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using symbolic::Expression;
using symbolic::Variable;

GTEST_TEST(IrisNp2OptionsTest, Serialize) {
  IrisNp2Options options;
  options.sampling_strategy = "ray";

  const std::string serialized = yaml::SaveYamlString(options);
  const auto deserialized = yaml::LoadYamlString<IrisNp2Options>(serialized);

  EXPECT_EQ(deserialized.sampling_strategy, options.sampling_strategy);

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
  EXPECT_EQ(deserialized.solver, nullptr);
  EXPECT_EQ(deserialized.sampled_iris_options.meshcat, nullptr);
  EXPECT_EQ(deserialized.sampled_iris_options.prog_with_additional_constraints,
            nullptr);
}

TEST_F(JointLimits1D, JointLimitsBasic) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);
  HPolyhedron region =
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

// Check unsupported features.
TEST_F(JointLimits1D, UnsupportedOptions) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  VectorX<Variable> varable_vector(1);
  VectorX<Expression> expression_vector(1);
  expression_vector[0] = varable_vector[0] + 1;
  options.parameterization =
      IrisParameterizationFunction(expression_vector, varable_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*autodiff-compatible parameterization.*");
  options.parameterization = IrisParameterizationFunction();

  const Sphere sphere(0.1);
  const BodyShapeDescription body_shape{sphere, {}, "limits", "movable"};
  scene_graph_checker->AddCollisionShape("test", body_shape);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*added collision shapes.*");
  scene_graph_checker->RemoveAllAddedCollisionShapes();

  options.ray_sampler_options.ray_search_num_steps = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*ray_search_num_steps.*");
  options.ray_sampler_options.ray_search_num_steps = 1;

  options.ray_sampler_options.num_particles_to_walk_towards = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*num_particles_to_walk_towards.*");
  options.ray_sampler_options.num_particles_to_walk_towards = 1;

  options.sampling_strategy = "another sampling strategy";
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*invalid IrisNp2 sampling strategy.*another sampling strategy.*");
  options.sampling_strategy = "greedy";
}

// Check padding as an unsupported feature. (We have to use a test environment
// with multiple collision geometries.)
TEST_F(DoublePendulum, PaddingUnsupported) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  scene_graph_checker->SetPaddingAllRobotEnvironmentPairs(-1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*negative padding.*");
}

// Test growing a region for the double pendulum along a parameterization of the
// configuration space built from RationalForwardKinematics. This also
// implicitly ensures that the IrisParameterizationFunction constructor that
// takes in a RationalForwardKinematics object populates the double- and
// AutoDiffXd-valued parameterization functions, since both must be available in
// order to call IrisNp2.
TEST_F(DoublePendulumRationalForwardKinematics,
       ParameterizationFromStaticConstructor) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

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

  HPolyhedron region = IrisNp2(*scene_graph_checker,
                               starting_ellipsoid_rational_forward_kinematics_,
                               domain_rational_forward_kinematics_, options);

  CheckRegionRationalForwardKinematics(region);
  PlotEnvironmentAndRegionRationalForwardKinematics(
      region, options.parameterization.get_parameterization_double(),
      region_query_point_1_);
}

TEST_F(DoublePendulum, IrisNp2Test) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  options.sampled_iris_options.verbose = true;

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
  CheckRegion(region);

  PlotEnvironmentAndRegion(region);

  // Changing the sampling options should lead to a still-correct, but
  // slightly-different region.
  options.sampled_iris_options.sample_particles_in_parallel = true;
  HPolyhedron region2 =
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
  CheckRegion(region2);
  EXPECT_FALSE(region.A().isApprox(region2.A(), 1e-10));
}

TEST_F(DoublePendulum, PositivePadding) {
  std::vector<IrisNp2Options> options_to_try(3);
  options_to_try[0].sampling_strategy = "greedy";
  options_to_try[1].sampling_strategy = "ray";
  options_to_try[2].sampling_strategy = "ray";
  options_to_try[2].ray_sampler_options.only_walk_toward_collisions = true;
  for (int i = 0; i < ssize(options_to_try); ++i) {
    meshcat_->Delete();
    auto& options = options_to_try[i];
    options.sampled_iris_options.meshcat = meshcat_;
    auto* scene_graph_checker =
        dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
    ASSERT_TRUE(scene_graph_checker != nullptr);

    const double padding = 0.5;
    scene_graph_checker->SetPaddingAllRobotEnvironmentPairs(padding);
    scene_graph_checker->SetPaddingAllRobotRobotPairs(padding);
    HPolyhedron region =
        IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);

    EXPECT_TRUE(region.PointInSet(Vector2d{.2, 0.0}));
    EXPECT_FALSE(region.PointInSet(Vector2d{.3, 0.0}));
    EXPECT_TRUE(region.PointInSet(Vector2d{-.2, 0.0}));
    EXPECT_FALSE(region.PointInSet(Vector2d{-.3, 0.0}));

    PlotEnvironmentAndRegion(region);
  }
}

// Check that we can filter out certain collisions.
TEST_F(DoublePendulum, FilterCollisions) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  options.sampled_iris_options.verbose = true;
  const auto& body_A = scene_graph_checker->plant().GetBodyByName("fixed");
  const auto& body_B = scene_graph_checker->plant().GetBodyByName("link2");
  scene_graph_checker->SetCollisionFilteredBetween(body_A, body_B, true);

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);

  // Check the four corners for the joint limits.
  VectorXd upper_limits = scene_graph_checker->plant().GetPositionUpperLimits();
  VectorXd lower_limits = scene_graph_checker->plant().GetPositionLowerLimits();
  EXPECT_EQ(upper_limits.size(), 2);
  EXPECT_EQ(lower_limits.size(), 2);
  EXPECT_TRUE(region.PointInSet(upper_limits));
  EXPECT_TRUE(region.PointInSet(lower_limits));
  EXPECT_TRUE(region.PointInSet(Vector2d(upper_limits[0], lower_limits[1])));
  EXPECT_TRUE(region.PointInSet(Vector2d(lower_limits[0], upper_limits[1])));

  PlotEnvironmentAndRegion(region);
}

// Verify we can specify the solver for the counterexample search by
// deliberately specifying a solver that can't solve problems of that type, and
// catch the error message.
TEST_F(DoublePendulum, SpecifySolver) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  solvers::EqualityConstrainedQPSolver invalid_solver;
  options.solver = &invalid_solver;

  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*EqualityConstrainedQPSolver is unable to solve.*");
}

TEST_F(DoublePendulum, PostprocessRemoveCollisions) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  // Deliberately set parameters so the initial region will pass the
  // probabilistic test.
  options.sampled_iris_options.tau = 0.01;
  options.sampled_iris_options.epsilon = 0.99;
  options.sampled_iris_options.delta = 0.99;
  options.sampled_iris_options.max_iterations = 1;
  options.sampled_iris_options.verbose = true;
  options.sampled_iris_options.remove_all_collisions_possible = false;

  HPolyhedron region =
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);

  Vector2d query_point(0.5, 0.0);
  EXPECT_FALSE(scene_graph_checker->CheckConfigCollisionFree(query_point));
  EXPECT_TRUE(region.PointInSet(query_point));

  options.sampled_iris_options.remove_all_collisions_possible = true;
  region = IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
  EXPECT_FALSE(region.PointInSet(query_point));
}

TEST_F(DoublePendulum, RelaxMargin) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  // Deliberately set the configuration space margin to be very large, so that
  // the hyperplanes added will cut off the seed point and cause an error.
  options.sampled_iris_options.configuration_space_margin = 1e8;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
      ".*within sampled_iris_options\\.configuration_space_margin of being "
      "infeasible.*");

  options.sampled_iris_options.relax_margin = true;
  EXPECT_NO_THROW(
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options));
}

TEST_F(DoublePendulumRationalForwardKinematics, FunctionParameterization) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  options.sampled_iris_options.meshcat = meshcat_;

  auto parameterization_double = [](const Vector2d& config) -> Vector2d {
    return Vector2d{atan2(2 * config(0), 1 - std::pow(config(0), 2)),
                    atan2(2 * config(1), 1 - std::pow(config(1), 2))};
  };
  auto parameterization_autodiff =
      [](const Vector2<AutoDiffXd>& config) -> Vector2<AutoDiffXd> {
    return drake::Vector2<AutoDiffXd>{
        atan2(2 * config(0), 1 - pow(config(0), 2)),
        atan2(2 * config(1), 1 - pow(config(1), 2))};
  };

  options.parameterization = IrisParameterizationFunction(
      parameterization_double, parameterization_autodiff,
      /* parameterization_is_threadsafe */ true,
      /* parameterization_dimension */ 2);

  CheckParameterization(options.parameterization.get_parameterization_double());

  HPolyhedron region = IrisNp2(*scene_graph_checker,
                               starting_ellipsoid_rational_forward_kinematics_,
                               domain_rational_forward_kinematics_, options);

  CheckRegionRationalForwardKinematics(region);
  PlotEnvironmentAndRegionRationalForwardKinematics(
      region, options.parameterization.get_parameterization_double(),
      region_query_point_1_);

  // Check that this still works with padding.
  const double padding = 0.5;
  scene_graph_checker->SetPaddingAllRobotEnvironmentPairs(padding);
  scene_graph_checker->SetPaddingAllRobotRobotPairs(padding);

  region = IrisNp2(*scene_graph_checker,
                   starting_ellipsoid_rational_forward_kinematics_,
                   domain_rational_forward_kinematics_, options);

  EXPECT_TRUE(region.PointInSet(Vector2d{.1, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{.2, 0.0}));
  EXPECT_TRUE(region.PointInSet(Vector2d{-.1, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{-.2, 0.0}));

  PlotEnvironmentAndRegionRationalForwardKinematics(
      region, options.parameterization.get_parameterization_double(),
      region_query_point_1_);
}

TEST_F(BlockOnGround, IrisNp2Test) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

TEST_F(ConvexConfigurationSpace, IrisNp2Test) {
  std::vector<IrisNp2Options> options_to_try(3);
  options_to_try[0].sampling_strategy = "greedy";
  options_to_try[1].sampling_strategy = "ray";
  options_to_try[2].sampling_strategy = "ray";
  options_to_try[2].ray_sampler_options.only_walk_toward_collisions = true;
  for (int i = 0; i < ssize(options_to_try); ++i) {
    meshcat_->Delete();
    auto& options = options_to_try[i];
    auto* scene_graph_checker =
        dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
    ASSERT_TRUE(scene_graph_checker != nullptr);

    options.sampled_iris_options.sample_particles_in_parallel = true;

    // Turn on meshcat for addition debugging visualizations.
    // This example is truly adversarial for IRIS. After one iteration, the
    // maximum-volume inscribed ellipse is approximately centered in C-free. So
    // finding a counter-example in the bottom corner (near the test point) is
    // not only difficult because we need to sample in a corner of the polytope,
    // but because the objective is actually pulling the counter-example search
    // away from that corner. Open the meshcat visualization to step through the
    // details!
    options.sampled_iris_options.meshcat = meshcat_;
    options.sampled_iris_options.verbose = true;

    // SNOPT has a large number of solve failures in this environment, but the
    // add_hyperplane_if_solve_fails helps mitigate them.
    options.add_hyperplane_if_solve_fails = true;

    HPolyhedron region =
        IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
    CheckRegion(region);
    if (i == 0) {
      PlotEnvironmentAndRegion(region);
    }
  }
}

// Test that we can grow regions along a parameterized subspace that is not
// full-dimensional.
TEST_F(ConvexConfigurationSubspace, FunctionParameterization) {
  std::vector<IrisNp2Options> options_to_try(3);
  options_to_try[0].sampling_strategy = "greedy";
  options_to_try[1].sampling_strategy = "ray";
  options_to_try[2].sampling_strategy = "ray";
  options_to_try[2].ray_sampler_options.only_walk_toward_collisions = true;
  for (int i = 0; i < ssize(options_to_try); ++i) {
    meshcat_->Delete();
    auto& options = options_to_try[i];
    auto* scene_graph_checker =
        dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
    ASSERT_TRUE(scene_graph_checker != nullptr);

    auto parameterization_double = [](const Vector1d& config) -> Vector2d {
      return Vector2d{config[0], 2 * config[0] + 1};
    };
    auto parameterization_autodiff =
        [](const Vector1<AutoDiffXd>& config) -> Vector2<AutoDiffXd> {
      return Vector2<AutoDiffXd>{config[0], 2 * config[0] + 1};
    };

    options.parameterization =
        IrisParameterizationFunction(parameterization_double,
                                     /* parameterization_is_threadsafe */ true,
                                     /* parameterization_dimension */ 1);
    DRAKE_EXPECT_THROWS_MESSAGE(
        IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options),
        ".*autodiff-compatible parameterization.*");

    options.parameterization = IrisParameterizationFunction(
        parameterization_double, parameterization_autodiff,
        /* parameterization_is_threadsafe */ true,
        /* parameterization_dimension */ 1);

    HPolyhedron region =
        IrisNp2(*scene_graph_checker, starting_ellipsoid_, domain_, options);
    CheckRegion(region);

    if (i == 0) {
      PlotEnvironmentAndRegionSubspace(
          region, options.parameterization.get_parameterization_double());
    }
  }
}

// Test that we can grow regions along a parameterized subspace that is not
// full-dimensional when we have additional constraints.
TEST_F(ConvexConfigurationSubspace,
       FunctionParameterizationWithAdditionalConstraint) {
  IrisNp2Options options;
  auto* scene_graph_checker =
      dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(scene_graph_checker != nullptr);

  std::vector<IrisNp2Options> options_to_try(3);
  options_to_try[0].sampling_strategy = "greedy";
  options_to_try[1].sampling_strategy = "ray";
  options_to_try[2].sampling_strategy = "ray";
  options_to_try[2].ray_sampler_options.only_walk_toward_collisions = true;

  for (int i = 0; i < ssize(options_to_try); ++i) {
    auto parameterization_double = [](const Vector1d& config) -> Vector2d {
      return Vector2d{config[0], 2 * config[0] + 1};
    };
    auto parameterization_autodiff =
        [](const Vector1<AutoDiffXd>& config) -> Vector2<AutoDiffXd> {
      return Vector2<AutoDiffXd>{config[0], 2 * config[0] + 1};
    };

    solvers::MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(1, "x");
    prog.AddLinearConstraint(Vector1d(1.0),
                             -std::numeric_limits<float>::infinity(), -0.25, x);
    options_to_try[i].sampled_iris_options.prog_with_additional_constraints =
        &prog;

    options_to_try[i].parameterization = IrisParameterizationFunction(
        parameterization_double, parameterization_autodiff,
        /* parameterization_is_threadsafe */ true,
        /* parameterization_dimension */ 1);

    HPolyhedron region = IrisNp2(*scene_graph_checker, starting_ellipsoid_,
                                 domain_, options_to_try[i]);

    Vector1d query_point_in(-0.3);
    Vector1d query_point_out(-0.2);

    EXPECT_TRUE(region.PointInSet(query_point_in));
    EXPECT_FALSE(region.PointInSet(query_point_out));

    // Verify that query_point_out is still collision free. (It just violates
    // the added constraint.)
    EXPECT_TRUE(scene_graph_checker->CheckConfigCollisionFree(
        parameterization_double(query_point_out)));
  }
}

// Verify that we throw a reasonable error when the initial point is in
// collision, and when the initial point violates an additional constraint.
TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, BadInitialEllipsoid) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  options.sampled_iris_options.prog_with_additional_constraints = &prog_;

  Hyperellipsoid ellipsoid_in_collision =
      Hyperellipsoid::MakeHypersphere(1e-2, Eigen::Vector2d(-1.0, 1.0));
  Hyperellipsoid ellipsoid_violates_constraint =
      Hyperellipsoid::MakeHypersphere(1e-2, Eigen::Vector2d(-0.1, 0.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, ellipsoid_in_collision, domain_, options),
      ".*Starting ellipsoid center.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, ellipsoid_violates_constraint, domain_, options),
      ".*Starting ellipsoid center.*");
}

TEST_F(ConvexConfigurationSpaceWithThreadsafeConstraint, IrisNp2Test) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  // We use IPOPT for this test since SNOPT has a large number of solve failures
  // in this environment.
  solvers::IpoptSolver solver;
  options.solver = &solver;

  options.sampled_iris_options.prog_with_additional_constraints = &prog_;
  options.sampled_iris_options.verbose = true;

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

TEST_F(ConvexConfigurationSpaceWithNotThreadsafeConstraint, IrisNp2Test) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  // We use IPOPT for this test since SNOPT has a large number of solve failures
  // in this environment.
  solvers::IpoptSolver solver;
  options.solver = &solver;

  options.sampled_iris_options.prog_with_additional_constraints = &prog_;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

// First, we verify that IrisZo throws when a single containment point is
// passed. In this case, the convex hull of the containment points does not
// containt center of the starting ellipsoid.
TEST_F(FourCornersBoxes, SingleContainmentPoint) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  Eigen::Matrix2Xd single_containment_point(2, 1);
  single_containment_point << 0, 1;

  options.sampled_iris_options.verbose = true;
  options.sampled_iris_options.configuration_space_margin = 0.04;
  options.sampled_iris_options.containment_points = single_containment_point;

  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*outside of the convex hull of the containment points.*");
}

// Test if we can force the containment of a two points (the ellipsoid center
// and one other point). This test is explicitly added to ensure that the
// procedure works when using fewer points than a simplex are given.
TEST_F(FourCornersBoxes, TwoContainmentPoints) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  Eigen::Matrix2Xd containment_points(2, 2);
  // clang-format off
  containment_points << 0, starting_ellipsoid_.center().x(),
                        1, starting_ellipsoid_.center().y();
  // clang-format on
  options.sampled_iris_options.verbose = true;
  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;
  options.sampled_iris_options.configuration_space_margin = 0.04;
  options.sampled_iris_options.containment_points = containment_points;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegionContainsPoints(region, containment_points);
  PlotEnvironmentAndRegion(region);
  PlotContainmentPoints(containment_points);
  MaybePauseForUser();
}

TEST_F(FourCornersBoxes, FourContainmentPoints) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  Eigen::Matrix2Xd containment_points(2, 4);
  double xw, yw;
  xw = 0.4;
  yw = 0.28;
  // clang-format off
  containment_points << -xw, xw,  xw, -xw,
                         yw, yw, -yw, -yw;
  // clang-format on
  options.sampled_iris_options.verbose = true;
  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;
  options.sampled_iris_options.containment_points = containment_points;
  options.sampled_iris_options.max_iterations_separating_planes = 100;
  options.sampled_iris_options.max_iterations = -1;
  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);

  CheckRegionContainsPoints(region, containment_points);
  PlotEnvironmentAndRegion(region);
  PlotContainmentPoints(containment_points);
  MaybePauseForUser();
}

}  // namespace
}  // namespace planning
}  // namespace drake
