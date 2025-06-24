#include "drake/planning/iris/iris_np2.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/iris/iris_common.h"
#include "drake/planning/iris/test/iris_test_utilities.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/nlopt_solver.h"

namespace drake {
namespace planning {
namespace {

using Eigen::Vector2d;
using Eigen::VectorX;
using Eigen::VectorXd;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using symbolic::Expression;
using symbolic::Variable;

TEST_F(JointLimits1D, JointLimitsBasic) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);
  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

// Check unsupported features.
TEST_F(JointLimits1D, UnsupportedOptions) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  options.sampled_iris_options.containment_points = Eigen::MatrixXd::Zero(1, 3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*additional containment points.*");
  options.sampled_iris_options.containment_points = std::nullopt;

  solvers::MathematicalProgram prog;
  options.sampled_iris_options.prog_with_additional_constraints = &prog;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*additional constriants.*");
  options.sampled_iris_options.prog_with_additional_constraints = nullptr;

  VectorX<Variable> varable_vector(1);
  VectorX<Expression> expression_vector(1);
  expression_vector[0] = varable_vector[0] + 1;
  options.parameterization =
      IrisParameterizationFunction(expression_vector, varable_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*parameterized subspace.*");
  options.parameterization = IrisParameterizationFunction();

  const Sphere sphere(0.1);
  const BodyShapeDescription body_shape{sphere, {}, "limits", "movable"};
  sgcc_ptr->AddCollisionShape("test", body_shape);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*added collision shapes.*");
  sgcc_ptr->RemoveAllAddedCollisionShapes();
}

// Check padding as an unsupported feature. (We have to use a test environment
// with multiple collision geometries.)
TEST_F(DoublePendulum, PaddingUnsupported) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  sgcc_ptr->SetPaddingAllRobotEnvironmentPairs(1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*nonzero padding.*");
}

TEST_F(DoublePendulum, IrisNp2Test) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  options.sampled_iris_options.verbose = true;

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);

  PlotEnvironmentAndRegion(region);

  // Changing the sampling options should lead to a still-correct, but
  // slightly-different region.
  options.sampled_iris_options.sample_particles_in_parallel = true;
  HPolyhedron region2 =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region2);
  EXPECT_FALSE(region.A().isApprox(region2.A(), 1e-10));
}

// Check that we can filter out certain collisions.
TEST_F(DoublePendulum, FilterCollisions) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  options.sampled_iris_options.verbose = true;
  const auto& body_A = sgcc_ptr->plant().GetBodyByName("fixed");
  const auto& body_B = sgcc_ptr->plant().GetBodyByName("link2");
  sgcc_ptr->SetCollisionFilteredBetween(body_A, body_B, true);

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);

  // Check the four corners for the joint limits.
  VectorXd upper_limits = sgcc_ptr->plant().GetPositionUpperLimits();
  VectorXd lower_limits = sgcc_ptr->plant().GetPositionLowerLimits();
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
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  solvers::EqualityConstrainedQPSolver invalid_solver;
  options.solver = &invalid_solver;

  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*EqualityConstrainedQPSolver is unable to solve.*");
}

TEST_F(DoublePendulum, PostprocessRemoveCollisions) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  // Deliberately set parameters so the initial region will pass the
  // probabilistic test.
  options.sampled_iris_options.tau = 0.01;
  options.sampled_iris_options.epsilon = 0.99;
  options.sampled_iris_options.delta = 0.99;
  options.sampled_iris_options.max_iterations = 1;
  options.sampled_iris_options.verbose = true;
  options.sampled_iris_options.remove_all_collisions_possible = false;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);

  Vector2d query_point(0.5, 0.0);
  EXPECT_FALSE(sgcc_ptr->CheckConfigCollisionFree(query_point));
  EXPECT_TRUE(region.PointInSet(query_point));

  options.sampled_iris_options.remove_all_collisions_possible = true;
  region = IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  EXPECT_FALSE(region.PointInSet(query_point));
}

TEST_F(BlockOnGround, IrisNp2Test) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  meshcat_->Delete();
  options.sampled_iris_options.meshcat = meshcat_;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

TEST_F(ConvexConfigurationSpace, IrisNp2Test) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

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

  // We use IPOPT for this test since SNOPT has a large number of solve failures
  // in this environment.
  solvers::IpoptSolver solver;
  options.solver = &solver;

  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
  PlotEnvironmentAndRegion(region);
}

}  // namespace
}  // namespace planning
}  // namespace drake
