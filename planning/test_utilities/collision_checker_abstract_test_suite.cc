#include "drake/planning/test_utilities/collision_checker_abstract_test_suite.h"

#include <unordered_map>

#include <common_robotics_utilities/print.hpp>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/shape_specification.h"
#include "drake/planning/collision_avoidance.h"

namespace drake {
namespace planning {
namespace test {

using geometry::GeometryId;
using internal::ComputeCollisionAvoidanceDisplacement;
using multibody::BodyIndex;

multibody::parsing::ModelDirectives MakeCollisionCheckerTestScene() {
  // Assemble model directives.
  multibody::parsing::ModelDirective add_env_model;
  add_env_model.add_model = multibody::parsing::AddModel{
      "package://drake/planning/test_utilities/collision_ground_plane.sdf",
      "ground_plane_box"};
  multibody::parsing::ModelDirective add_env_weld;
  add_env_weld.add_weld = multibody::parsing::AddWeld{
      "world", "ground_plane_box::ground_plane_box"};

  multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = multibody::parsing::AddModel{
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf",
      "iiwa"};
  multibody::parsing::ModelDirective add_robot_weld;
  add_robot_weld.add_weld = multibody::parsing::AddWeld{"world", "iiwa::base"};

  const multibody::parsing::ModelDirectives directives{
      .directives = {add_env_model, add_env_weld, add_robot_model,
                     add_robot_weld}};
  return directives;
}

std::ostream& operator<<(std::ostream& out,
                         const CollisionCheckerTestParams& p) {
  out << "checker = " << NiceTypeName::Get(*p.checker);
  out << ", supports_added_world_obstacles = "
      << p.supports_added_world_obstacles;
  out << ", thread_stress_iterations = " << p.thread_stress_iterations;
  return out;
}

TEST_P(CollisionCheckerAbstractTestSuite, Clone) {
  auto params = GetParam();
  auto& checker = *params.checker;
  const auto cloned = checker.Clone();
  EXPECT_NE(cloned, nullptr);

  // Cloned object is different (identity, address) from its source.
  EXPECT_NE(cloned.get(), params.checker.get());

  // Logically, the objects should be the same. Provide some evidence.
  EXPECT_EQ(checker.robot_model_instances(), cloned->robot_model_instances());
  EXPECT_EQ(checker.GetZeroConfiguration(), cloned->GetZeroConfiguration());
  EXPECT_EQ(checker.edge_step_size(), cloned->edge_step_size());
  EXPECT_EQ(checker.MaxNumDistances(), cloned->MaxNumDistances());
  EXPECT_EQ(checker.num_allocated_contexts(), cloned->num_allocated_contexts());
  EXPECT_EQ(checker.GetPaddingMatrix(), cloned->GetPaddingMatrix());
  EXPECT_EQ(checker.GetFilteredCollisionMatrix(),
            cloned->GetFilteredCollisionMatrix());
  EXPECT_EQ(checker.ClassifyBodyCollisions(qs_.q1),
            cloned->ClassifyBodyCollisions(qs_.q1));
}

TEST_P(CollisionCheckerAbstractTestSuite, MaxNumDistances) {
  auto params = GetParam();
  auto& checker = *params.checker;
  // The exact number here depends on the model and the type of checker.
  int max_distances = checker.MaxNumDistances();
  // Try to max out our result from CalcRobotClearance(). Using a giant
  // influence distance should help.
  RobotClearance clearance = checker.CalcRobotClearance(qs_.q1, 1e6);
  EXPECT_GE(max_distances, clearance.size());
  // Empirically, the query above should consume over 80% of the predicted
  // allocation.
  // TODO(rpoyner-tri): Only the voxelized checker fails to consume the
  // predicted size; account for the discrepancy.
  EXPECT_GE(clearance.size(), max_distances * 0.8);
}

TEST_P(CollisionCheckerAbstractTestSuite, AddSpheres) {
  auto params = GetParam();
  auto& checker = *params.checker;
  const auto& dut_frame = checker.plant().GetFrameByName("iiwa_link_2");
  // Add a really big ball so that collision will definitely happen.
  checker.AddCollisionShapeToFrame("test", dut_frame, geometry::Sphere(1.0),
                                   math::RigidTransform<double>());
  EXPECT_FALSE(checker.CheckConfigCollisionFree(qs_.q1));
  EXPECT_NE(ssize(checker.GetAllAddedCollisionShapes()), 0);

  // While there are collisions, do some classifying.
  std::vector<RobotCollisionType> kinds =
      checker.ClassifyBodyCollisions(qs_.q1);
  std::unordered_map<RobotCollisionType, int> kind_counts;
  for (const auto& kind : kinds) {
    ++kind_counts[kind];
  }
  EXPECT_GE(kind_counts[RobotCollisionType::kNoCollision], 1);
  EXPECT_GE(kind_counts[RobotCollisionType::kSelfCollision], 1);
  EXPECT_GE(kind_counts[RobotCollisionType::kEnvironmentAndSelfCollision], 1);
  EXPECT_EQ(kind_counts[RobotCollisionType::kEnvironmentCollision], 0);

  // Also test the logic of removals and enumeration.
  checker.RemoveAllAddedCollisionShapes("other");
  EXPECT_FALSE(checker.CheckConfigCollisionFree(qs_.q1));
  EXPECT_NE(ssize(checker.GetAllAddedCollisionShapes()), 0);

  checker.RemoveAllAddedCollisionShapes("test");
  EXPECT_TRUE(checker.CheckConfigCollisionFree(qs_.q1));
  EXPECT_EQ(ssize(checker.GetAllAddedCollisionShapes()), 0);

  // Re-add a shape to test removal of everything.
  checker.AddCollisionShapeToFrame("test", dut_frame, geometry::Sphere(1.0),
                                   math::RigidTransform<double>());
  EXPECT_FALSE(checker.CheckConfigCollisionFree(qs_.q1));
  EXPECT_NE(ssize(checker.GetAllAddedCollisionShapes()), 0);

  checker.RemoveAllAddedCollisionShapes();
  EXPECT_TRUE(checker.CheckConfigCollisionFree(qs_.q1));
  EXPECT_EQ(ssize(checker.GetAllAddedCollisionShapes()), 0);
}

TEST_P(CollisionCheckerAbstractTestSuite, AddObstacles) {
  auto params = GetParam();
  auto& checker = *params.checker;
  // Add a really big cylinder so that collision will definitely happen.
  const bool added = checker.AddCollisionShapeToFrame(
      "test", checker.plant().world_frame(), geometry::Cylinder(1.0, 1.0),
      math::RigidTransform<double>());
  EXPECT_EQ(params.supports_added_world_obstacles, added);
  if (added) {
    EXPECT_FALSE(checker.CheckConfigCollisionFree(qs_.q1));
    checker.RemoveAllAddedCollisionShapes();
    EXPECT_TRUE(checker.CheckConfigCollisionFree(qs_.q1));
  }
}

// Tests "discrete" configuration and edge collision check methods. Provided
// `parallelism` controls parallelism of parallel check methods.
// TODO(calderpg-tri) Improve so that parallelism is exercised on all queries.
void CollisionCheckerAbstractTestSuite::TestDiscreteQueries(
    const CollisionCheckerTestParams& params, const Parallelism parallelism) {
  auto& checker = *params.checker;
  // Check single queries Queries we know are collision-free, given the robot
  // that MakeCollisionCheckerTestScene loads and the qs_ in the text fixture.
  EXPECT_TRUE(checker.CheckConfigCollisionFree(qs_.q1));
  EXPECT_TRUE(checker.CheckConfigCollisionFree(qs_.q2));

  // Queries we know are in collision.
  EXPECT_FALSE(checker.CheckConfigCollisionFree(qs_.q3));
  EXPECT_FALSE(checker.CheckConfigCollisionFree(qs_.q4));

  // TODO(SeanCurtis-TRI) These edge tests are more properly tested in the base
  // class's tests. The test here does *one* thing that that test doesn't do:
  // Stress test: the base class unit tests uses edges with just a few samples
  // and doesn't repeatedly invoke it like is done here. (How many samples are
  // produced in these various qs?) Figure out how important the stress testing
  // is and how best to reproduce it in the base unit tests.

  // Edge we know is collision-free.
  EXPECT_TRUE(checker.CheckEdgeCollisionFree(qs_.q1, qs_.q2));
  EXPECT_TRUE(
      checker.CheckEdgeCollisionFreeParallel(qs_.q1, qs_.q2, parallelism));
  EXPECT_TRUE(
      checker.MeasureEdgeCollisionFree(qs_.q1, qs_.q2).completely_free());
  EXPECT_TRUE(
      checker.MeasureEdgeCollisionFreeParallel(qs_.q1, qs_.q2, parallelism)
          .completely_free());

  // Edge we know is partially in collision.
  EXPECT_FALSE(checker.CheckEdgeCollisionFree(qs_.q2, qs_.q3));
  EXPECT_FALSE(checker.CheckEdgeCollisionFree(qs_.q3, qs_.q2));
  // q2 is free, q3 collides, we have a valid alpha value in the range [0, 1).
  const EdgeMeasure forward = checker.MeasureEdgeCollisionFree(qs_.q2, qs_.q3);
  ASSERT_TRUE(forward.partially_free());
  EXPECT_GE(forward.alpha(), 0.0);
  EXPECT_LT(forward.alpha(), 1.0);
  // q3 collides, q2 is free, no valid alpha.
  EXPECT_FALSE(
      checker.MeasureEdgeCollisionFree(qs_.q3, qs_.q2).partially_free());

  // Now repeat for the same (q2, q3) and (q3, q2) edges, but in parallel.
  EXPECT_FALSE(
      checker.CheckEdgeCollisionFreeParallel(qs_.q2, qs_.q3, parallelism));
  EXPECT_FALSE(
      checker.CheckEdgeCollisionFreeParallel(qs_.q3, qs_.q2, parallelism));
  const EdgeMeasure forward_parallel =
      checker.MeasureEdgeCollisionFreeParallel(qs_.q2, qs_.q3, parallelism);
  ASSERT_TRUE(forward_parallel.partially_free());
  EXPECT_FLOAT_EQ(forward.alpha(), forward_parallel.alpha());
  EXPECT_GE(forward_parallel.alpha(), 0.0);
  EXPECT_LT(forward_parallel.alpha(), 1.0);
  EXPECT_FALSE(
      checker.MeasureEdgeCollisionFreeParallel(qs_.q3, qs_.q2, parallelism)
          .partially_free());

  // Edge we know is entirely in collision.
  EXPECT_FALSE(checker.CheckEdgeCollisionFree(qs_.q3, qs_.q4));
  EXPECT_FALSE(
      checker.CheckEdgeCollisionFreeParallel(qs_.q3, qs_.q4, parallelism));
  EXPECT_FALSE(
      checker.MeasureEdgeCollisionFree(qs_.q3, qs_.q4).partially_free());
  EXPECT_FALSE(
      checker.MeasureEdgeCollisionFreeParallel(qs_.q3, qs_.q4, parallelism)
          .partially_free());

  // Check parallel queries.
  const std::vector<uint8_t> checks =
      checker.CheckConfigsCollisionFree(qs_.configs, parallelism);
  EXPECT_TRUE(checks.size() == qs_.configs.size());
  EXPECT_EQ(checks.at(0), 1);
  EXPECT_EQ(checks.at(1), 1);
  EXPECT_EQ(checks.at(2), 0);
  EXPECT_EQ(checks.at(3), 0);

  const std::vector<uint8_t> edge_checks =
      checker.CheckEdgesCollisionFree(qs_.edges, parallelism);
  EXPECT_TRUE(edge_checks.size() == qs_.edges.size());
  EXPECT_EQ(edge_checks.at(0), 1);
  EXPECT_EQ(edge_checks.at(1), 0);
  EXPECT_EQ(edge_checks.at(2), 0);
  EXPECT_EQ(edge_checks.at(3), 0);

  const std::vector<EdgeMeasure> results =
      checker.MeasureEdgesCollisionFree(qs_.edges, parallelism);
  EXPECT_EQ(results.size(), qs_.edges.size());
  EXPECT_FLOAT_EQ(results.at(0).alpha(), 1.0);
  EXPECT_FLOAT_EQ(results.at(1).alpha(), forward.alpha());
  EXPECT_FALSE(results.at(2).partially_free());
  EXPECT_FALSE(results.at(3).partially_free());
}

TEST_P(CollisionCheckerAbstractTestSuite, StressParallelDiscreteQueries) {
  auto params = GetParam();
  const auto parallelism = Parallelism::Max();
  // Since there are parallel query options, we test repeatedly to ensure that
  // tests are not flaky.
  const int iterations = params.thread_stress_iterations;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    TestDiscreteQueries(params, parallelism);
  }
}

TEST_P(CollisionCheckerAbstractTestSuite, ForceSerializeDiscreteQueries) {
  auto params = GetParam();
  const auto parallelism = Parallelism::None();
  TestDiscreteQueries(params, parallelism);
}

// Tests "distance" clearance and collision avoidance methods. Provided
// `parallelism` controls parallelism of calls to `CalcRobotClearance` only.
// TODO(calderpg-tri) Improve so that parallelism is exercised on all queries.
void CollisionCheckerAbstractTestSuite::TestDistanceQueries(
    const CollisionCheckerTestParams& params, const Parallelism parallelism) {
  auto& checker = *params.checker;

  // Test parallel calls to CalcRobotClearance.
#if defined(_OPENMP)
#pragma omp parallel for num_threads(parallelism.num_threads()) schedule(static)
#endif
  for (int thread = 0; thread < parallelism.num_threads(); ++thread) {
    EXPECT_NO_THROW(checker.CalcRobotClearance(qs_.q1, 0.0));
  }

  // Test collision gradient:
  // q1 is not in collision, so we should have a free-space only avoidance
  // gradient.
  const Eigen::VectorXd grad_q1_free =
      ComputeCollisionAvoidanceDisplacement(checker, qs_.q1, 0.0, 1.0);
  EXPECT_GT(grad_q1_free.norm(), 0.0);
  // q2 is not in collision, so we should have a free-space only avoidance
  // gradient.
  const Eigen::VectorXd grad_q2_free =
      ComputeCollisionAvoidanceDisplacement(checker, qs_.q2, 0.0, 1.0);
  EXPECT_GT(grad_q2_free.norm(), 0.0);
  // q3 is in collision, so we should have a free-space only avoidance
  // gradient.
  const Eigen::VectorXd grad_q3_free =
      ComputeCollisionAvoidanceDisplacement(checker, qs_.q3, 0.0, 1.0);
  EXPECT_GT(grad_q3_free.norm(), 0.0);
  // q3 is in collision, so we should have a in-collision avoidance gradient.
  const Eigen::VectorXd grad_q3_penetrating =
      ComputeCollisionAvoidanceDisplacement(checker, qs_.q3, -1.0, 0.0);
  EXPECT_GT(grad_q3_penetrating.norm(), 0.0);
  // Our collision avoidance gradient should be enough to push the arm out of
  // collision.
  const int max_iterations = 5;
  Eigen::VectorXd qtest = Eigen::VectorXd::Zero(7);
  qtest << 0.0, M_PI_2, 0.0, -M_PI_4 * 1.25, 0.0, 0.0, 0.0;
  const Eigen::VectorXd qtest_initial = qtest;
  EXPECT_FALSE(checker.CheckConfigCollisionFree(qtest));
  log()->info("Starting q: {}", common_robotics_utilities::print::Print(qtest));
  int current_iteration = 0;
  while (current_iteration < max_iterations &&
         !checker.CheckConfigCollisionFree(qtest)) {
    // The gradient gives us a direction and magnitude in C-space, but here we
    // only use the direction, not the magnitude. In each loop iteration, we'll
    // move a smidge along that direction; the next time around we'll compute
    // a new gradient.
    const Eigen::VectorXd grad_qtest =
        ComputeCollisionAvoidanceDisplacement(checker, qtest, -0.125, 0.0);
    const Eigen::VectorXd scaled_grad_qtest =
        grad_qtest.stableNormalized() * 0.05;
    qtest = qtest + scaled_grad_qtest;
    log()->info("Iteration {}; raw grad_q: {}; scaled_grad_q: {}; new q: {}",
                current_iteration,
                common_robotics_utilities::print::Print(grad_qtest),
                common_robotics_utilities::print::Print(scaled_grad_qtest),
                common_robotics_utilities::print::Print(qtest));
    ++current_iteration;
  }
  EXPECT_TRUE(checker.CheckConfigCollisionFree(qtest));
  log()->info("Modified qtest {} to collision-free {} in {} iterations",
              common_robotics_utilities::print::Print(qtest_initial),
              common_robotics_utilities::print::Print(qtest),
              current_iteration);
}

TEST_P(CollisionCheckerAbstractTestSuite, StressParallelGradientQueries) {
  auto params = GetParam();
  const auto parallelism = Parallelism::Max();
  // Since there are parallel query options, we test repeatedly to ensure that
  // tests are not flaky.
  const int iterations = params.thread_stress_iterations;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    TestDistanceQueries(params, parallelism);
  }
}

TEST_P(CollisionCheckerAbstractTestSuite, ForceSerializeGradientQueries) {
  auto params = GetParam();
  const auto parallelism = Parallelism::None();
  TestDistanceQueries(params, parallelism);
}

}  // namespace test
}  // namespace planning
}  // namespace drake
