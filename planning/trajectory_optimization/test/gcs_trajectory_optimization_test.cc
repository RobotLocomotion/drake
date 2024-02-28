#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/geodesic_convexity.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {
namespace {

using Eigen::Map;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::Hyperrectangle;
using geometry::optimization::Intersection;
using geometry::optimization::MakeConvexSets;
using geometry::optimization::Point;
using geometry::optimization::VPolytope;
using multibody::MultibodyPlant;
using multibody::PlanarJoint;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialInertia;
using solvers::MathematicalProgram;

bool GurobiOrMosekSolverAvailable() {
  return (solvers::MosekSolver::is_available() &&
          solvers::MosekSolver::is_enabled()) ||
         (solvers::GurobiSolver::is_available() &&
          solvers::GurobiSolver::is_enabled());
}

GTEST_TEST(GcsTrajectoryOptimizationTest, Basic) {
  const int kDimension = 2;
  const double kMinimumDuration = 1.0;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add a single region (the unit box), and plan a line segment inside that
  // box.
  Vector2d start(-0.5, -0.5), goal(0.5, 0.5);

  auto& regions =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1,
                     kMinimumDuration);
  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  // Verify that the subgraphs are present in gcs trajectory optimization.
  auto all_subgraphs = gcs.GetSubgraphs();
  EXPECT_NE(std::find(all_subgraphs.begin(), all_subgraphs.end(), &regions),
            all_subgraphs.end());
  EXPECT_NE(std::find(all_subgraphs.begin(), all_subgraphs.end(), &source),
            all_subgraphs.end());
  EXPECT_NE(std::find(all_subgraphs.begin(), all_subgraphs.end(), &target),
            all_subgraphs.end());

  auto& source_to_regions = gcs.AddEdges(source, regions);
  auto& regions_to_target = gcs.AddEdges(regions, target);

  // Verify that the edges between subgraphs are present in gcs trajectory
  // optimization.
  auto all_subgraph_edges = gcs.GetEdgesBetweenSubgraphs();
  EXPECT_NE(std::find(all_subgraph_edges.begin(), all_subgraph_edges.end(),
                      &source_to_regions),
            all_subgraph_edges.end());
  EXPECT_NE(std::find(all_subgraph_edges.begin(), all_subgraph_edges.end(),
                      &regions_to_target),
            all_subgraph_edges.end());

  auto [traj, result] = gcs.SolvePath(source, target);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_EQ(traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));

  // If we would like to find another path for a different goal, we can remove
  // the target subgraph and add a new one while keeping the remaining graph.
  gcs.RemoveSubgraph(target);
  Vector2d new_goal(0.5, -0.5);
  auto& new_target = gcs.AddRegions(MakeConvexSets(Point(new_goal)), 0);
  gcs.AddEdges(regions, new_target);

  auto [new_traj, new_result] = gcs.SolvePath(source, new_target);
  EXPECT_TRUE(new_result.is_success());
  EXPECT_EQ(new_traj.rows(), 2);
  EXPECT_EQ(new_traj.cols(), 1);
  EXPECT_TRUE(
      CompareMatrices(new_traj.value(new_traj.start_time()), start, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(new_traj.value(new_traj.end_time()), new_goal, 1e-6));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, PathLengthCost) {
  const int kDimension = 2;
  GcsTrajectoryOptimization trajopt(kDimension);

  // Add the unit box.
  const int kOrder = 2;
  auto& regions = trajopt.AddRegions(
      MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), kOrder);
  regions.AddPathLengthCost(Matrix2d::Identity());

  const GraphOfConvexSets& gcs = trajopt.graph_of_convex_sets();
  EXPECT_EQ(gcs.Vertices().size(), 1);
  const GraphOfConvexSets::Vertex* v = gcs.Vertices()[0];
  EXPECT_EQ(v->ambient_dimension(), kDimension * (kOrder + 1) + 1);

  EXPECT_EQ(v->GetCosts().size(), 2);

  // Make a small mathematical program just to evaluate the bindings.
  VectorXd x = VectorXd::LinSpaced(v->ambient_dimension(), 1.23, 4.56);
  MathematicalProgram prog;
  prog.AddDecisionVariables(v->x());
  prog.SetInitialGuessForAllVariables(x);
  EXPECT_NEAR(prog.EvalBindingAtInitialGuess(v->GetCosts()[0])[0],
              (x.segment(2, 2) - x.segment(0, 2)).norm(), 1e-12);
  EXPECT_NEAR(prog.EvalBindingAtInitialGuess(v->GetCosts()[1])[0],
              (x.segment(4, 2) - x.segment(2, 2)).norm(), 1e-12);
}

GTEST_TEST(GcsTrajectoryOptimizationTest, VelocityBounds) {
  const int kDimension = 2;
  GcsTrajectoryOptimization trajopt(kDimension);

  // Add the unit box.
  const int kOrder = 2;
  auto& regions = trajopt.AddRegions(
      MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), kOrder);
  const Vector2d lb(-1, -2);
  const Vector2d ub(1, 2);
  regions.AddVelocityBounds(lb, ub);

  const GraphOfConvexSets& gcs = trajopt.graph_of_convex_sets();
  EXPECT_EQ(gcs.Vertices().size(), 1);
  const GraphOfConvexSets::Vertex* v = gcs.Vertices()[0];
  EXPECT_EQ(v->ambient_dimension(), kDimension * (kOrder + 1) + 1);

  // Make a small mathematical program just to evaluate the bindings.
  EXPECT_EQ(v->ambient_dimension(), 7);
  MatrixXd control_points(2, 3);
  // clang-format off
  control_points << 0, 1, 3,
                    4, 1, 0;
  // clang-format on
  const double time_scaling = 2.0;
  VectorXd x(7);
  x << Map<VectorXd>(control_points.data(), 6), time_scaling;
  MathematicalProgram prog;
  prog.AddDecisionVariables(v->x());
  prog.SetInitialGuessForAllVariables(x);

  EXPECT_EQ(v->GetConstraints().size(), 4);
  // dimension 0 lower bound. Should be satisfied for all times.
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(v->GetConstraints()[0]));
  // dimension 0 upper bound. The velocity in the second half of the trajectory
  // is ((3 - 1) / 0.5) / 2) = 2. That exceeds the upper bound of 1.
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(v->GetConstraints()[1]));
  // dimension 1 lower bound. The velocity in the first half of the trajectory
  // is ((1 - 4) / 0.5) / 2 = -3. That exceeds the lower bound of -2.
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(v->GetConstraints()[2]));
  // dimension 1 upper bound. Should be satisfied for all times.
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(v->GetConstraints()[3]));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, MinimumTimeVsPathLength) {
  /* This simple 2D example will show the difference between the minimum time
  and minimum path length objectives. Bob wants to get from start (S) to goal
  (G). Bob can either walk or take an e-scooter. While the e-scooter is faster
  (10 m/s in x and y), it can't drive through gravel. However, Bob can walk at 1
  m/s (in x and y) through gravel. Bob wants to find the fastest path from S to
  G.

  ░ - wall
  ▒ - gravel
  ░░░░░░░░░░░░░░░░░░░░░░░░
  ░░                    ░░
  ░░                    ░░
  ░░      ░░░░░░░░      ░░
  ░░      ░░░░░░░░      ░░
  ░░      ░░░░░░░░      ░░
  ░░  S   ▒▒▒▒▒▒▒▒   G  ░░
  ░░      ▒▒▒▒▒▒▒▒      ░░
  ░░░░░░░░░░░░░░░░░░░░░░░░
  */

  const int kDimension = 2;
  const double kWalkingSpeed = 1;
  const double kScooterSpeed = 10;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(-0.5, 0), goal(0.5, 0);

  auto& walking_regions =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeBox(Vector2d(-0.5, -0.5),
                                                         Vector2d(0.5, 0.5))),
                     3, 0, 20, "walking");
  // Bob can walk at 1 m/s in x and y.
  walking_regions.AddVelocityBounds(Vector2d(-kWalkingSpeed, -kWalkingSpeed),
                                    Vector2d(kWalkingSpeed, kWalkingSpeed));

  auto& scooter_regions = gcs.AddRegions(
      MakeConvexSets(
          HPolyhedron::MakeBox(Vector2d(-0.5, -0.5), Vector2d(-0.2, 1)),
          HPolyhedron::MakeBox(Vector2d(-0.5, 1), Vector2d(0.5, 1.5)),
          HPolyhedron::MakeBox(Vector2d(0.2, -0.5), Vector2d(0.5, 1))),
      3, 0, 20, "scooter");
  // Bob can ride an e-scooter at 10 m/s in x and y.
  scooter_regions.AddVelocityBounds(Vector2d(-kScooterSpeed, -kScooterSpeed),
                                    Vector2d(kScooterSpeed, kScooterSpeed));

  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs.AddEdges(source, walking_regions);
  gcs.AddEdges(walking_regions, target);

  gcs.AddEdges(source, scooter_regions);
  gcs.AddEdges(scooter_regions, target);

  // Add shortest path objective to compare against the minimum time objective.
  gcs.AddPathLengthCost();

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LE(gcs.EstimateComplexity(), 160);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  auto [shortest_path_traj, shortest_path_result] =
      gcs.SolvePath(source, target);
  ASSERT_TRUE(shortest_path_result.is_success());
  EXPECT_EQ(shortest_path_traj.rows(), 2);
  EXPECT_EQ(shortest_path_traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(
      shortest_path_traj.value(shortest_path_traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(
      shortest_path_traj.value(shortest_path_traj.end_time()), goal, 1e-6));

  // We expect the shortest path to be a straight line from S to G, going
  // through the gravel. Thus we expect the number of segments in the trajectory
  // to be 1 (one for the walking_regions) and the path length to be 1.
  EXPECT_EQ(shortest_path_traj.get_number_of_segments(), 1);

  const double kExpectedShortestPathLength = 1.0;
  double shortest_path_length = 0.0;
  const double dt = 0.01;  // Time step for numerical integration.

  for (double t = shortest_path_traj.start_time();
       t < shortest_path_traj.end_time(); t += dt) {
    const double t_next = std::min(t + dt, shortest_path_traj.end_time());
    const double dx =
        (shortest_path_traj.value(t_next) - shortest_path_traj.value(t)).norm();
    shortest_path_length += dx;
  }
  EXPECT_NEAR(shortest_path_length, kExpectedShortestPathLength, 1e-6);

  const double shortest_path_duration =
      shortest_path_traj.end_time() - shortest_path_traj.start_time();

  // Bob is having too much fun on the scooter and doesn't want to get off.
  // He has a clear discrete path in mind that he firmly believes in, but still
  // wants to find the shortest path.
  // We can manually specify the vertices Bob wants to visit and solve the
  // convex restriction.
  const auto source_vertices = source.Vertices();
  const auto scooter_vertices = scooter_regions.Vertices();
  const auto target_vertices = target.Vertices();

  std::vector<const geometry::optimization::GraphOfConvexSets::Vertex*>
      active_vertices;
  std::copy(source_vertices.begin(), source_vertices.end(),
            std::back_inserter(active_vertices));
  std::copy(scooter_vertices.begin(), scooter_vertices.end(),
            std::back_inserter(active_vertices));
  std::copy(target_vertices.begin(), target_vertices.end(),
            std::back_inserter(active_vertices));

  auto [detour_traj, detour_result] =
      gcs.SolveConvexRestriction(active_vertices);
  EXPECT_TRUE(detour_result.is_success());
  EXPECT_EQ(detour_traj.rows(), 2);
  EXPECT_EQ(detour_traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(detour_traj.value(detour_traj.start_time()),
                              start, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(detour_traj.value(detour_traj.end_time()), goal, 1e-6));

  // The detour should be longer than the shortest path.
  double detour_path_length = 0.0;
  for (double t = detour_traj.start_time(); t < detour_traj.end_time();
       t += dt) {
    const double t_next = std::min(t + dt, detour_traj.end_time());
    const double dx = (detour_traj.value(t_next) - detour_traj.value(t)).norm();
    detour_path_length += dx;
  }
  EXPECT_GT(detour_path_length, shortest_path_length);

  // Now we want to find the fastest path from S to G.
  // Add minimum time objective with a very high weight to overpower the
  // shortest path objective.
  gcs.AddTimeCost(1e6);

  auto [fastest_path_traj, fastest_path_result] = gcs.SolvePath(source, target);
  EXPECT_TRUE(fastest_path_result.is_success());
  EXPECT_EQ(fastest_path_traj.rows(), 2);
  EXPECT_EQ(fastest_path_traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(
      fastest_path_traj.value(fastest_path_traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(
      fastest_path_traj.value(fastest_path_traj.end_time()), goal, 1e-6));

  // We expect the fastest path to take a detour and avoid the going through the
  // gravel. Thus we expect the number of segments in the trajectory to be 3
  // (three for the scooter_regions) and the path length to be greater than 2.4
  // (the minimum path length for the detour).
  EXPECT_EQ(fastest_path_traj.get_number_of_segments(), 3);

  const double kLeastExpectedFastestPathLength = 2.4;
  double fastest_path_length = 0.0;

  // During the path integration, we will also check that the velocity bounds
  // are consistently hit.
  auto fastest_path_velocity = fastest_path_traj.MakeDerivative();
  for (double t = fastest_path_traj.start_time();
       t < fastest_path_traj.end_time(); t += dt) {
    const double t_next = std::min(t + dt, fastest_path_traj.end_time());
    const double dx =
        (fastest_path_traj.value(t_next) - fastest_path_traj.value(t)).norm();

    // We also expect the fastest path to consistently hit either of the
    // velocity bounds.
    EXPECT_TRUE((fastest_path_velocity->value(t).cwiseAbs() -
                 Vector2d(kScooterSpeed, kScooterSpeed))
                    .cwiseAbs()
                    .minCoeff() < 1e-6);
    fastest_path_length += dx;
  }
  EXPECT_GT(fastest_path_length, kLeastExpectedFastestPathLength);
  // We expect the fastest path to be longer than the shortest path.
  EXPECT_GT(fastest_path_length, shortest_path_length);

  // The fastest path should be faster than the shortest path.
  const double fastest_path_duration =
      fastest_path_traj.end_time() - fastest_path_traj.start_time();
  EXPECT_LT(fastest_path_duration, shortest_path_duration);

  // Oh oh, it has been raining a lot! The street has been flooded and Bob
  // can't take the scooter. We can remove the scooter regions from the graph
  // and solve the path again.
  gcs.RemoveSubgraph(scooter_regions);

  auto [rain_path_traj, rain_path_result] = gcs.SolvePath(source, target);
  EXPECT_TRUE(rain_path_result.is_success());
  EXPECT_EQ(rain_path_traj.rows(), 2);
  EXPECT_EQ(rain_path_traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(rain_path_traj.value(rain_path_traj.start_time()),
                              start, 1e-6));
  EXPECT_TRUE(CompareMatrices(rain_path_traj.value(rain_path_traj.end_time()),
                              goal, 1e-6));

  // Since the scooter regions have been removed, Bob can only walk through the
  // through the gravel.
  std::vector<const geometry::optimization::GraphOfConvexSets::Edge*>
      expected_walking_path_edges = {
          walking_regions.Vertices()[0]->incoming_edges()[0],
          walking_regions.Vertices()[0]->outgoing_edges()[0]};

  std::vector<const geometry::optimization::GraphOfConvexSets::Edge*>
      rain_path_edges = gcs.graph_of_convex_sets().GetSolutionPath(
          *source.Vertices()[0], *target.Vertices()[0], rain_path_result, 1.0);
  for (size_t i = 0; i < rain_path_edges.size(); i++) {
    EXPECT_EQ(rain_path_edges[i], expected_walking_path_edges[i]);
  }
}

GTEST_TEST(GcsTrajectoryOptimizationTest, VelocityBoundsOnEdges) {
  /* This simple 2D example will test the velocity bound constraints on edges,
  and illustrate and example with delays. Bob is a drag racer who wants to beat
  the world record for the 305m strip. No rolling start is allowed, so the car
  has to have zero velocity in the beginning. However, he can drive as fast as
  his car allows through the finish line (50 m/s). There is only one problem,
  this drag strip is known to get crossed by ducks, so Bob has to be careful not
  to hit any ducks. It takes the ducks about 10 seconds to cross the track, and
  Bob has to wait for them.
  */
  // clang-format off
  /*    305m                                        75m             🚥
  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
  ░░    🏁                                          🦆               |   ░░
  ░░    🏁                                          🦆               |🏎️ ░░
  ░░    🏁                                          🦆               |   ░░
  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
  */
  // clang-format on

  const int kDimension = 2;
  const double kMaxSpeed = 50;   // m/s
  const double kDuckDelay = 10;  // s
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0, 0), goal(305, 0);

  // We need to duplicate the race track regions to sandwich the duck regions.
  auto& race_track_1 = gcs.AddRegions(
      MakeConvexSets(HPolyhedron::MakeBox(Vector2d(0, -5), Vector2d(305, 5))),
      3);
  auto& race_track_2 = gcs.AddRegions(
      MakeConvexSets(HPolyhedron::MakeBox(Vector2d(0, -5), Vector2d(305, 5))),
      3);
  // Bob's car can only drive straight. So he can drive at 50 m/s forward in x,
  // but not in y.
  race_track_1.AddVelocityBounds(Vector2d(0, 0), Vector2d(kMaxSpeed, 0));
  race_track_2.AddVelocityBounds(Vector2d(0, 0), Vector2d(kMaxSpeed, 0));

  // The ducks are cross the track at the 75m mark and they need at least 10
  // seconds.
  auto& ducks = gcs.AddRegions(
      MakeConvexSets(HPolyhedron::MakeBox(Vector2d(75, -5), Vector2d(75, 5))),
      0, kDuckDelay, kDuckDelay + 10);

  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  auto& source_to_race_track_1 = gcs.AddEdges(source, race_track_1);
  auto& race_track_1_to_ducks = gcs.AddEdges(race_track_1, ducks);
  auto& ducks_to_race_track_2 = gcs.AddEdges(ducks, race_track_2);
  gcs.AddEdges(race_track_2, target);

  // Bob's car has to start with zero velocity.
  source_to_race_track_1.AddVelocityBounds(Vector2d(0, 0), Vector2d(0, 0));

  // Bob doesn't want to run the ducks over, so he has to stop before the ducks.
  // Hence the car will be at zero velocity.
  race_track_1_to_ducks.AddVelocityBounds(Vector2d(0, 0), Vector2d(0, 0));
  ducks_to_race_track_2.AddVelocityBounds(Vector2d(0, 0), Vector2d(0, 0));

  // Bob wants to beat the time record!
  gcs.AddTimeCost();

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LT(gcs.EstimateComplexity(), 125);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  auto [traj, result] = gcs.SolvePath(source, target);

  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_EQ(traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));

  // We expect the fastest path to drive through the race track regions and wait
  // at the duck region. Thus we expect the number of segments in the trajectory
  // to be 3 (one for each race track and one for the ducks).
  EXPECT_EQ(traj.get_number_of_segments(), 3);

  // The initial velocity should be zero and the final velocity should be at the
  // maximum.
  auto traj_vel = traj.MakeDerivative();
  EXPECT_TRUE(CompareMatrices(traj_vel->value(traj.start_time()),
                              Vector2d(0, 0), 1e-6));
  EXPECT_TRUE(CompareMatrices(traj_vel->value(traj.end_time()),
                              Vector2d(kMaxSpeed, 0), 1e-6));

  // The total duration should be at least the duck delay and the minimum time
  // it would take to drive the track down at maximum speed.
  // kDuckDelay + 305m /kMaxSpeed.
  EXPECT_GE(traj.end_time() - traj.start_time(), kDuckDelay + 305 / kMaxSpeed);

  // Let's verify that the Bob didn't run over the ducks!
  double stopped_at_ducks_time = 0;
  const double kTimeStep = 0.01;
  for (double t = traj.start_time(); t < traj.end_time(); t += kTimeStep) {
    if (traj.value(t)(0) >= 75) {
      stopped_at_ducks_time = t;
      break;
    }
  }

  // The car should be stopped at the ducks for kDuckDelay seconds.
  EXPECT_TRUE(CompareMatrices(traj_vel->value(stopped_at_ducks_time),
                              Vector2d(0, 0), 1e-6));

  EXPECT_TRUE(
      CompareMatrices(traj_vel->value(stopped_at_ducks_time + kDuckDelay),
                      Vector2d(0, 0), 1e-6));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, RemoveSubgraph) {
  /*Ensure that removing a subgraph actually removes vertices/edges in the gcs*/
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  auto& graph1 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension),
                                    HPolyhedron::MakeUnitBox(kDimension)),
                     1);
  auto& graph2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension),
                                    HPolyhedron::MakeUnitBox(kDimension)),
                     1);
  gcs.AddEdges(graph1, graph2);

  // We expect four vertices, since there are two regions per subgraph.
  const int kExpectedVertices = 4;
  EXPECT_EQ(gcs.graph_of_convex_sets().Vertices().size(), kExpectedVertices);
  // The regions in the subgraph are connected via two edges and since they
  // overlap, another four edges are added between the subgraphs.
  const int kExpectedEdges = 2 * 2 + 4;
  EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), kExpectedEdges);

  // After removing both subgraphs, we expect no vertices or edges in the gcs.
  gcs.RemoveSubgraph(graph1);
  gcs.RemoveSubgraph(graph2);
  EXPECT_EQ(gcs.graph_of_convex_sets().Vertices().size(), 0);
  EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), 0);
  EXPECT_EQ(gcs.GetSubgraphs().size(), 0);
  EXPECT_EQ(gcs.GetEdgesBetweenSubgraphs().size(), 0);

  // Create to remove a subgraph that hasn't been associated with the gcs
  // instance. For that, we will create another gcs object with a separate
  // subgraph and try to remove it from the previous instance.
  GcsTrajectoryOptimization gcs2(kDimension);

  auto& graph3 =
      gcs2.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension),
                                     HPolyhedron::MakeUnitBox(kDimension)),
                      1);
  DRAKE_EXPECT_THROWS_MESSAGE(gcs.RemoveSubgraph(graph3),
                              ".* is not registered with `this`.*");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, InvalidPositions) {
  /* Positions passed into GcsTrajectoryOptimization must be greater than 0.*/
  DRAKE_EXPECT_THROWS_MESSAGE(GcsTrajectoryOptimization(0),
                              ".*num_positions.*");
  DRAKE_EXPECT_THROWS_MESSAGE(GcsTrajectoryOptimization(-1),
                              ".*num_positions.*");
  DRAKE_EXPECT_NO_THROW(GcsTrajectoryOptimization(1));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, InvalidOrder) {
  /* Subgraph order must be non-negative.*/
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), -1),
      ".*order.*");

  DRAKE_EXPECT_NO_THROW(
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, ZeroOrderPathLengthCost) {
  /* Path length cost is not defined for a set of order 0.*/
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  auto& regions =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      regions.AddPathLengthCost(),
      "Path length cost is not defined for a set of order 0.");
  // This should consider the order of the subgraphs and not add the path length
  // cost to the regions.
  DRAKE_EXPECT_NO_THROW(gcs.AddPathLengthCost());
}

GTEST_TEST(GcsTrajectoryOptimizationTest, InvalidVelocityBounds) {
  /* The velocity of a curve is not defined for a subgraph of order 0.*/
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  auto& regions1 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  auto& regions2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  auto& regions1_to_regions2 = gcs.AddEdges(regions1, regions2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero()),
      "Velocity Bounds are not defined for a set of order 0.");
  // This should consider the order of the subgraphs and not add the velocity
  // bounds to the regions.
  DRAKE_EXPECT_NO_THROW(
      gcs.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero()));

  // lower and upper bound must have the same dimension as the graph.
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1.AddVelocityBounds(Vector3d::Zero(), Vector2d::Zero()),
      ".*size.*.num_positions.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddVelocityBounds(Vector2d::Zero(), Vector3d::Zero()),
      ".*size.*.num_positions.*");
  DRAKE_EXPECT_THROWS_MESSAGE(regions1_to_regions2.AddVelocityBounds(
                                  Vector2d::Zero(), Vector3d::Zero()),
                              ".*size.*.num_positions.*");

  // Can't add velocity bounds to an edge if both regions have order 0.
  DRAKE_EXPECT_THROWS_MESSAGE(regions1_to_regions2.AddVelocityBounds(
                                  Vector2d::Zero(), Vector2d::Zero()),
                              "Cannot add velocity bounds to a subgraph edges "
                              "where both subgraphs have zero order.");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, InvalidContinuityConstraints) {
  /* The velocity of a curve is not defined for a subgraph of order 0.*/
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  auto& regions1 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1);

  auto& regions2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1);

  auto& regions1_to_regions2 = gcs.AddEdges(regions1, regions2);

  // Zero order path continuity should throw an error, since its support by
  // default.
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1.AddPathContinuityConstraints(0),
      "Path continuity is enforced by default. Choose a higher order.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1_to_regions2.AddPathContinuityConstraints(0),
      "Path continuity is enforced by default. Choose a higher order.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddPathContinuityConstraints(0),
      "Path continuity is enforced by default. Choose a higher order.");

  // Negative continuity should be rejected as well.
  DRAKE_EXPECT_THROWS_MESSAGE(regions1.AddPathContinuityConstraints(-1),
                              "Order must be greater than or equal to 1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1_to_regions2.AddPathContinuityConstraints(-1),
      "Order must be greater than or equal to 1.");
  DRAKE_EXPECT_THROWS_MESSAGE(gcs.AddPathContinuityConstraints(-1),
                              "Order must be greater than or equal to 1.");

  // Adding global continuity constraints should consider the order of the
  // subgraphs and edges between subgraphs. Thus very large continuity orders
  // shouldn't be rejected.
  DRAKE_EXPECT_NO_THROW(gcs.AddPathContinuityConstraints(100));

  // Since the order of region1 is 1, velocity continuity should be support.
  DRAKE_EXPECT_NO_THROW(regions1.AddPathContinuityConstraints(1));
  // But acceleration continuity would require the region to be of order 2.
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1.AddPathContinuityConstraints(2),
      "Cannot add continuity constraint of order greater than the set "
      "order.");

  // Similarly the edges between subgraphs allow for velocity but no
  // acceleration continuity.
  DRAKE_EXPECT_NO_THROW(regions1_to_regions2.AddPathContinuityConstraints(1));
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions1_to_regions2.AddPathContinuityConstraints(2),
      "Cannot add continuity constraint to a subgraph edge where both "
      "subgraphs order are not greater than or equal to the requested "
      "continuity order.");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, DisjointGraph) {
  /* Adds four points to the graph without any edges. */
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start1(0.2, 0.2), start2(0.3, 3.2);
  Vector2d goal1(4.8, 4.8), goal2(4.9, 2.4);

  auto& source =
      gcs.AddRegions(MakeConvexSets(Point(start1), Point(start2)), 0);

  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal1), Point(goal2)), 0);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  // Define solver options.
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 3;

  auto [traj, result] = gcs.SolvePath(source, target, options);

  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(traj.get_number_of_segments(), 0);
}

GTEST_TEST(GcsTrajectoryOptimizationTest, DisconnectedPathInConvexRestriction) {
  /* Add two subgraphs to the graph without connecting them.*/
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start1(0.2, 0.2), start2(0.3, 3.2);
  Vector2d goal1(4.8, 4.8), goal2(4.9, 2.4);

  auto& source = gcs.AddRegions(
      MakeConvexSets(HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(1, 1)),
                     HPolyhedron::MakeBox(Vector2d(0.5, 0.5), Vector2d(2, 2))),
      0);

  auto& target = gcs.AddRegions(
      MakeConvexSets(HPolyhedron::MakeBox(Vector2d(3, 3), Vector2d(4, 4)),
                     HPolyhedron::MakeBox(Vector2d(3.5, 3.5), Vector2d(5, 5))),
      0);

  // Retrieve the vertices of the source and target subgraphs and try to solve
  // the convex restriction.
  const auto source_vertices = source.Vertices();
  const auto target_vertices = target.Vertices();

  std::vector<const geometry::optimization::GraphOfConvexSets::Vertex*>
      active_vertices;
  std::copy(source_vertices.begin(), source_vertices.end(),
            std::back_inserter(active_vertices));
  std::copy(target_vertices.begin(), target_vertices.end(),
            std::back_inserter(active_vertices));

  // Since we did not use the AddEdges method to connect the subgraphs, the
  // vertices from the source and target subgraphs are not connected.
  // This should result in an error.
  DRAKE_EXPECT_THROWS_MESSAGE(gcs.SolveConvexRestriction(active_vertices),
                              ".*is not connected to vertex.*");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, MultipleEdgesInConvexRestriction) {
  /* Solving the convex restriction should through an error if there are
  multiple edges between any two vertices. This can happen if a user tries to
  connect two subgraphs with different subspace constraints. */

  const int kDimension = 2;
  const double kMinimumDuration = 1.0;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add a single region (the unit box), and plan a line segment inside that
  // box. We will then add two points to the graph
  /*
  Consider a simple planning problem to find a path from start to goal while
  either going through point 1 or point 2. We will be using subspaces to
  enforce the point constraint, resulting in two edges between the unit regions.

           ┌────────┐ Point 1 ┌────────┐
           │        ├────────►│        │
  Start───►│  Unit  │         │  Unit  ├───►Goal
           │ Region │ Point 2 │ Region │
           │        ├────────►│        │
           └────────┘         └────────┘
  */

  Vector2d start(-0.5, -0.5), goal(0.5, 0.5);
  Point subspace1(Vector2d(0.0, 0.0)), subspace2(Vector2d(0.2, 0.2));

  auto& graph1 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1,
                     kMinimumDuration);
  auto& graph2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1,
                     kMinimumDuration);

  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs.AddEdges(source, graph1);
  gcs.AddEdges(graph1, graph2, &subspace1);
  gcs.AddEdges(graph1, graph2, &subspace2);
  gcs.AddEdges(graph2, target);

  // Retrieve the vertices of the subgraphs and try to solve the convex
  // restriction.
  const auto source_vertices = source.Vertices();
  const auto graph1_vertices = graph1.Vertices();
  const auto graph2_vertices = graph2.Vertices();
  const auto target_vertices = target.Vertices();

  std::vector<const geometry::optimization::GraphOfConvexSets::Vertex*>
      active_vertices;
  std::copy(source_vertices.begin(), source_vertices.end(),
            std::back_inserter(active_vertices));
  std::copy(graph1_vertices.begin(), graph1_vertices.end(),
            std::back_inserter(active_vertices));
  std::copy(graph2_vertices.begin(), graph2_vertices.end(),
            std::back_inserter(active_vertices));
  std::copy(target_vertices.begin(), target_vertices.end(),
            std::back_inserter(active_vertices));

  // Since the vertex in graph1 and graph2 are connected by multiple edges, we
  // expect the convex restriction to throw an error.
  DRAKE_EXPECT_THROWS_MESSAGE(gcs.SolveConvexRestriction(active_vertices),
                              ".*through multiple edges.*");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, InvalidSubspace) {
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  auto& regions1 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  auto& regions2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  auto subspace_wrong_dimension = HPolyhedron::MakeUnitBox(kDimension + 1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddEdges(regions1, regions2, &subspace_wrong_dimension),
      "Subspace dimension must match the number of positions.");

  Eigen::Matrix<double, 2, 4> vertices;
  vertices << 5.0, 5.0, 4.4, 4.4, 2.8, 5.0, 5.0, 2.8;
  VPolytope vertices_subspace{vertices};

  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddEdges(regions1, regions2, &vertices_subspace),
      "Subspace must be a Point or HPolyhedron.");
}

/* This 2D environment has been presented in the GCS paper.
░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
░░      ░░░░░░░░                                      ░░░░░░░░         ░░
░░      ░░░░░░░░                                      ░░░░░░░░         ░░
░░      ░░░░░░░░                                      ░░░░░░░░    G    ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░                       ░░
░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░                       ░░
░░                                 ░░░░░░░░░░░░░      ░░░░░░░░░░░░░░░░░░░
░░                          ░░░░░░░░░░░░░░░░░░░░                       ░░
░░                        ░░░░░░░░░░░░░░░░░░░░░░░░░                    ░░
░░      ░░░░░░░░        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░                  ░░
░░      ░░░░░░░░          ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░                ░░
░░      ░░░░░░░░            ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░              ░░
░░      ░░░░░░░░              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░            ░░
░░      ░░░░░░░░                ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░          ░░
░░      ░░░░░░░░                  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░        ░░
░░      ░░░░░░░░                    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░      ░░
░░      ░░░░░░░░                      ░░░░░░░░░░░░░░░░░░░░░░░░░        ░░
░░      ░░░░░░░░                        ░░░░░░░░░░░░░░░░░░░░░          ░░
░░      ░░░░░░░░                          ░░░░░░░░░░░░░░░░░            ░░
░░      ░░░░░░░░                            ░░░░░░░░░░░░░              ░░
░░      ░░░░░░░░                                                       ░░
░░  S   ░░░░░░░░                                                       ░░
░░      ░░░░░░░░                                                       ░░
░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
*/

class SimpleEnv2D : public ::testing::Test {
 protected:
  SimpleEnv2D() {
    Eigen::Matrix<double, 2, 4> vertices_1;
    // clang-format off
    vertices_1 << 0.4, 0.4, 0.0, 0.0,
                  0.0, 5.0, 5.0, 0.0;
    // clang-format on
    HPolyhedron region_1{VPolytope(vertices_1)};

    Eigen::Matrix<double, 2, 4> vertices_2;
    // clang-format off
    vertices_2 << 0.4, 1.0, 1.0, 0.4,
                  2.4, 2.4, 2.6, 2.6;
    // clang-format on
    HPolyhedron region_2{VPolytope(vertices_2)};

    Eigen::Matrix<double, 2, 4> vertices_3;
    // clang-format off
    vertices_3 << 1.4, 1.4, 1.0, 1.0,
                  2.2, 4.6, 4.6, 2.2;
    // clang-format on
    HPolyhedron region_3{VPolytope(vertices_3)};

    Eigen::Matrix<double, 2, 4> vertices_4;
    // clang-format off
    vertices_4 << 1.4, 2.4, 2.4, 1.4,
                  2.2, 2.6, 2.8, 2.8;
    // clang-format on
    HPolyhedron region_4{VPolytope(vertices_4)};

    Eigen::Matrix<double, 2, 4> vertices_5;
    // clang-format off
    vertices_5 << 2.2, 2.4, 2.4, 2.2,
                  2.8, 2.8, 4.6, 4.6;
    // clang-format on
    HPolyhedron region_5{VPolytope(vertices_5)};

    Eigen::Matrix<double, 2, 5> vertices_6;
    // clang-format off
    vertices_6 << 1.4, 1., 1., 3.8, 3.8,
                  2.2, 2.2, 0.0, 0.0, 0.2;
    // clang-format on
    HPolyhedron region_6{VPolytope(vertices_6)};

    Eigen::Matrix<double, 2, 4> vertices_7;
    // clang-format off
    vertices_7 << 3.8, 3.8, 1.0, 1.0,
                  4.6, 5.0, 5.0, 4.6;
    // clang-format on
    HPolyhedron region_7{VPolytope(vertices_7)};

    Eigen::Matrix<double, 2, 5> vertices_8;
    // clang-format off
    vertices_8 << 5.0, 5.0, 4.8, 3.8, 3.8,
                  0.0, 1.2, 1.2, 0.2, 0.0;
    // clang-format on
    HPolyhedron region_8{VPolytope(vertices_8)};

    Eigen::Matrix<double, 2, 4> vertices_9;
    // clang-format off
    vertices_9 << 3.4, 4.8, 5.0, 5.0,
                  2.6, 1.2, 1.2, 2.6;
    // clang-format on
    HPolyhedron region_9{VPolytope(vertices_9)};

    Eigen::Matrix<double, 2, 4> vertices_10;
    // clang-format off
    vertices_10 << 3.4, 3.8, 3.8, 3.4,
                   2.6, 2.6, 4.6, 4.6;
    // clang-format on
    HPolyhedron region_10{VPolytope(vertices_10)};

    Eigen::Matrix<double, 2, 4> vertices_11;
    // clang-format off
    vertices_11 << 3.8, 4.4, 4.4, 3.8,
                   2.8, 2.8, 3.0, 3.0;
    // clang-format on
    HPolyhedron region_11{VPolytope(vertices_11)};

    Eigen::Matrix<double, 2, 4> vertices_12;
    // clang-format off
    vertices_12 << 5.0, 5.0, 4.4, 4.4,
                   2.8, 5.0, 5.0, 2.8;
    // clang-format on
    HPolyhedron region_12{VPolytope(vertices_12)};

    regions_ = MakeConvexSets(region_1, region_2, region_3, region_4, region_5,
                              region_6, region_7, region_8, region_9, region_10,
                              region_11, region_12);
  }
  ConvexSets regions_;
};

TEST_F(SimpleEnv2D, BasicShortestPath) {
  const int kDimension = 2;
  const double kMinimumDuration = 1.0;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 1, kMinimumDuration);
  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

  gcs.AddPathLengthCost();

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LT(gcs.EstimateComplexity(), 1e3);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  // Define solver options.
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 3;

  auto [traj, result] = gcs.SolvePath(source, target, options);

  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));
}

TEST_F(SimpleEnv2D, GlobalContinuityConstraints) {
  const int kDimension = 2;
  const double kSpeed = 1.0;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 6);
  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0, 0.0, 0.0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0, 0.0, 0.0);

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

  gcs.AddPathLengthCost();
  gcs.AddTimeCost();
  gcs.AddVelocityBounds(Vector2d(-kSpeed, -kSpeed), Vector2d(kSpeed, kSpeed));

  // Add velocity and acceleration continuity on r(s).
  gcs.AddPathContinuityConstraints(1);
  gcs.AddPathContinuityConstraints(2);

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LT(gcs.EstimateComplexity(), 2.5e3);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  // Define solver options.
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 5;

  auto [traj, result] = gcs.SolvePath(source, target, options);

  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));

  auto normalized_traj = GcsTrajectoryOptimization::NormalizeSegmentTimes(traj);
  EXPECT_TRUE(CompareMatrices(
      normalized_traj.value(normalized_traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(normalized_traj.value(normalized_traj.end_time()),
                              goal, 1e-6));
  EXPECT_EQ(normalized_traj.get_number_of_segments(),
            traj.get_number_of_segments());
  EXPECT_EQ(normalized_traj.start_time(), traj.start_time());
  // Since each segment will be one second long, we expect the number of
  // segments to match the duration of the normalized trajectory.
  EXPECT_EQ(normalized_traj.get_number_of_segments(),
            normalized_traj.end_time() - normalized_traj.start_time());

  // Check for velocity and acceleration continuity.
  for (int i = 0; i < normalized_traj.get_number_of_segments() - 1; ++i) {
    auto normalized_segment_vel = normalized_traj.segment(i).MakeDerivative();
    auto normalized_next_segment_vel =
        normalized_traj.segment(i + 1).MakeDerivative();
    EXPECT_TRUE(CompareMatrices(
        normalized_segment_vel->value(normalized_segment_vel->end_time()),
        normalized_next_segment_vel->value(
            normalized_next_segment_vel->start_time()),
        1e-6));

    auto normalized_segment_acc = normalized_traj.segment(i).MakeDerivative(2);
    auto normalized_next_segment_acc =
        normalized_traj.segment(i + 1).MakeDerivative(2);
    EXPECT_TRUE(CompareMatrices(
        normalized_segment_acc->value(normalized_segment_acc->end_time()),
        normalized_next_segment_acc->value(
            normalized_next_segment_acc->start_time()),
        1e-6));
  }
}

TEST_F(SimpleEnv2D, DurationDelay) {
  /* The durations bounds in a subgraph can be used to enforce delays.*/
  const int kDimension = 2;
  const double kMinimumDuration = 1.0;
  const double kStartDelay = 10;
  const double kGoalDelay = 20;

  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add path length cost to whole graph before adding regions.
  // New added regions will inherit this cost.
  gcs.AddPathLengthCost();

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 1, kMinimumDuration);
  // Setting h_min = h_max = kStartDelay seconds will force to stay a the start
  // of the trajectory for kStartDelay seconds.
  auto& source =
      gcs.AddRegions(MakeConvexSets(Point(start)), 0, kStartDelay, kStartDelay);
  // Similarly for the goal, but with a delay of kGoalDelay seconds.
  auto& target =
      gcs.AddRegions(MakeConvexSets(Point(goal)), 0, kGoalDelay, kGoalDelay);

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LT(gcs.EstimateComplexity(), 1e3);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  // Define solver options.
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 3;

  auto [traj, result] = gcs.SolvePath(source, target, options);

  EXPECT_TRUE(result.is_success());
  // The trajectory should stay at the start for kStartDelay seconds.
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(kStartDelay), start, 1e-6));

  // The trajectory should stay at the goal for kGaolDelay seconds.
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(traj.value(traj.end_time() - kGoalDelay), goal, 1e-6));

  // The total trajectory duration should be at least kStartDelay + kGoalDelay.
  EXPECT_GE(traj.end_time() - traj.start_time(), kStartDelay + kGoalDelay);
}

TEST_F(SimpleEnv2D, MultiStartGoal) {
  /* Planning between subgraphs also enables to select the shortest path
  between multiple start and goal points.
  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
  ░░      ░░░░░░░░                                      ░░░░░░░░         ░░
  ░░      ░░░░░░░░                                      ░░░░░░░░         ░░
  ░░      ░░░░░░░░                                      ░░░░░░░░    G1   ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░   S2 ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░                       ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░                       ░░
  ░░                                 ░░░░░░░░░░░░░      ░░░░░░░░░░░░░░░░░░░
  ░░                          ░░░░░░░░░░░░░░░░░░░░                       ░░
  ░░                        ░░░░░░░░░░░░░░░░░░░░░░░░░                 G2 ░░
  ░░      ░░░░░░░░        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░                  ░░
  ░░      ░░░░░░░░          ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░                ░░
  ░░      ░░░░░░░░            ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░              ░░
  ░░      ░░░░░░░░              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░            ░░
  ░░      ░░░░░░░░                ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░          ░░
  ░░      ░░░░░░░░                  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░        ░░
  ░░      ░░░░░░░░                    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░      ░░
  ░░      ░░░░░░░░                      ░░░░░░░░░░░░░░░░░░░░░░░░░        ░░
  ░░      ░░░░░░░░                        ░░░░░░░░░░░░░░░░░░░░░          ░░
  ░░      ░░░░░░░░                          ░░░░░░░░░░░░░░░░░            ░░
  ░░      ░░░░░░░░                            ░░░░░░░░░░░░░              ░░
  ░░      ░░░░░░░░                                                       ░░
  ░░  S1  ░░░░░░░░                                                       ░░
  ░░      ░░░░░░░░                                                       ░░
  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
  */
  const int kDimension = 2;

  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start1(0.2, 0.2), start2(0.3, 3.2);
  Vector2d goal1(4.8, 4.8), goal2(4.9, 2.4);
  const double kSpeed = 1.0;

  auto& regions = gcs.AddRegions(regions_, 3);
  auto& source =
      gcs.AddRegions(MakeConvexSets(Point(start1), Point(start2)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal1), Point(goal2)), 0);

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

  gcs.AddVelocityBounds(Vector2d(-kSpeed, -kSpeed), Vector2d(kSpeed, kSpeed));

  gcs.AddPathLengthCost();
  gcs.AddTimeCost();

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LT(gcs.EstimateComplexity(), 1e3);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  // Define solver options.
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 3;

  auto [traj, result] = gcs.SolvePath(source, target, options);

  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start2, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal2, 1e-6));
}

TEST_F(SimpleEnv2D, IntermediatePoint) {
  /* Connecting two subgraphs with a subspace enables start to
  goal planning with a desired intermediate point, I, somewhere along the path
  or with a point contained in the subspace (lower right corner).
  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
  ░░      ░░░░░░░░                                      ░░░░░░░░         ░░
  ░░      ░░░░░░░░                                      ░░░░░░░░         ░░
  ░░      ░░░░░░░░                                      ░░░░░░░░    G    ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░ I ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░      ░░░░░░░░         ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░                       ░░
  ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░░                       ░░
  ░░                                 ░░░░░░░░░░░░░      ░░░░░░░░░░░░░░░░░░░
  ░░                          ░░░░░░░░░░░░░░░░░░░░                       ░░
  ░░                        ░░░░░░░░░░░░░░░░░░░░░░░░░                    ░░
  ░░      ░░░░░░░░        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░                  ░░
  ░░      ░░░░░░░░          ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░                ░░
  ░░      ░░░░░░░░            ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░              ░░
  ░░      ░░░░░░░░              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░            ░░
  ░░      ░░░░░░░░                ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░          ░░
  ░░      ░░░░░░░░                  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░        ░░
  ░░      ░░░░░░░░                    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░xxxxxx░░
  ░░      ░░░░░░░░                      ░░░░░░░░░░░░░░░░░░░░░░░░░xxxxxxxx░░
  ░░      ░░░░░░░░                        ░░░░░░░░░░░░░░░░░░░░░xxxxxxxxxx░░
  ░░      ░░░░░░░░                          ░░░░░░░░░░░░░░░░░xxxxxxxxxxxx░░
  ░░      ░░░░░░░░                            ░░░░░░░░░░░░░xxx Subspace x░░
  ░░      ░░░░░░░░                                       xxxxxxxxxxxxxxxx░░
  ░░  S   ░░░░░░░░                                       xxxxxxxxxxxxxxxx░░
  ░░      ░░░░░░░░                                       xxxxxxxxxxxxxxxx░░
  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
  */
  const int kDimension = 2;

  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0.2, 0.2), goal(4.8, 4.8), intermediate(2.3, 3.5);
  const double kSpeed = 1.0;

  // Both a Point and an HPolytope are valid subspaces.
  Point subspace_point(intermediate);

  Eigen::Matrix<double, 2, 5> vertices;
  // clang-format off
  vertices << 5.0, 5.0, 4.8, 3.8, 3.8,
              0.0, 1.2, 1.2, 0.2, 0.0;
  // clang-format on
  HPolyhedron subspace_region{VPolytope(vertices)};

  // We can have different order subgraphs.
  // The trajectory from start to either subspace or intermidiate point will be
  // of order 3 and the trajectory from intermediate to goal will be of order 2.
  auto& main1 = gcs.AddRegions(regions_, 3, 1e-6, 20, "main1");
  auto& main2 = gcs.AddRegions(regions_, 2, 1e-6, 20, "main2");

  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  // The following wiring will give GCS the choice to either go
  // through subspace point or the subspace region.
  auto& source_to_main = gcs.AddEdges(source, main1);
  // Connect the two subgraphs through the intermediate point.
  auto& main1_to_main2_pt = gcs.AddEdges(main1, main2, &subspace_point);
  // Connect the two subgraphs through the subspace region.
  auto& main1_to_main2_region = gcs.AddEdges(main1, main2, &subspace_region);
  auto& main2_to_target = gcs.AddEdges(main2, target);

  // Add zero velocity constraints to the source, target and the intermediate
  // point and region.
  source_to_main.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero());
  main1_to_main2_pt.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero());
  main1_to_main2_region.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero());
  main2_to_target.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero());

  // We can add different costs and constraints to the individual subgraphs.
  main1.AddPathLengthCost(5);
  main1.AddTimeCost(1);
  main1.AddVelocityBounds(Vector2d(-kSpeed, -kSpeed), Vector2d(kSpeed, kSpeed));

  // This weight matrix penalizes movement in the y direction three times more
  // than in the x direction.
  Eigen::MatrixXd weight_matrix =
      Eigen::MatrixXd::Identity(gcs.num_positions(), gcs.num_positions());
  weight_matrix(1, 1) = 3.0;
  main2.AddPathLengthCost(weight_matrix);

  // Nonregression bound on the complexity of the underlying GCS MICP.
  EXPECT_LT(gcs.EstimateComplexity(), 1100);

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  // Define solver options.
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 3;

  auto [traj, result] = gcs.SolvePath(source, target, options);

  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, EdgesFromWrappedJoints) {
  // With wrapping continous joints
  Vector3d point_1(0.5, -12 * M_PI + 0.1, 12 * M_PI - 0.1);
  Vector3d point_2(0.5, 12 * M_PI - 0.1, 0.0);
  HPolyhedron box = HPolyhedron::MakeUnitBox(3);
  const auto regions = MakeConvexSets(Point(point_1), Point(point_2), box);
  // Let's make 4 scenarios
  const std::vector<int> continuous_revolute_joints_1 = {};
  const std::vector<int> continuous_revolute_joints_2 = {1};
  const std::vector<int> continuous_revolute_joints_3 = {0, 2};
  const std::vector<int> continuous_revolute_joints_4 = {2, 1};
  // scenario 1: no wrapping, no edges
  auto gcs_1 = GcsTrajectoryOptimization(3, continuous_revolute_joints_1);
  gcs_1.AddRegions(regions, 1);
  EXPECT_EQ(gcs_1.graph_of_convex_sets().Edges().size(), 0);
  // scenario 2: wrapping on joint 1, point_2 <--> box
  auto gcs_2 = GcsTrajectoryOptimization(3, continuous_revolute_joints_2);
  gcs_2.AddRegions(regions, 1);
  EXPECT_EQ(gcs_2.graph_of_convex_sets().Edges().size(), 2);
  EXPECT_EQ(
      gcs_2.graph_of_convex_sets().Vertices().at(0)->outgoing_edges().size(),
      0);
  EXPECT_EQ(
      gcs_2.graph_of_convex_sets().Vertices().at(1)->outgoing_edges().size(),
      1);
  EXPECT_EQ(
      gcs_2.graph_of_convex_sets().Vertices().at(2)->outgoing_edges().size(),
      1);
  // scenario 3: wrapping on joint 0 and 2, no edges
  auto gcs_3 = GcsTrajectoryOptimization(3, continuous_revolute_joints_3);
  gcs_3.AddRegions(regions, 1);
  EXPECT_EQ(gcs_3.graph_of_convex_sets().Edges().size(), 0);
  // scenario 4: wrapping on joint 1 and 2, point_1 <--> box and point_2 <-->
  // box
  auto gcs_4 = GcsTrajectoryOptimization(3, continuous_revolute_joints_4);
  gcs_4.AddRegions(regions, 1);
  EXPECT_EQ(gcs_4.graph_of_convex_sets().Edges().size(), 4);
  EXPECT_EQ(
      gcs_4.graph_of_convex_sets().Vertices().at(0)->outgoing_edges().size(),
      1);
  EXPECT_EQ(
      gcs_4.graph_of_convex_sets().Vertices().at(1)->outgoing_edges().size(),
      1);
  EXPECT_EQ(
      gcs_4.graph_of_convex_sets().Vertices().at(2)->outgoing_edges().size(),
      2);
}

GTEST_TEST(GcsTrajectoryOptimizationTest, WraparoundInOneDimension) {
  /*
             0, 2π
    6 (rad) *********    ◄──┐
         G *         ***    │
       ***             ***  Short path
      **                 ** (with wraparound)
      **                 **
      *                   *
      **                 **
      **                 S  1 (rad)
       ***             ***
         ***         ***
     ▲      *********
     └── Long path (no wraparound)
  */
  const double tol = 1e-7;

  Eigen::Matrix<double, 1, 2> points1, points2, points3;
  points1 << 0, 2.4;
  VPolytope v1(points1);
  points2 << 2.39, 4.78;
  VPolytope v2(points2);
  points3 << 4.78, 7.17;
  VPolytope v3(points3);

  Vector1d start(1.0), goal(6.0);
  DRAKE_DEMAND(v1.PointInSet(start, tol));
  DRAKE_DEMAND(v3.PointInSet(goal, tol));

  const double expected_cost_wraparound = 2.0 * M_PI - 6.0 + 1.0;
  const double expected_cost_no_wraparound = 6.0 - 1.0;
  DRAKE_DEMAND(expected_cost_wraparound < expected_cost_no_wraparound);

  std::vector<int> continuous_revolute_joints = {0};

  // Check multiple permutations to make sure the ordering of which set is
  // "wrapped around" does not affect the algorithm. Sets v1 and v3 overlap with
  // a wraparound edge, so flipping their order is sufficient for checking this
  // behavior.
  std::vector<ConvexSets> permutations = {MakeConvexSets(v1, v2, v3),
                                          MakeConvexSets(v3, v2, v1)};
  for (const auto& convexsets : permutations) {
    GcsTrajectoryOptimization gcs1(1, continuous_revolute_joints);
    auto& regions1 = gcs1.AddRegions(convexsets, 1, 0.01, 1, "");

    auto& source1 = gcs1.AddRegions(MakeConvexSets(Point(start)), 0);
    auto& target1 = gcs1.AddRegions(MakeConvexSets(Point(goal)), 0);

    gcs1.AddEdges(source1, regions1);
    gcs1.AddEdges(regions1, target1);

    // 3 sets fully connected implies 3 bidirectional edges implies 6 edges
    // The start and goal are each connected to one set, so 8 total.
    EXPECT_EQ(gcs1.graph_of_convex_sets().Edges().size(), 8);

    regions1.AddPathLengthCost(1);

    auto [traj1, result1] = gcs1.SolvePath(source1, target1);
    ASSERT_TRUE(result1.is_success());
    EXPECT_NEAR(result1.get_optimal_cost(), expected_cost_wraparound, tol);
    EXPECT_EQ(traj1.get_number_of_segments(), 2);
    EXPECT_TRUE(CompareMatrices(traj1.value(traj1.start_time()), start, 1e-6));
    EXPECT_TRUE(CompareMatrices(traj1.value(traj1.end_time()), goal, 1e-6));

    // Now check that the wraparound is supported when adding edges between
    // subgraphs.
    Vector1d start_alternate(start[0] - 4.0 * M_PI);
    Vector1d goal_alternate(goal[0] + 6.0 * M_PI);
    auto& source1_alternate =
        gcs1.AddRegions(MakeConvexSets(Point(start_alternate)), 0);
    auto& target1_alternate =
        gcs1.AddRegions(MakeConvexSets(Point(goal_alternate)), 0);

    gcs1.AddEdges(source1_alternate, regions1);
    gcs1.AddEdges(regions1, target1_alternate);

    // source1_alternate and target1_alternate should each be connected to one
    // set in regions1, so there should now be 10 edges.
    EXPECT_EQ(gcs1.graph_of_convex_sets().Edges().size(), 10);

    auto [traj1_alternate, result1_alternate] =
        gcs1.SolvePath(source1, target1);
    ASSERT_TRUE(result1_alternate.is_success());
    EXPECT_NEAR(result1_alternate.get_optimal_cost(), expected_cost_wraparound,
                tol);
    EXPECT_EQ(traj1_alternate.get_number_of_segments(), 2);
  }

  // Now no wraparound
  GcsTrajectoryOptimization gcs2(1);
  auto& regions2 = gcs2.AddRegions(MakeConvexSets(v1, v2, v3), 1, 0.01, 1, "");

  auto& source2 = gcs2.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target2 = gcs2.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs2.AddEdges(source2, regions2);
  gcs2.AddEdges(regions2, target2);

  // 3 sets connected in a chain implies 2 bidirectional edges implies 4 edges
  // The start and goal are each connected to one set, so 6 total.
  EXPECT_EQ(gcs2.graph_of_convex_sets().Edges().size(), 6);

  regions2.AddPathLengthCost(1);

  auto [traj2, result2] = gcs2.SolvePath(source2, target2);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(result2.get_optimal_cost(), expected_cost_no_wraparound, tol);
  EXPECT_EQ(traj2.get_number_of_segments(), 3);
  EXPECT_TRUE(CompareMatrices(traj2.value(traj2.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj2.value(traj2.end_time()), goal, 1e-6));
}

GTEST_TEST(GcsTrajectoryOptimizationTest, WraparoundInTwoDimensions) {
  if (!GurobiOrMosekSolverAvailable()) {
    // These test cases are too large for free solvers such as CSDP.
    return;
  }
  const double tol = 1e-7;

  ConvexSets sets;
  Eigen::Matrix<double, 2, 4> points;
  // clang-format off
  points << 0, 0,   2.4, 2.4,
            0, 2.4, 0,   2.4;
  // clang-format on
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Eigen::MatrixXd new_points = points;
      new_points = new_points.colwise() + i * Eigen::Vector2d{2.39, 0};
      new_points = new_points.colwise() + j * Eigen::Vector2d{0, 2.39};
      sets.emplace_back(copyable_unique_ptr(VPolytope(new_points)));
    }
  }

  Vector2d start(1.0, 1.0), goal(6.0, 6.0);
  DRAKE_DEMAND(sets[0]->PointInSet(start, tol));
  DRAKE_DEMAND(sets[8]->PointInSet(goal, tol));

  const double dist_long_way = 5.0;
  const double dist_shortcut = 1.0 + (2.0 * M_PI - 6.0);
  const double expected_cost_both_wraparound = sqrt(2) * dist_shortcut;
  const double expected_cost_one_wraparound =
      sqrt((dist_long_way * dist_long_way) + (dist_shortcut * dist_shortcut));
  const double expected_cost_no_wraparound = sqrt(2) * dist_long_way;
  DRAKE_DEMAND(expected_cost_both_wraparound < expected_cost_one_wraparound);
  DRAKE_DEMAND(expected_cost_one_wraparound < expected_cost_no_wraparound);

  std::vector<int> continuous_revolute_joints;
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 100;

  struct TestConfig {
    std::string name;
    std::vector<int> continuous_joints;
    double expected_cost{};
    int edge_count{};
    Eigen::Vector2d start_offset_works;
    Eigen::Vector2d start_offset_fails;
  };

  const std::vector<TestConfig> configs{
      {.name = "No wraparound",
       .expected_cost = expected_cost_no_wraparound,
       // 9 sets connected in a grid implies 12 bidirectional edges implies 24
       // edges Edges crossing the corners gives us 8 more bidirectional edges
       // (16 edges) The start and goal are each connected to one set, so 42
       // total.
       .edge_count = 42,
       .start_offset_works = {0.0, 0.0},
       .start_offset_fails = {-2.0 * M_PI, 2.0 * M_PI}},
      {.name = "One wraparound",
       .continuous_joints = {0},
       .expected_cost = expected_cost_one_wraparound,
       // One wraparound adds 3 new edges wrapping around along the rows, and
       // also 4 new edges crossing the rows, so 14 new directional edges.
       .edge_count = 56,
       .start_offset_works = {-2.0 * M_PI, 0.0},
       .start_offset_fails = {-2.0 * M_PI, 4.0 * M_PI}},
      {.name = "Two wraparound",
       .continuous_joints = {0, 1},
       .expected_cost = expected_cost_both_wraparound,
       // Another wraparound adds another 3 new edges wrapping around along the
       // columns, and also 4 new edges crossing the columns, and finally 2 more
       // edges connecting the opposite corners (e.g. top-left to bottom-right),
       // so 18 new directional edges.
       .edge_count = 74,
       .start_offset_works = {-2.0 * M_PI, 4.0 * M_PI},
       .start_offset_fails = {0.0, 0.0}}};
  // For two wraparound, none fail, so we use a conditional
  // check in the test body to skip this case.
  for (const TestConfig& config : configs) {
    SCOPED_TRACE(config.name);
    GcsTrajectoryOptimization gcs(2, config.continuous_joints);
    auto& regions = gcs.AddRegions(sets, 1, 0.01, 1, "");

    auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
    auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

    gcs.AddEdges(source, regions);
    gcs.AddEdges(regions, target);

    EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), config.edge_count);

    regions.AddPathLengthCost(1);

    auto [traj, result] = gcs.SolvePath(source, target, options);
    EXPECT_TRUE(result.is_success());
    EXPECT_NEAR(result.get_optimal_cost(), config.expected_cost, tol);
    EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
    EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));

    auto& source2 = gcs.AddRegions(
        MakeConvexSets(Point(start + config.start_offset_works)), 0);
    gcs.AddEdges(source2, regions);

    auto [traj2, result2] = gcs.SolvePath(source2, target, options);
    EXPECT_TRUE(result2.is_success());
    EXPECT_NEAR(result2.get_optimal_cost(), config.expected_cost, tol);

    if (config.continuous_joints.size() != 2) {
      // When both dimensions are continuous revolute, any start offset will
      // work, so we want to skip it.
      auto& source3 = gcs.AddRegions(
          MakeConvexSets(Point(start + config.start_offset_fails)), 0);
      gcs.AddEdges(source3, regions);

      auto [traj3, result3] = gcs.SolvePath(source3, target, options);
      EXPECT_FALSE(result3.is_success());
    }
  }
}

GTEST_TEST(GcsTrajectoryOptimizationTest,
           WraparoundWithInvalidConvexityRadius) {
  // 1D example
  Eigen::Matrix<double, 1, 2> points;
  points << 0, 4;
  const VPolytope v(points);
  std::vector<int> continuous_revolute_joints = {0};
  GcsTrajectoryOptimization gcs(1, continuous_revolute_joints);
  DRAKE_EXPECT_THROWS_MESSAGE(gcs.AddRegions(MakeConvexSets(v), 1),
                              ".*doesn't respect the convexity radius.*");
  const std::vector<std::pair<int, int>> edges;
  DRAKE_EXPECT_THROWS_MESSAGE(gcs.AddRegions(MakeConvexSets(v), edges, 1),
                              ".*doesn't respect the convexity radius.*");

  // 2D example
  Eigen::Matrix<double, 2, 4> points_new;
  // clang-format off
  points_new << 0, 0, 2, 2,
                0, 4, 0, 4;
  // clang-format on
  const VPolytope w(points_new);
  std::vector<int> continuous_revolute_joints1 = {0};
  std::vector<int> continuous_revolute_joints2 = {1};
  std::vector<int> continuous_revolute_joints3 = {0, 1};
  GcsTrajectoryOptimization gcs0(2);
  GcsTrajectoryOptimization gcs1(2, continuous_revolute_joints1);
  GcsTrajectoryOptimization gcs2(2, continuous_revolute_joints2);
  GcsTrajectoryOptimization gcs3(2, continuous_revolute_joints3);
  DRAKE_EXPECT_NO_THROW(gcs0.AddRegions(MakeConvexSets(w), 1));
  DRAKE_EXPECT_NO_THROW(gcs1.AddRegions(MakeConvexSets(w), 1));
  DRAKE_EXPECT_THROWS_MESSAGE(gcs2.AddRegions(MakeConvexSets(w), 1),
                              ".*doesn't respect the convexity radius.*");
  DRAKE_EXPECT_THROWS_MESSAGE(gcs3.AddRegions(MakeConvexSets(w), 1),
                              ".*doesn't respect the convexity radius.*");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, WraparoundSubspace) {
  // 1D example.
  Eigen::Matrix<double, 1, 2> points1;
  Eigen::Matrix<double, 1, 2> points2;
  Eigen::Matrix<double, 1, 2> points3;
  points1 << 0, 3;
  points2 << 2.5, 5.5;
  points3 << 5, 8;
  const VPolytope v1(points1);
  const VPolytope v2(points2);
  const VPolytope v3(points3);
  std::vector<int> continuous_revolute_joints = {0};
  const Point subspace12(Vector1d{2.75 + (6 * M_PI)});
  const Point subspace23(Vector1d{5.25 - (4 * M_PI)});
  const Point subspace13(Vector1d{7.0 + (2 * M_PI)});

  GcsTrajectoryOptimization gcs(1, continuous_revolute_joints);
  auto& subgraph1 = gcs.AddRegions(MakeConvexSets(HPolyhedron(v1)), 1);
  auto& subgraph2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron(v2), HPolyhedron(v3)), 1);

  // Initially, there should be two edges, v2 -> v3 and v3 -> v2.
  int expected_num_edges = 2;

  // When we add edges from subgraph1 to subgraph2, we add edges v1 -> v2
  // and v1 -> v3.
  gcs.AddEdges(subgraph1, subgraph2);
  expected_num_edges += 2;
  EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), expected_num_edges);

  // When we add edges from subgraph1 to subgraph2 through subspace12, only
  // v1 -> v2 is added as an edge, since their intersection overlaps with
  // subspace12. the intersection of v1 and v3 does not overlap with subspace12,
  // so v1 -> v3 is not added.
  gcs.AddEdges(subgraph1, subgraph2, &subspace12);
  expected_num_edges += 1;
  EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), expected_num_edges);

  // When we add edges from subgraph1 to subgraph2 through subspace13, only
  // v1 -> v3 is added as an edge, since their intersection overlaps with
  // subspace13. the intersection of v1 and v2 does not overlap with subspace13,
  // so v1 -> v2 is not added.
  gcs.AddEdges(subgraph1, subgraph2, &subspace13);
  expected_num_edges += 1;
  EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), expected_num_edges);

  // When we add edges from subgraph1 to subgraph2 through subspace23, no edges
  // are added, since the intersection between v1 and v2 does not overlap with
  // subspace23, and the intersection between v1 and v3 does not overlap with
  // subspace23.
  gcs.AddEdges(subgraph1, subgraph2, &subspace23);
  EXPECT_EQ(gcs.graph_of_convex_sets().Edges().size(), expected_num_edges);
}

GTEST_TEST(GcsTrajectoryOptimizationTest, ContinuousJointsApi) {
  Eigen::Matrix<double, 1, 2> points;
  points << 0, 2;
  const VPolytope v(points);
  std::vector<int> continuous_revolute_joints = {0};
  GcsTrajectoryOptimization gcs(1, continuous_revolute_joints);
  EXPECT_EQ(gcs.continuous_revolute_joints().size(),
            continuous_revolute_joints.size());
  EXPECT_EQ(gcs.continuous_revolute_joints()[0], continuous_revolute_joints[0]);

  continuous_revolute_joints.emplace_back(0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GcsTrajectoryOptimization(1, continuous_revolute_joints),
      ".*continuous_revolute_joints must not contain duplicate entries.*");

  continuous_revolute_joints = {-1};
  DRAKE_EXPECT_THROWS_MESSAGE(
      GcsTrajectoryOptimization(1, continuous_revolute_joints),
      ".*Each joint index in continuous_revolute_joints must lie in the "
      "interval.*");

  continuous_revolute_joints = {2};
  DRAKE_EXPECT_THROWS_MESSAGE(
      GcsTrajectoryOptimization(1, continuous_revolute_joints),
      ".*Each joint index in continuous_revolute_joints must lie in the "
      "interval.*");
}

GTEST_TEST(GcsTrajectoryOptimizationTest, GetContinuousJoints) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& first_body =
      plant.AddRigidBody("first_body", SpatialInertia<double>());
  const RigidBody<double>& second_body =
      plant.AddRigidBody("second_body", SpatialInertia<double>());
  const RigidBody<double>& third_body =
      plant.AddRigidBody("third_body", SpatialInertia<double>());
  const RigidBody<double>& fourth_body =
      plant.AddRigidBody("fourth_body", SpatialInertia<double>());

  // Add a planar joint without limits
  plant.AddJoint<PlanarJoint>("first_joint", plant.world_body(), {}, first_body,
                              {}, Eigen::Matrix<double, 3, 1>::Zero());

  // Add a planar joint with limits
  std::unique_ptr<PlanarJoint<double>> second_joint_ptr(new PlanarJoint<double>(
      "second_joint", first_body.body_frame(), second_body.body_frame(),
      Eigen::Matrix<double, 3, 1>::Zero()));
  second_joint_ptr->set_position_limits(Eigen::Vector3d{-1.0, -1.0, -1.0},
                                        Eigen::Vector3d{1.0, 1.0, 1.0});
  plant.AddJoint<PlanarJoint>(std::move(second_joint_ptr));

  // Add a revolute joint without limits
  plant.AddJoint<RevoluteJoint>("third_joint", second_body, {}, third_body, {},
                                Eigen::Matrix<double, 3, 1>{1.0, 0.0, 0.0});

  // Add a revolute joint with limits
  plant.AddJoint<RevoluteJoint>("fourth_joint", third_body, {}, fourth_body, {},
                                Eigen::Matrix<double, 3, 1>{1.0, 0.0, 0.0},
                                -1.0, 1.0);

  plant.Finalize();

  const std::vector<int> continuous_joint_indices =
      trajectory_optimization::GetContinuousRevoluteJointIndices(plant);
  ASSERT_EQ(continuous_joint_indices.size(), 2);
  EXPECT_EQ(continuous_joint_indices[0], 2);
  EXPECT_EQ(continuous_joint_indices[1], 6);
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
