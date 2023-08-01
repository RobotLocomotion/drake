#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"
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
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::MakeConvexSets;
using geometry::optimization::Point;
using geometry::optimization::VPolytope;
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

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

  auto [traj, result] = gcs.SolvePath(source, target);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_EQ(traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));
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
  EXPECT_TRUE(fastest_path_length > kLeastExpectedFastestPathLength);
  // We expect the fastest path to be longer than the shortest path.
  EXPECT_TRUE(fastest_path_length > shortest_path_length);

  // The fastest path should be faster than the shortest path.
  const double fastest_path_duration =
      fastest_path_traj.end_time() - fastest_path_traj.start_time();
  EXPECT_TRUE(fastest_path_duration < shortest_path_duration);
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
  EXPECT_TRUE(traj.end_time() - traj.start_time() >=
              kDuckDelay + 305 / kMaxSpeed);

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
  // TODO(wrangelvid) Add a better way to check if composite trajectory is
  // empty.
  DRAKE_EXPECT_THROWS_MESSAGE(traj.rows(), ".*no segments.*");
  DRAKE_EXPECT_THROWS_MESSAGE(traj.cols(), ".*no segments.*");
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
  EXPECT_TRUE(traj.end_time() - traj.start_time() >= kStartDelay + kGoalDelay);
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

}  // namespace
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
