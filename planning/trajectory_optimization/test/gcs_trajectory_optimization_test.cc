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

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::MakeConvexSets;
using geometry::optimization::Point;
using geometry::optimization::VPolytope;

bool GurobiOrMosekSolverAvailable() {
  return (solvers::MosekSolver::is_available() &&
          solvers::MosekSolver::is_enabled()) ||
         (solvers::GurobiSolver::is_available() &&
          solvers::GurobiSolver::is_enabled());
}

GTEST_TEST(GcsTrajectoryOptimizationTest, Basic) {
  const int kDimension = 2;
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add a single region (the unit box), and plan a line segment inside that
  // box.
  Vector2d start(-0.5, -0.5), goal(0.5, 0.5);

  auto& regions =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1);
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
                     3);
  // Bob can walk at 1 m/s in x and y.
  walking_regions.AddVelocityBounds(Vector2d(-kWalkingSpeed, -kWalkingSpeed),
                                    Vector2d(kWalkingSpeed, kWalkingSpeed));

  auto& scooter_regions = gcs.AddRegions(
      MakeConvexSets(
          HPolyhedron::MakeBox(Vector2d(-0.5, -0.5), Vector2d(-0.2, 1)),
          HPolyhedron::MakeBox(Vector2d(-0.5, 1), Vector2d(0.5, 1.5)),
          HPolyhedron::MakeBox(Vector2d(0.2, -0.5), Vector2d(0.5, 1))),
      3);
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

  if (!GurobiOrMosekSolverAvailable()) {
    return;
  }

  auto [shortest_path_traj, shortest_path_result] =
      gcs.SolvePath(source, target);
  EXPECT_TRUE(shortest_path_result.is_success());
  EXPECT_EQ(shortest_path_traj.rows(), 2);
  EXPECT_EQ(shortest_path_traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(
      shortest_path_traj.value(shortest_path_traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(
      shortest_path_traj.value(shortest_path_traj.end_time()), goal, 1e-6));

  // We expect the shortest path to be a straight line from S to G, going
  // through the gravel. Thus we expect the number of segments in the trajectory
  // to be 3 (one for start, one for the walking_regions and one for the goal)
  // and the path length to be 1.
  EXPECT_EQ(shortest_path_traj.get_number_of_segments(), 3);

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
  // gravel. Thus we expect the number of segments in the trajectory to be 5
  // (one for start, three for the scooter_regions and one for the goal) and the
  // path length to be greater than 2.4 (the minimum path length for the
  // detour).
  EXPECT_EQ(fastest_path_traj.get_number_of_segments(), 5);

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
    // TODO(wrangelvid) as we add the start and end points as curves with
    // order zero to the composite trajectory, the velocity is not computed
    // properly.
    if (fastest_path_traj.start_time() < t &&
        t < fastest_path_traj.end_time()) {
      EXPECT_TRUE((fastest_path_velocity->value(t).cwiseAbs() -
                   Vector2d(kScooterSpeed, kScooterSpeed))
                      .cwiseAbs()
                      .minCoeff() < 1e-6);
    }
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

  auto& regions =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      regions.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero()),
      "Velocity Bounds are not defined for a set of order 0.");
  // This should consider the order of the subgraphs and not add the velocity
  // bounds to the regions.
  DRAKE_EXPECT_NO_THROW(
      gcs.AddVelocityBounds(Vector2d::Zero(), Vector2d::Zero()));

  // lower and upper bound must have the same dimension as the graph.
  DRAKE_EXPECT_THROWS_MESSAGE(
      regions.AddVelocityBounds(Vector3d::Zero(), Vector2d::Zero()),
      ".*size.*.num_positions.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddVelocityBounds(Vector2d::Zero(), Vector3d::Zero()),
      ".*size.*.num_positions.*");
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
  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 1);
  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

  gcs.AddPathLengthCost();

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
  const double kStartDelay = 10;
  const double kGoalDelay = 20;

  GcsTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add path length cost to whole graph before adding regions.
  // New added regions will inherit this cost.
  gcs.AddPathLengthCost();

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 1);
  // Setting h_min = h_max = kStartDelay seconds will force to stay a the start
  // of the trajectory for kStartDelay seconds.
  auto& source =
      gcs.AddRegions(MakeConvexSets(Point(start)), 0, kStartDelay, kStartDelay);
  // Similarly for the goal, but with a delay of kGoalDelay seconds.
  auto& target =
      gcs.AddRegions(MakeConvexSets(Point(goal)), 0, kGoalDelay, kGoalDelay);

  gcs.AddEdges(source, regions);
  gcs.AddEdges(regions, target);

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
  gcs.AddEdges(source, main1);
  // Connect the two subgraphs through the intermediate point.
  gcs.AddEdges(main1, main2, &subspace_point);
  // Connect the two subgraphs through the subspace region.
  gcs.AddEdges(main1, main2, &subspace_region);
  gcs.AddEdges(main2, target);

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
