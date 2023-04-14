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

GTEST_TEST(GCSTrajectoryOptimizationTest, Basic) {
  const int kDimension = 2;
  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add a single region (the unit box), and plan a line segment inside that
  // box.
  Vector2d start(-0.5, -0.5), goal(0.5, 0.5);

  auto& regions =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 1);
  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs.AddEdges(&source, &regions);
  gcs.AddEdges(&regions, &target);

  auto [traj, result] = gcs.SolvePath(source, target);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_EQ(traj.cols(), 1);
  EXPECT_TRUE(CompareMatrices(traj.value(traj.start_time()), start, 1e-6));
  EXPECT_TRUE(CompareMatrices(traj.value(traj.end_time()), goal, 1e-6));
}

GTEST_TEST(GCSTrajectoryOptimizationTest, InvalidPositions) {
  /** Positions passed into GCSTrajectoryOptimization must be greater than 0.*/
  DRAKE_EXPECT_THROWS_MESSAGE(GCSTrajectoryOptimization(0),
                              "Dimension must be greater than 0.");
  DRAKE_EXPECT_THROWS_MESSAGE(GCSTrajectoryOptimization(-1),
                              "Dimension must be greater than 0.");
  DRAKE_EXPECT_NO_THROW(GCSTrajectoryOptimization(1));
}

GTEST_TEST(GCSTrajectoryOptimizationTest, InvalidOrder) {
  /** Subgraph order must be positive.*/
  const int kDimension = 2;
  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), -1),
      "Order must be positive.");

  DRAKE_EXPECT_NO_THROW(
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0));
}

GTEST_TEST(GCSTrajectoryOptimizationTest, ZeroOrderPathLengthCost) {
  /** Path length cost is not defined for a set of order 0.*/
  const int kDimension = 2;
  GCSTrajectoryOptimization gcs(kDimension);
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

GTEST_TEST(GCSTrajectoryOptimizationTest, DisjointGraph) {
  /** Adds four points to the graph without any edges. */
  const int kDimension = 2;
  GCSTrajectoryOptimization gcs(kDimension);
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
  DRAKE_EXPECT_THROWS_MESSAGE(traj.rows(), ".*no segments.*");
  DRAKE_EXPECT_THROWS_MESSAGE(traj.cols(), ".*no segments.*");
}

GTEST_TEST(GCSTrajectoryOptimizationTest, InvalidSubspace) {
  const int kDimension = 2;
  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  auto& regions1 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  auto& regions2 =
      gcs.AddRegions(MakeConvexSets(HPolyhedron::MakeUnitBox(kDimension)), 0);

  auto subspace_wrong_dimension = HPolyhedron::MakeUnitBox(kDimension + 1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gcs.AddEdges(&regions1, &regions2, &subspace_wrong_dimension),
      "Subspace dimension must match the number of positions.");

  Eigen::Matrix<double, 2, 4> vertices;
  vertices << 5.0, 5.0, 4.4, 4.4, 2.8, 5.0, 5.0, 2.8;
  VPolytope vertices_subspace{vertices};

  DRAKE_EXPECT_THROWS_MESSAGE(gcs.AddEdges(&regions1, &regions2, &vertices_subspace),
                              "Subspace must be a Point or HPolyhedron.");
}

/** This 2D environment has been presented in the GCS paper.
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
    vertices_1 << 0.4, 0.4, 0.0, 0.0,
                  0.0, 5.0, 5.0, 0.0;
    HPolyhedron region_1{VPolytope(vertices_1)};


    Eigen::Matrix<double, 2, 4> vertices_2;
    vertices_2 << 0.4, 1.0, 1.0, 0.4,
                  2.4, 2.4, 2.6, 2.6;
    HPolyhedron region_2{VPolytope(vertices_2)};

    Eigen::Matrix<double, 2, 4> vertices_3;
    vertices_3 << 1.4, 1.4, 1.0, 1.0,
                  2.2, 4.6, 4.6, 2.2;
    HPolyhedron region_3{VPolytope(vertices_3)};

    Eigen::Matrix<double, 2, 4> vertices_4;
    vertices_4 << 1.4, 2.4, 2.4, 1.4,
                  2.2, 2.6, 2.8, 2.8;
    HPolyhedron region_4{VPolytope(vertices_4)};

    Eigen::Matrix<double, 2, 4> vertices_5;
    vertices_5 << 2.2, 2.4, 2.4, 2.2,
                  2.8, 2.8, 4.6, 4.6;
    HPolyhedron region_5{VPolytope(vertices_5)};

    Eigen::Matrix<double, 2, 5> vertices_6;
    vertices_6 << 1.4, 1., 1., 3.8, 3.8,
                  2.2, 2.2, 0.0, 0.0, 0.2;
    HPolyhedron region_6{VPolytope(vertices_6)};

    Eigen::Matrix<double, 2, 4> vertices_7;
    vertices_7 << 3.8, 3.8, 1.0, 1.0,
                  4.6, 5.0, 5.0, 4.6;
    HPolyhedron region_7{VPolytope(vertices_7)};

    Eigen::Matrix<double, 2, 5> vertices_8;
    vertices_8 << 5.0, 5.0, 4.8, 3.8, 3.8,
                  0.0, 1.2, 1.2, 0.2, 0.0;
    HPolyhedron region_8{VPolytope(vertices_8)};

    Eigen::Matrix<double, 2, 4> vertices_9;
    vertices_9 << 3.4, 4.8, 5.0, 5.0,
                  2.6, 1.2, 1.2, 2.6;
    HPolyhedron region_9{VPolytope(vertices_9)};

    Eigen::Matrix<double, 2, 4> vertices_10;
    vertices_10 << 3.4, 3.8, 3.8, 3.4,
                  2.6, 2.6, 4.6, 4.6;
    HPolyhedron region_10{VPolytope(vertices_10)};

    Eigen::Matrix<double, 2, 4> vertices_11;
    vertices_11 << 3.8, 4.4, 4.4, 3.8,
                  2.8, 2.8, 3.0, 3.0;
    HPolyhedron region_11{VPolytope(vertices_11)};

    Eigen::Matrix<double, 2, 4> vertices_12;
    vertices_12 << 5.0, 5.0, 4.4, 4.4,
                  2.8, 5.0, 5.0, 2.8;
    HPolyhedron region_12{VPolytope(vertices_12)};

    regions_ = MakeConvexSets(region_1, region_2, region_3, region_4,
                                    region_5, region_6, region_7, region_8,
                                    region_9, region_10, region_11, region_12);

  }
  ConvexSets regions_;
};


TEST_F(SimpleEnv2D, BasicShortestPath) {
  const int kDimension = 2;
  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 1);
  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  gcs.AddEdges(&source, &regions);
  gcs.AddEdges(&regions, &target);

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
  /** The durations bounds in a subgraph can be used to enforce delays.*/
  const int kDimension = 2;
  const double kStartDelay = 10;
  const double kGoalDelay = 20;

  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  // Add path length cost to whole graph before adding regions.
  // New added regions will inherit this cost.
  gcs.AddPathLengthCost();

  Vector2d start(0.2, 0.2), goal(4.8, 4.8);
  auto& regions = gcs.AddRegions(regions_, 1);
  // Setting d_min = d_max = kStartDelay seconds will force to stay a the start
  // of the trajectory for kStartDelay seconds.
  auto& source =
      gcs.AddRegions(MakeConvexSets(Point(start)), 0, kStartDelay, kStartDelay);
  // Similarly for the goal, but with a delay of kGoalDelay seconds.
  auto& target =
      gcs.AddRegions(MakeConvexSets(Point(goal)), 0, kGoalDelay, kGoalDelay);

  gcs.AddEdges(&source, &regions);
  gcs.AddEdges(&regions, &target);


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
  /** Planning between subgraphs also enables to select the shortest path
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

  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start1(0.2, 0.2), start2(0.3, 3.2);
  Vector2d goal1(4.8, 4.8), goal2(4.9, 2.4);

  auto& regions = gcs.AddRegions(regions_, 3);
  // Setting d_min = d_max = kStartDelay seconds will force to stay a the start
  // of the trajectory for kStartDelay seconds.
  auto& source =
      gcs.AddRegions(MakeConvexSets(Point(start1), Point(start2)), 0);
  // Similarly for the goal, but with a delay of kGoalDelay seconds.
  auto& target =
      gcs.AddRegions(MakeConvexSets(Point(goal1), Point(goal2)), 0);

  gcs.AddEdges(&source, &regions);
  gcs.AddEdges(&regions, &target);

  gcs.AddPathLengthCost();

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
  /** Connecting two subgraphs with a subspace enables start to
  goal planning with a desired intermediate point, I,  somewhere along the path
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

  GCSTrajectoryOptimization gcs(kDimension);
  EXPECT_EQ(gcs.num_positions(), kDimension);

  Vector2d start(0.2, 0.2), goal(4.8, 4.8), intermediate(2.3, 3.5);

  // Both a Point and an HPolytope are valid subspaces.
  Point subspace_point(intermediate);

  Eigen::Matrix<double, 2, 5> vertices;
  vertices << 5.0, 5.0, 4.8, 3.8, 3.8,
                0.0, 1.2, 1.2, 0.2, 0.0;
  HPolyhedron subspace_region{VPolytope(vertices)};


  // We can have different order subgraphs.
  // The trajectory from start to either subspace or intermidiate point will be of order 3
  // and the trajectory from intermediate to goal will be of order 2.
  auto& main1 = gcs.AddRegions(regions_, 3, 1e-6, 20, "main1");
  auto& main2 = gcs.AddRegions(regions_, 2, 1e-6, 20, "main2");

  auto& source = gcs.AddRegions(MakeConvexSets(Point(start)), 0);
  auto& target = gcs.AddRegions(MakeConvexSets(Point(goal)), 0);

  // The following wiring will give GCS the choice to either go 
  // through subspace point or the subspace region.
  gcs.AddEdges(&source, &main1);
  // Connect the two subgraphs through the intermediate point.
  gcs.AddEdges(&main1, &main2, &subspace_point);
  // Connect the two subgraphs through the subspace region.
  gcs.AddEdges(&main1, &main2, &subspace_region);
  gcs.AddEdges(&main2, &target);

  // We can add different costs to the individual subgraphs.
  main1.AddPathLengthCost(5);

  // This weight matrix penalizes movement in the y direction three times more than
  // in the x direction.
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(gcs.num_positions(), gcs.num_positions());
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
