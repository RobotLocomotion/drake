#include "drake/geometry/optimization/bspline_graph_of_convex_sets.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Vector2d;
using trajectories::BsplineTrajectory;

namespace {
/*
     3        ┏━━━━━━━━━━━━━● target
              ┃             ┃
              ┃             ┃
     2 ┏━━━━━━┃━━━━━━┓      ┃
       ┃      ┃      ┃      ┃
       ┃      ┃      ┃      ┃
     1 ┃      ┗━━━━━━━━━━━━━┛
       ┃             ┃
       ┃             ┃
source ●━━━━━━━━━━━━━┛
       0      1      2      3
*/
BsplineTrajectoryThroughUnionOfHPolyhedra MakeSimpleProblem() {
  Vector2d source{0, 0};
  Vector2d target{3, 3};
  std::vector<HPolyhedron> regions{};
  regions.push_back(HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(2, 2)));
  regions.push_back(HPolyhedron::MakeBox(Vector2d(1, 1), Vector2d(3, 3)));

  return{source, target, regions};
}
}  // namespace

GTEST_TEST(BsplineTrajectoryThroughUnionOfHPolyhedraTests, Constructor) {
  BsplineTrajectoryThroughUnionOfHPolyhedra problem = MakeSimpleProblem();
  EXPECT_EQ(problem.ambient_dimension(), 2);
  EXPECT_EQ(problem.num_regions(), 4);
  EXPECT_EQ(problem.max_repetitions(), 1);
  EXPECT_EQ(problem.order(), 6);
  EXPECT_EQ(problem.source(), Vector2d(0.0, 0.0));
  EXPECT_EQ(problem.target(), Vector2d(3.0, 3.0));
  EXPECT_EQ(problem.max_velocity().size(), problem.ambient_dimension());
  EXPECT_TRUE(problem.max_velocity().array().isInf().all());
}

GTEST_TEST(BsplineTrajectoryThroughUnionOfHPolyhedraTests, SetMaxRepetitions) {
  BsplineTrajectoryThroughUnionOfHPolyhedra problem = MakeSimpleProblem();
  problem.set_max_repetitions(3);
  EXPECT_EQ(problem.max_repetitions(), 3);
}

GTEST_TEST(BsplineTrajectoryThroughUnionOfHPolyhedraTests,
           SetExtraControlPointsPerRegion) {
  BsplineTrajectoryThroughUnionOfHPolyhedra problem = MakeSimpleProblem();
  problem.set_extra_control_points_per_region(3);
  EXPECT_EQ(problem.extra_control_points_per_region(), 3);
}

GTEST_TEST(BsplineTrajectoryThroughUnionOfHPolyhedraTests, SetOrder) {
  BsplineTrajectoryThroughUnionOfHPolyhedra problem = MakeSimpleProblem();
  problem.set_order(3);
  EXPECT_EQ(problem.order(), 3);
}

GTEST_TEST(BsplineTrajectoryThroughUnionOfHPolyhedraTests, Solve) {
  // Check that solving the mixed-integer program directly works.
  BsplineTrajectoryThroughUnionOfHPolyhedra problem = MakeSimpleProblem();
  problem.set_max_velocity(Eigen::Vector2d(1, 1));
  std::optional<BsplineTrajectory<double>> solution_trajectory =
      problem.Solve();
  EXPECT_TRUE(solution_trajectory.has_value());
  EXPECT_EQ(solution_trajectory->InitialValue(), problem.source());
  EXPECT_EQ(solution_trajectory->FinalValue(), problem.target());
  // Check that solving with rounding works as well.
  std::optional<BsplineTrajectory<double>> rounding_solution_trajectory =
      problem.Solve(true);
  EXPECT_TRUE(rounding_solution_trajectory.has_value());
  EXPECT_EQ(rounding_solution_trajectory->InitialValue(), problem.source());
  EXPECT_EQ(rounding_solution_trajectory->FinalValue(), problem.target());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
