#include "drake/geometry/optimization/spheres_cover.h"

#include <gtest/gtest.h>

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
GTEST_TEST(SpheresOuterApproximatorTest, SingleBox1) {
  // Test covering a single box with spheres. The sphere can avoid all outliers.
  Eigen::Matrix<double, 2, 4> box;
  // clang-format off
  box << 0.5, 0.5, -0.5, -0.5,
         0.5, -0.5, -0.5, 0.5;
  // clang-format on
  // The outliers are just outside the corners of the box.
  Eigen::Matrix<double, 2, 4> outliers = 1.5 * box;

  // Test without setting max_sphere_radius.
  SpheresOuterApproximator dut1(1, {box}, outliers);

  auto result = solvers::Solve(dut1.prog());
  EXPECT_TRUE(result.is_success());
  // The optimal cost should be 0, that none of the outliers is covered by the
  // sphere.
  EXPECT_NEAR(result.get_optimal_cost(), 0, 1E-7);
  // Make sure the sphere contains all vertices of the box, but no outliers.
  Eigen::Vector2d a_val = result.GetSolution(dut1.spheres()[0].a);
  double b_val = result.GetSolution(dut1.spheres()[0].b);
  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(box.col(i).dot(box.col(i)) + box.col(i).dot(a_val) + b_val, 0);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_GE(outliers.col(i).dot(outliers.col(i)) +
                  outliers.col(i).dot(a_val) + b_val,
              0);
  }
  // Now add the sphere radius constraint. Check the sphere radius constraint
  double max_sphere_radius = 0.8;
  SpheresOuterApproximator dut2(1, {box}, outliers, max_sphere_radius);
  result = solvers::Solve(dut2.prog());
  EXPECT_TRUE(result.is_success());
  a_val = result.GetSolution(dut2.spheres()[0].a);
  b_val = result.GetSolution(dut2.spheres()[0].b);
  EXPECT_LE((a_val / 2).dot(a_val / 2) - b_val,
            max_sphere_radius * max_sphere_radius);

  // Now further tighten the max sphere radius. The problem should be
  // infeasible.
  max_sphere_radius = 0.7;
  SpheresOuterApproximator dut3(1, {box}, outliers, max_sphere_radius);
  result = solvers::Solve(dut3.prog());
  EXPECT_FALSE(result.is_success());
}

GTEST_TEST(SpheresOuterApproximatorTest, SingleBox2) {
  // Test covering a single box with spheres. The sphere has to contain some
  // outliers.
  Eigen::Matrix<double, 2, 4> box;
  // clang-format off
  box << 0.5, 0.5, -0.5, -0.5,
         0.5, -0.5, -0.5, 0.5;
  // clang-format on
  Eigen::Matrix<double, 2, 3> outliers;
  // clang-format off
  outliers << 0,  1, 0,
              0.6, 1, -0.6;
  // clang-format on

  SpheresOuterApproximator dut1(1, {box}, outliers);

  auto result = solvers::Solve(dut1.prog());
  EXPECT_TRUE(result.is_success());
  // The optimal cost should be 2, that two outliers are covered by the
  // sphere.
  EXPECT_EQ(result.get_optimal_cost(), 2);
  // Make sure the sphere contains all vertices of the box
  Eigen::Vector2d a_val = result.GetSolution(dut1.spheres()[0].a);
  double b_val = result.GetSolution(dut1.spheres()[0].b);
  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(box.col(i).dot(box.col(i)) + box.col(i).dot(a_val) + b_val, 0);
  }
  const auto zeta_sol = result.GetSolution(dut1.zeta());
  for (int i = 0; i < outliers.cols(); ++i) {
    if (std::abs(zeta_sol(i)) < 1E-6) {
      // zeta_sol(i) = 0, the outlier is not inside the sphere.
      EXPECT_GE(outliers.col(i).dot(outliers.col(i)) +
                    outliers.col(i).dot(a_val) + b_val,
                0);
    } else {
      EXPECT_LE(outliers.col(i).dot(outliers.col(i)) +
                    outliers.col(i).dot(a_val) + b_val,
                0);
    }
    EXPECT_GE(outliers.col(1).dot(outliers.col(1)) +
                  outliers.col(1).dot(a_val) + b_val,
              0);
  }
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
