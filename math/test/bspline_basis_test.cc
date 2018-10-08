#include "drake/math/bspline_basis.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

using drake::VectorX;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace drake {
namespace math {
// Verifies that the constructors work as expected.
GTEST_TEST(BSplineBasisTests, ConstructorTest) {
  const int expected_order = 4;
  const int expected_num_control_points = 11;
  const std::vector<double> expected_knots{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  // Check the order and num_control_points constructor.
  BsplineBasis bspline_basis_0{expected_order, expected_num_control_points};
  EXPECT_EQ(bspline_basis_0.order(), expected_order);
  EXPECT_EQ(bspline_basis_0.num_control_points(), expected_num_control_points);
  EXPECT_EQ(bspline_basis_0.knots(), expected_knots);

  // Check the order and knots constructor.
  BsplineBasis bspline_basis_1{expected_order, expected_knots};
  EXPECT_EQ(bspline_basis_1.order(), expected_order);
  EXPECT_EQ(bspline_basis_1.num_control_points(), expected_num_control_points);
  EXPECT_EQ(bspline_basis_1.knots(), expected_knots);

  // Verify that the bases generate the same values at a range of times.
  const int num_times = 100;
  VectorX<double> t = VectorX<double>::LinSpaced(num_times, 0, 1);
  for (int i = 0; i < num_times; ++i) {
    for (int j = 0; j < expected_num_control_points; ++j) {
      EXPECT_EQ(bspline_basis_0.EvaluateBasisFunction(j, t(i)),
                bspline_basis_1.EvaluateBasisFunction(j, t(i)));
    }
  }
}

// Verifies that ComputeActiveControlPointIndices() returns the correct values
// for selected inputs.
GTEST_TEST(BSplineBasisTests, ComputeActiveControlPointIndicesTest) {
  const int expected_order = 5;
  const int expected_num_control_points = 14;
  BsplineBasis bspline_basis_0{expected_order, expected_num_control_points};

  const std::array<double, 2> plan_interval_0{{0.9, 1.0}};
  const std::vector<int> expected_active_control_point_indices_0{9, 10, 11, 12,
                                                                 13};
  std::vector<int> active_control_point_indices_0{
      bspline_basis_0.ComputeActiveControlPointIndices(plan_interval_0)};
  EXPECT_EQ(active_control_point_indices_0,
            expected_active_control_point_indices_0);

  const std::array<double, 2> plan_interval_1{{0.95, 1.0}};
  const std::vector<int> expected_active_control_point_indices_1{9, 10, 11, 12,
                                                                 13};
  std::vector<int> active_control_point_indices_1{
      bspline_basis_0.ComputeActiveControlPointIndices(plan_interval_1)};
  EXPECT_EQ(active_control_point_indices_1,
            expected_active_control_point_indices_1);

  const std::array<double, 2> plan_interval_2{{0.85, 1.0}};
  const std::vector<int> expected_active_control_point_indices_2{8,  9,  10,
                                                                 11, 12, 13};
  std::vector<int> active_control_point_indices_2{
      bspline_basis_0.ComputeActiveControlPointIndices(plan_interval_2)};
  EXPECT_EQ(active_control_point_indices_2,
            expected_active_control_point_indices_2);

  const std::array<double, 2> plan_interval_3{{0.0, 0.0}};
  const std::vector<int> expected_active_control_point_indices_3{0, 1, 2, 3, 4};
  std::vector<int> active_control_point_indices_3{
      bspline_basis_0.ComputeActiveControlPointIndices(plan_interval_3)};
  EXPECT_EQ(active_control_point_indices_3,
            expected_active_control_point_indices_3);

  BsplineBasis bspline_basis_1{1, expected_num_control_points};
  const std::array<double, 2> plan_interval_4{{0.0, 0.0}};
  const std::vector<int> expected_active_control_point_indices_4{0};
  std::vector<int> active_control_point_indices_4{
      bspline_basis_1.ComputeActiveControlPointIndices(plan_interval_4)};
  EXPECT_EQ(active_control_point_indices_4,
            expected_active_control_point_indices_4);
}
}  // namespace math
}  // namespace drake
