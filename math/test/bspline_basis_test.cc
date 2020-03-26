#include "drake/math/bspline_basis.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace math {

// Verifies that the constructors work as expected.
GTEST_TEST(BsplineBasisTests, ConstructorTest) {
  const int expected_order = 4;
  const int expected_num_basis_functions = 11;
  const std::vector<double> expected_knots_0{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  const std::vector<double> expected_knots_1{-3, -2, -1, 0, 1, 2,  3, 4,
                                             5,  6,  7,  8, 9, 10, 11};
  // Check the order and num_basis_functions constructor.
  BsplineBasis<double> bspline_basis_0{expected_order,
                                       expected_num_basis_functions};
  EXPECT_EQ(bspline_basis_0.order(), expected_order);
  EXPECT_EQ(bspline_basis_0.num_basis_functions(), expected_num_basis_functions);
  EXPECT_EQ(bspline_basis_0.knots(), expected_knots_0);
  BsplineBasis<double> bspline_basis_1{
      expected_order, expected_num_basis_functions, KnotVectorType::kUniform};
  EXPECT_EQ(bspline_basis_1.order(), expected_order);
  EXPECT_EQ(bspline_basis_1.num_basis_functions(), expected_num_basis_functions);
  EXPECT_EQ(bspline_basis_1.knots(), expected_knots_1);

  // Check the order and knots constructor.
  BsplineBasis<double> bspline_basis_2{expected_order, expected_knots_0};
  EXPECT_EQ(bspline_basis_2.order(), expected_order);
  EXPECT_EQ(bspline_basis_2.num_basis_functions(), expected_num_basis_functions);
  EXPECT_EQ(bspline_basis_2.knots(), expected_knots_0);
}

GTEST_TEST(BsplineBasisTests, MinNumControlPoints) {
  const int order = 4;
  const char* expected_message_0 =
      "The number of control points (.*) should be greater than or equal to "
      "the order (.*).";
  DRAKE_EXPECT_THROWS_MESSAGE(BsplineBasis<double>(order, 3),
                              std::invalid_argument, expected_message_0);
  const char* expected_message_1 =
      "The number of knots (.*) should be greater than or equal to twice the "
      "order (.*).";
  DRAKE_EXPECT_THROWS_MESSAGE(
      BsplineBasis<double>(order, {0, 1, 2, 3, 4, 5, 6}), std::invalid_argument,
      expected_message_1);
}

// Verifies that ComputeActiveBasisFunctionIndices() returns the correct values
// for selected inputs.
GTEST_TEST(BsplineBasisTests, ComputeActiveBasisFunctionIndicesTest) {
  const int expected_order = 5;
  const int expected_num_basis_functions = 14;
  BsplineBasis<double> bspline_basis_0{expected_order,
                                       expected_num_basis_functions};

  const std::array<double, 2> plan_interval_0{0.9, 1.0};
  const std::vector<int> expected_active_control_point_indices_0{9, 10, 11, 12,
                                                                 13};
  std::vector<int> active_control_point_indices_0{
      bspline_basis_0.ComputeActiveBasisFunctionIndices(plan_interval_0)};
  EXPECT_EQ(active_control_point_indices_0,
            expected_active_control_point_indices_0);

  const std::array<double, 2> plan_interval_1{0.95, 1.0};
  const std::vector<int> expected_active_control_point_indices_1{9, 10, 11, 12,
                                                                 13};
  std::vector<int> active_control_point_indices_1{
      bspline_basis_0.ComputeActiveBasisFunctionIndices(plan_interval_1)};
  EXPECT_EQ(active_control_point_indices_1,
            expected_active_control_point_indices_1);

  const std::array<double, 2> plan_interval_2{0.85, 1.0};
  const std::vector<int> expected_active_control_point_indices_2{8,  9,  10,
                                                                 11, 12, 13};
  std::vector<int> active_control_point_indices_2{
      bspline_basis_0.ComputeActiveBasisFunctionIndices(plan_interval_2)};
  EXPECT_EQ(active_control_point_indices_2,
            expected_active_control_point_indices_2);

  const std::array<double, 2> plan_interval_3{0.0, 0.0};
  const std::vector<int> expected_active_control_point_indices_3{0, 1, 2, 3, 4};
  std::vector<int> active_control_point_indices_3{
      bspline_basis_0.ComputeActiveBasisFunctionIndices(plan_interval_3)};
  EXPECT_EQ(active_control_point_indices_3,
            expected_active_control_point_indices_3);

  const std::array<double, 2> plan_interval_4{1.0, 1.0};
  const std::vector<int> expected_active_control_point_indices_4{9, 10, 11, 12,
                                                                 13};
  std::vector<int> active_control_point_indices_4{
      bspline_basis_0.ComputeActiveBasisFunctionIndices(plan_interval_4)};
  EXPECT_EQ(active_control_point_indices_4,
            expected_active_control_point_indices_4);

  BsplineBasis<double> bspline_basis_1{1, expected_num_basis_functions};
  const std::array<double, 2> plan_interval_5{0.0, 0.0};
  const std::vector<int> expected_active_control_point_indices_5{0};
  std::vector<int> active_control_point_indices_5{
      bspline_basis_1.ComputeActiveBasisFunctionIndices(plan_interval_5)};
  EXPECT_EQ(active_control_point_indices_5,
            expected_active_control_point_indices_5);
}

// Verify that FindContainingInterval() works as expected.
GTEST_TEST(BsplineBasisTests, FindContainingIntervalTest) {
  const int order = 3;
  // Use 6 knots, t[0], ..., t[m], where m = 5.
  BsplineBasis<double> bspline_basis_0{order, {1, 2, 3, 4, 5, 6}};
  EXPECT_EQ(bspline_basis_0.FindContainingInterval(1), 0);
  EXPECT_EQ(bspline_basis_0.FindContainingInterval(1.5), 0);
  EXPECT_EQ(bspline_basis_0.FindContainingInterval(2), 1);
  EXPECT_EQ(bspline_basis_0.FindContainingInterval(2.5), 1);
  EXPECT_EQ(bspline_basis_0.FindContainingInterval(5.5), 4);
  // Check the special case where t = t[m]
  EXPECT_EQ(bspline_basis_0.FindContainingInterval(6), 4 /* m - 1 */);
}

// Compares values returned by BsplineBasis::BasisFunctionValue() to those
// generated by scipy.interpolate.Bspline.basis_element():
//
//     import numpy as np
//     from scipy.interpolate import BSpline
//     knots = [0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1]
//     order = 4
//     num_basis_functions = len(knots) - order
//     for i in range(0, num_basis_functions):
//         bspline = BSpline.basis_element(knots[i:(i+order+1)],
//                                         extrapolate=False)
//         u = np.union1d(np.linspace(bspline.t[order - 1],
//                                    bspline.t[-order], 7), knots)
//         print("parameter_values.push_back(std::vector<double>{{{}}});"
//               .format(", ".join("{:.17f}".format(n) for n in u)))
//         print("expected_basis_function_values.push_back" +
//               "(std::vector<double>{{{}}});"
//               .format(", ".join("{:.17f}".format(0.0 if np.isnan(n) else n)
//                                 for n in bspline(u))))
//
// NOTE(avalenzu): Scipy gives B[6](1.0) = 0.0, which is wrong. That value has
// been replaced with the correct one (1.0).
GTEST_TEST(BsplineBasisTests, BasisFunctionValueScipyComparison) {
  const int order = 4;
  const int num_basis_functions = 7;
  BsplineBasis<double> basis{order, num_basis_functions};
  std::vector<std::vector<double>> parameter_values{};
  std::vector<std::vector<double>> expected_basis_function_values{};
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.04166666666666666, 0.08333333333333333,
      0.12500000000000000, 0.16666666666666666, 0.20833333333333331,
      0.25000000000000000, 0.50000000000000000, 0.75000000000000000,
      1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      1.00000000000000000, 0.57870370370370383, 0.29629629629629639,
      0.12500000000000000, 0.03703703703703705, 0.00462962962962964,
      0.00000000000000000, 0.00000000000000000, 0.00000000000000000,
      0.00000000000000000});
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.08333333333333333, 0.16666666666666666,
      0.25000000000000000, 0.33333333333333331, 0.41666666666666663,
      0.50000000000000000, 0.75000000000000000, 1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      0.00000000000000000, 0.56481481481481488, 0.51851851851851860,
      0.25000000000000000, 0.07407407407407410, 0.00925925925925927,
      0.00000000000000000, 0.00000000000000000, 0.00000000000000000});
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.12500000000000000, 0.25000000000000000,
      0.37500000000000000, 0.50000000000000000, 0.62500000000000000,
      0.75000000000000000, 1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      0.00000000000000000, 0.26041666666666663, 0.58333333333333326,
      0.46875000000000000, 0.16666666666666666, 0.02083333333333333,
      0.00000000000000000, 0.00000000000000000});
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.16666666666666666, 0.25000000000000000,
      0.33333333333333331, 0.50000000000000000, 0.66666666666666663,
      0.75000000000000000, 0.83333333333333326, 1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      0.00000000000000000, 0.04938271604938271, 0.16666666666666666,
      0.37037037037037035, 0.66666666666666663, 0.37037037037037052,
      0.16666666666666666, 0.04938271604938278, 0.00000000000000000});
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.25000000000000000, 0.37500000000000000,
      0.50000000000000000, 0.62500000000000000, 0.75000000000000000,
      0.87500000000000000, 1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      0.00000000000000000, 0.00000000000000000, 0.02083333333333333,
      0.16666666666666666, 0.46875000000000000, 0.58333333333333326,
      0.26041666666666663, 0.00000000000000000});
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.25000000000000000, 0.50000000000000000,
      0.58333333333333337, 0.66666666666666663, 0.75000000000000000,
      0.83333333333333326, 0.91666666666666663, 1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      0.00000000000000000, 0.00000000000000000, 0.00000000000000000,
      0.00925925925925927, 0.07407407407407403, 0.25000000000000000,
      0.51851851851851827, 0.56481481481481488, 0.00000000000000000});
  parameter_values.push_back(std::vector<double>{
      0.00000000000000000, 0.25000000000000000, 0.50000000000000000,
      0.75000000000000000, 0.79166666666666663, 0.83333333333333337,
      0.87500000000000000, 0.91666666666666663, 0.95833333333333326,
      1.00000000000000000});
  expected_basis_function_values.push_back(std::vector<double>{
      0.00000000000000000, 0.00000000000000000, 0.00000000000000000,
      0.00000000000000000, 0.00462962962962962, 0.03703703703703709,
      0.12500000000000000, 0.29629629629629611, 0.57870370370370305,
      1.00000000000000000});
  ASSERT_EQ(static_cast<int>(parameter_values.size()), num_basis_functions);
  ASSERT_EQ(static_cast<int>(expected_basis_function_values.size()),
            num_basis_functions);
  for (int i = 0; i < num_basis_functions; ++i) {
    const int num_parameter_values =
        static_cast<int>(parameter_values[i].size());
    ASSERT_EQ(static_cast<int>(expected_basis_function_values[i].size()),
              num_parameter_values);
    for (int j = 0; j < num_parameter_values; ++j) {
      EXPECT_NEAR(basis.BasisFunctionValue(i, parameter_values[i][j]),
                  expected_basis_function_values[i][j],
                  std::numeric_limits<double>::epsilon());
    }
  }
}

}  // namespace math
}  // namespace drake
