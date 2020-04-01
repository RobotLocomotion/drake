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
  // Check the order and num_basis_functions constructor with kClampedUniform.
  BsplineBasis<double> bspline_basis_0{expected_order,
                                       expected_num_basis_functions,
                                       KnotVectorType::kClampedUniform};
  EXPECT_EQ(bspline_basis_0.order(), expected_order);
  EXPECT_EQ(bspline_basis_0.num_basis_functions(),
            expected_num_basis_functions);
  EXPECT_EQ(bspline_basis_0.knots(), expected_knots_0);

  // Check that the order and num_basis_functions constructor defaults to
  // kClampedUniform.
  EXPECT_EQ(bspline_basis_0,
            BsplineBasis<double>(expected_order, expected_num_basis_functions));

  // Check the order and num_basis_functions constructor with kUniform.
  BsplineBasis<double> bspline_basis_1{
      expected_order, expected_num_basis_functions, KnotVectorType::kUniform};
  EXPECT_EQ(bspline_basis_1.order(), expected_order);
  EXPECT_EQ(bspline_basis_1.num_basis_functions(),
            expected_num_basis_functions);
  EXPECT_EQ(bspline_basis_1.knots(), expected_knots_1);

  // Check the order and knots constructor.
  const std::vector<double> expected_knots_2 = {-5, 0, 0.1, 0.2, 0.3, 0.4, 3,
                                                10, 17, 25, 50, 100, 200, 400,
                                                401};
  BsplineBasis<double> bspline_basis_2{expected_order, expected_knots_2};
  EXPECT_EQ(bspline_basis_2.order(), expected_order);
  EXPECT_EQ(bspline_basis_2.num_basis_functions(),
            expected_num_basis_functions);
  EXPECT_EQ(bspline_basis_2.knots(), expected_knots_2);
}

GTEST_TEST(BsplineBasisTests, ConstructorErrors) {
  const int order = 4;
  const char* expected_message_0 =
      "The number of basis functions (.*) should be greater than or equal to "
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
  /* For a 5-th order B-spline basis with 14 basis functions (k = 5, n = 13),
  the clamped, uniform knot vector from 0 to 1 is
    [0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1, 1, 1, 1].
  For i = {0, ..., 9}, t ∈ (0.1 * i, 0.1 * (i + 1)) implies that the k basis
  functions with indices {i, ..., i + k - 1} are active (potentially non-zero)
  while all other basis functions are in-active (definitely zero). By convention
  the active basis functions at t = i * 0.1 are defined to be the same as those
  for the following interval, despite the fact that the i-th basis function is
  also zero at that point. Similarly, the active basis functions at t = 1, are
  defined to be the same as those for the preceeding interval, despite the fact
  that basis function 9 is zero at that point. */
  const int expected_order = 5;
  const int expected_num_basis_functions = 14;
  BsplineBasis<double> bspline_basis_0{expected_order,
                                       expected_num_basis_functions};

  auto expected_indices = [](std::initializer_list<int> values) {
        return std::vector<int>(values);
  };

  // Test that the active basis functions for the full final interval are the
  // last five (and only the last five).
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices({{0.9, 1.0}}),
            expected_indices({9, 10, 11, 12, 13}));

  // Test that the active basis functions for part of the final interval are
  // the last five (and only the last five).
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices({{0.95, 1.0}}),
            expected_indices({9, 10, 11, 12, 13}));

  // Test that extending the query interval back into the previous knot interval
  // adds 8 to the list of active basis function indices.
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices({{0.85, 1.0}}),
            expected_indices({8,  9,  10, 11, 12, 13}));

  // Test an query interval that extends across three knot intervals.
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices({{0.35, 0.59}}),
            expected_indices({3, 4, 5, 6, 7, 8,  9}));

  // Test that the first five basis functions are active when
  // t = initial_parameter_value().
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices(0.0),
            expected_indices({0, 1, 2, 3, 4}));

  // Test an intermediate point, t = 0.43 ∈ (0.1 * i, 0.1 * (i + 1)), for i = 4.
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices(0.43),
            expected_indices({4, 5, 6, 7, 8}));

  // Test that the last five basis functions are active when
  // t = final_parameter_value().
  EXPECT_EQ(bspline_basis_0.ComputeActiveBasisFunctionIndices(1.0),
            expected_indices({9, 10, 11, 12, 13}));

  BsplineBasis<double> bspline_basis_1{1, expected_num_basis_functions};

  // Test that the for a 1-st order basis, only the first basis function is
  // active when t = initial_parameter_value().
  EXPECT_EQ(bspline_basis_1.ComputeActiveBasisFunctionIndices(0),
            expected_indices({0}));
}

/* Compares values returned by BsplineBasis::EvaluateBasisFunctionI() to those
generated by scipy.interpolate.Bspline.basis_element():
    import numpy as np
    from scipy.interpolate import BSpline
    knots = [0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1]
    order = 4
    num_basis_functions = len(knots) - order
    for i in range(0, num_basis_functions):
        bspline = BSpline.basis_element(knots[i:(i+order+1)],
                                        extrapolate=False)
        u = np.union1d(np.linspace(bspline.t[order - 1],
                                   bspline.t[-order],
                                   num_basis_functions),
                       knots)
        print("parameter_values.push_back(std::vector<double>{{{}}});"
              .format(", ".join("{:.17f}".format(n) for n in u)))
        print("expected_basis_function_values.push_back" +
              "(std::vector<double>{{{}}});"
              .format(", ".join("{:.17f}".format(0.0 if np.isnan(n) else n)
                                for n in bspline(u))))
NOTE(avalenzu): Scipy gives B[6](1.0) = 0.0, which is wrong. That value has
been replaced with the correct one (1.0).

This tests (indirectly) the BsplineBasis::EvaluateCurve() method. */
GTEST_TEST(BsplineBasisTests, EvaluateBasisFunctionIScipyComparison) {
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
      EXPECT_NEAR(basis.EvaluateBasisFunctionI(i, parameter_values[i][j]),
                  expected_basis_function_values[i][j],
                  std::numeric_limits<double>::epsilon());
    }
  }
}

// Tests that {initial,final}_parameter_value() behave as expected.
GTEST_TEST(BsplineBasisTests, InitialAndFinalParameterValueTest) {
  const int order{3};
  const std::vector<double> knots{-5, 0, 0.1, 0.2, 0.3, 0.4, 3, 10, 17, 25};
  BsplineBasis<double> bspline_basis{order, knots};
  EXPECT_EQ(bspline_basis.initial_parameter_value(), knots[order - 1]);
  EXPECT_EQ(bspline_basis.final_parameter_value(), knots[knots.size() - order]);
}

// Tests that operator==() behaves as expected.
GTEST_TEST(BsplineBasisTests, OperatorEqualsTest) {
  const int order{3};
  const std::vector<double> knots{-5, 0, 0.1, 0.2, 0.3, 0.4, 3, 10, 17, 25};

  // Test that identically constructed objects are equal.
  EXPECT_TRUE(BsplineBasis<double>(order, knots).operator==(
    BsplineBasis<double>(order, knots)));

  // Test that objects with different orders and the same knots are not equal.
  EXPECT_FALSE(BsplineBasis<double>(order, knots).operator==(
    BsplineBasis<double>(order + 1, knots)));

  // Test that objects with the same orders and different knots are not equal.
  const std::vector<double> other_knots{1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_FALSE(BsplineBasis<double>(order, knots).operator==(
    BsplineBasis<double>(order, other_knots)));

  // Test that objects with different orders and different knots are not equal.
  EXPECT_FALSE(BsplineBasis<double>(order, knots).operator==(
    BsplineBasis<double>(order + 1, other_knots)));
}

}  // namespace math
}  // namespace drake
