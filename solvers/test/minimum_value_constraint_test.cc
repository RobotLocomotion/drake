#include "drake/solvers/minimum_value_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/test_utilities/check_constraint_eval_nonsymbolic.h"

namespace drake {
namespace solvers {
namespace {

double kEps{std::numeric_limits<double>::epsilon()};

// Returns the element-wise square of it's input.
template <typename T>
VectorX<T> SquareAndReturnAll(const Eigen::Ref<const VectorX<T>>& x, double) {
  return x.array().square();
}

// Returns the elements of the element-wise square of its input
// that are less than `influence_value`.
template <typename T>
VectorX<T> SquareAndReturnLessThanInfluenceValue(
    const Eigen::Ref<const VectorX<T>>& x, double influence_value) {
  int max_num_values = static_cast<int>(x.size());
  VectorX<T> values(max_num_values);
  int value_count{0};
  double sqrt_influence_value = std::sqrt(influence_value);
  for (int i = 0; i < max_num_values; ++i) {
    if (x(i) < sqrt_influence_value) {
      values(value_count++) = x(i) * x(i);
    }
  }
  values.conservativeResize(value_count);
  return values;
}

// Returns a zero-element vector.
template <typename T>
VectorX<T> ReturnNoValues(const Eigen::Ref<const VectorX<T>>&, double) {
  return VectorX<T>(0);
}

// Verify that the constructor works as expected.
GTEST_TEST(MinimumValueConstraintTests, ConstructorTest) {
  int expected_num_vars{5};
  int expected_max_num_values{3};
  double expected_minimum_value{0.1};
  double expected_influence_value{0.2};
  MinimumValueConstraint dut(expected_num_vars, expected_minimum_value,
                             expected_influence_value - expected_minimum_value,
                             expected_max_num_values,
                             &SquareAndReturnAll<AutoDiffXd>);
  EXPECT_EQ(dut.num_vars(), expected_num_vars);
  EXPECT_EQ(dut.max_num_values(), expected_max_num_values);
  EXPECT_EQ(dut.minimum_value(), expected_minimum_value);
  EXPECT_EQ(dut.influence_value(), expected_influence_value);
  EXPECT_EQ(dut.num_constraints(), 1);
}

// Verify that the non-symbolic versions of Eval() behave as expected.
GTEST_TEST(MinimumValueConstraintTests, EvalNonsymbolicTest) {
  int num_vars{5};
  int max_num_values{5};
  double minimum_value{0.1};
  double influence_value{0.2};
  MinimumValueConstraint dut_return_all(
      num_vars, minimum_value, influence_value - minimum_value, max_num_values,
      &SquareAndReturnAll<AutoDiffXd>, &SquareAndReturnAll<double>);
  MinimumValueConstraint dut_return_less_than_influence_value(
      num_vars, minimum_value, influence_value - minimum_value, max_num_values,
      &SquareAndReturnLessThanInfluenceValue<AutoDiffXd>);
  double tol = kEps;
  AutoDiffVecXd x_0 = AutoDiffVecXd::Zero(num_vars);
  AutoDiffVecXd x_0_to_twice_sqrt_influence_value =
      AutoDiffVecXd::LinSpaced(num_vars, 0, 2 * std::sqrt(influence_value));
  AutoDiffVecXd x_sqrt_min_value_to_twice_sqrt_influence_value =
      AutoDiffVecXd::LinSpaced(num_vars, std::sqrt(minimum_value),
                               2 * std::sqrt(influence_value));
  auto check_constraints = [&](std::vector<AutoDiffVecXd> inputs,
                               bool should_constraints_be_satisfied) {
    for (const AutoDiffVecXd& x : inputs) {
      test::CheckConstraintEvalNonsymbolic(dut_return_all, x, tol);
      test::CheckConstraintEvalNonsymbolic(dut_return_less_than_influence_value,
                                           x, tol);
      AutoDiffVecXd y_return_all, y_return_less_than_influence_value;
      dut_return_all.Eval(x, &y_return_all);
      dut_return_less_than_influence_value.Eval(
          x, &y_return_less_than_influence_value);
      ASSERT_EQ(y_return_all.size(), 1);
      ASSERT_EQ(y_return_less_than_influence_value.size(), 1);
      EXPECT_EQ(y_return_all(0), y_return_less_than_influence_value(0));
      EXPECT_EQ(dut_return_all.CheckSatisfied(x, kEps),
                should_constraints_be_satisfied);
      EXPECT_EQ(dut_return_less_than_influence_value.CheckSatisfied(x, kEps),
                should_constraints_be_satisfied);
    }
  };
  // Check with inputs that should violate the constraints.
  check_constraints({x_0, x_0_to_twice_sqrt_influence_value,
                     x_sqrt_min_value_to_twice_sqrt_influence_value -
                         AutoDiffVecXd::Constant(num_vars, kEps)},
                    false);
  // Check with inputs that should satisfy the constraints.
  check_constraints({x_sqrt_min_value_to_twice_sqrt_influence_value}, true);
}

GTEST_TEST(MinimumValueConstraintTests, EvalNoValuesTest) {
  int num_vars{5};
  int max_num_values{0};
  double minimum_value{0.1};
  double influence_value{0.2};
  MinimumValueConstraint dut_no_values(
      num_vars, minimum_value, influence_value - minimum_value, max_num_values,
      &ReturnNoValues<AutoDiffXd>);
  test::CheckConstraintEvalNonsymbolic(dut_no_values,
                                       AutoDiffVecXd::Zero(num_vars), kEps);
}

// Verify that Eval() throws for symbolic inputs.
GTEST_TEST(MinimumValueConstraintTests, EvalSymbolicTest) {
  int num_vars{5};
  int max_num_values{5};
  double minimum_value{0.1};
  double influence_value{0.2};
  MinimumValueConstraint dut(num_vars, minimum_value,
                             influence_value - minimum_value, max_num_values,
                             &SquareAndReturnAll<AutoDiffXd>);
  VectorX<symbolic::Variable> x{num_vars};
  VectorX<symbolic::Expression> y;
  EXPECT_THROW(dut.Eval(x, &y), std::logic_error);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
