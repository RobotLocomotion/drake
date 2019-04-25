#include <iostream>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <fmt/format.h>

#include "drake/common/symbolic.h"
#include "drake/common/symbolic_cond_literals.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

using std::vector;

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

class SymbolicExpressionTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
};

// TODO(jwnimmer-tri) Test use of true/false literals as conditions.

// Tests conditionally assigning to a result.
template <typename T>
std::pair<T, T> MinMax(const T& a, const T& b) {
  using namespace drake::symbolic::branching_literals;
  std::pair<T, T> result{};
  lazy_assign(&result.first, &result.second) =
  lazy_if (a < b) ^[&]() {
    result = std::make_pair(a, b);
  } || lazy_else ^[&]() {
    result = std::make_pair(b, a);
  };
  return result;
}
TEST_F(SymbolicExpressionTest, MinMax) {
  EXPECT_EQ(MinMax(2.0, 1.0), std::make_pair(1.0, 2.0));
  auto result = MinMax(x_, y_);
  EXPECT_PRED2(ExprEqual, result.first, if_then_else(x_ < y_, x_, y_));
  EXPECT_PRED2(ExprEqual, result.second, if_then_else(x_ < y_, y_, x_));
}

// Tests conditionally raising an exception.
template <typename T>
T CheckedSqrt(const T& a) {
  using std::sqrt;
  using namespace drake::symbolic::branching_literals;
  T result{};
  lazy_assign(&result) =
  lazy_if (a < 0) ^[&]() {
    throw std::logic_error("CheckedSqrt requires a positive value");
  } || lazy_else ^[&]() {
    result = sqrt(a);
  };
  return result;
}
TEST_F(SymbolicExpressionTest, CheckedSqrt) {
  EXPECT_EQ(CheckedSqrt(4.0), 2.0);
  EXPECT_THROW(CheckedSqrt(x_), std::exception);
#if 0  // We want this to pass, but it doesn't yet.
  EXPECT_PRED2(ExprEqual, CheckedSqrt(Expression{4.0}), 2.0);
#endif
}

// Tests that elif predicates remain unevaluated when a prior condition has
// already matched.
template <typename T>
boolean<T> ReturnsBoolButAlwaysThrows() {
  throw std::logic_error("ReturnsBoolButAlwaysThrows was invoked");
  return boolean<T>(false);
}
template <typename T>
T Identity(const T& a) {
  using namespace drake::symbolic::branching_literals;
  T result{};
  lazy_assign(&result) =
  lazy_if (a == a) ^[&]() {
    result = a;
  } || lazy_elif (ReturnsBoolButAlwaysThrows<T>()) ^[&]() {
    throw std::logic_error("Wrong branch in Identity");
  };
  return result;
}
TEST_F(SymbolicExpressionTest, Identity) {
  EXPECT_EQ(Identity(1.0), 1.0);
#if 0  // We want this to pass, but it doesn't yet.
  EXPECT_PRED2(ExprEqual, Identity(Expression{1.0}), 1.0);
  EXPECT_PRED2(ExprEqual, Identity(x_), x_);
#endif
}

// Tests side effects that aren't scalar assignments.
template <typename T>
T AbsLogWhenNegative(const T& a, std::ostream& out) {
  using namespace drake::symbolic::branching_literals;
  T result{};
  lazy_assign(&result) =
  lazy_if (a >= 0) ^[&]() {
    result = a;
  } || lazy_else ^[&]() {
    out << fmt::format("a = '{}' was negative", a);
    result = -a;
  };
  return result;
}
TEST_F(SymbolicExpressionTest, AbsLogWhenNegative) {
  std::stringstream out;
  EXPECT_EQ(AbsLogWhenNegative(1.0, out), 1.0);
  EXPECT_EQ(out.str(), "");
  EXPECT_EQ(AbsLogWhenNegative(-1.0, out), 1.0);
  EXPECT_EQ(out.str(), "a = '-1' was negative");
  // TODO Add Expression tests.
}

// Tests nested conditional assignments.
// TODO Write me.

}  // namespace
}  // namespace symbolic
}  // namespace drake
