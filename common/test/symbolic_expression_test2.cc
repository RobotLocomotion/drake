#include <utility>
#include <vector>

#include <gtest/gtest.h>

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

template <typename T>
std::pair<T, T> min_max(const T& a, const T& b) {
  using namespace drake::symbolic::cond_literals;
  T less;
  T more;
  ""_mutating_only(&less, &more) =
  ""_if (a < b) ^[&]() {
    less = a;
    more = b;
  } || ""_elif (boolean<T>{false}) ^[&]() {
  } || ""_else ^[&]() {
    less = b;
    more = a;
  };
  return std::make_pair(less, more);
}

TEST_F(SymbolicExpressionTest, MinMax) {
  auto result = min_max(x_, y_);
  EXPECT_PRED2(ExprEqual, result.first, if_then_else(x_ < y_, x_, y_));
  EXPECT_PRED2(ExprEqual, result.second, if_then_else(x_ < y_, y_, x_));
}

GTEST_TEST(DoubleTest, ShortCircuit1) {
  using namespace drake::symbolic::cond_literals;
  using T = double;
  T a{0.0};
  ""_mutating_only(&a) =
  ""_if (a >= 0) ^[&]() {
    a -= 1.0;
    std::cout << "Side effect!\n";
  } || ""_else ^[&]() {
    // This will be skipped, because `a` was >= 0.
    DRAKE_DEMAND(false);
  };
  EXPECT_EQ(a, -1.0);
}

bool NeverCalled() {
  DRAKE_DEMAND(false);
  return false;
}

GTEST_TEST(DoubleTest, ShortCircuit2) {
  using namespace drake::symbolic::cond_literals;
  using T = double;
  T a{0.0};
  ""_mutating_only(&a) =
  ""_if (a >= 0) ^[&]() {
  } || ""_elif (NeverCalled()) ^[&]() {
    DRAKE_DEMAND(false);
  };
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
