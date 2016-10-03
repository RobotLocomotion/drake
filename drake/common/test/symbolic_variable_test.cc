#include "drake/common/symbolic_variable.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/symbolic_expression.h"
#include "gtest/gtest.h"

namespace drake {
namespace symbolic {
namespace core {
namespace test {
namespace {

using std::cerr;
using std::endl;
using std::equal_to;
using std::ostringstream;
using std::string;
using std::to_string;
using std::runtime_error;

GTEST_TEST(SymVariableTest, get_id) {
  Variable x{"x"};
  Variable x_{"x"};
  EXPECT_NE(x.get_id(), x_.get_id());
}

GTEST_TEST(SymVariableTest, get_name) {
  Variable x{"x"};
  Variable x_{"x"};
  EXPECT_EQ(x.get_name(), x_.get_name());
}

GTEST_TEST(SymVariableTest, operator_lt) {
  Variable x{"x"};
  Variable y{"y"};
  Variable z{"z"};
  Variable w{"w"};

  EXPECT_FALSE(x < x);
  EXPECT_TRUE(x < y);
  EXPECT_TRUE(x < z);
  EXPECT_TRUE(x < w);

  EXPECT_FALSE(y < x);
  EXPECT_FALSE(y < y);
  EXPECT_TRUE(y < z);
  EXPECT_TRUE(y < w);

  EXPECT_FALSE(z < x);
  EXPECT_FALSE(z < y);
  EXPECT_FALSE(z < z);
  EXPECT_TRUE(z < w);

  EXPECT_FALSE(w < x);
  EXPECT_FALSE(w < y);
  EXPECT_FALSE(w < z);
  EXPECT_FALSE(w < w);
}

GTEST_TEST(SymVariableTest, operator_eq) {
  Variable x{"x"};
  Variable y{"y"};
  Variable z{"z"};
  Variable w{"w"};

  EXPECT_TRUE(x == x);
  EXPECT_FALSE(x == y);
  EXPECT_FALSE(x == z);
  EXPECT_FALSE(x == w);

  EXPECT_FALSE(y == x);
  EXPECT_TRUE(y == y);
  EXPECT_FALSE(y == z);
  EXPECT_FALSE(y == w);

  EXPECT_FALSE(z == x);
  EXPECT_FALSE(z == y);
  EXPECT_TRUE(z == z);
  EXPECT_FALSE(z == w);

  EXPECT_FALSE(w == x);
  EXPECT_FALSE(w == y);
  EXPECT_FALSE(w == z);
  EXPECT_TRUE(w == w);
}

GTEST_TEST(SymVariableTest, output_operator) {
  Variable x{"x"};
  Variable y{"y"};
  Variable z{"z"};
  Variable w{"w"};

  EXPECT_EQ(to_string(x), "x");
  EXPECT_EQ(to_string(y), "y");
  EXPECT_EQ(to_string(z), "z");
  EXPECT_EQ(to_string(w), "w");
}

}  // namespace
}  // namespace test
}  // namespace core
}  // namespace symbolic
}  // namespace drake
