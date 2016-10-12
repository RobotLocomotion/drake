#include "drake/common/symbolic_variable.h"

#include <utility>

#include "gtest/gtest.h"

namespace drake {
namespace symbolic {
namespace {

using std::move;

// Provides common variables that are used by the following tests.
class SymbolicVariableTest : public ::testing::Test {
 protected:
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
};

TEST_F(SymbolicVariableTest, GetId) {
  const Variable x_prime{"x"};
  EXPECT_NE(x.get_id(), x_prime.get_id());
}

TEST_F(SymbolicVariableTest, GetName) {
  const Variable x_prime{"x"};
  EXPECT_EQ(x.get_name(), x_prime.get_name());
}

TEST_F(SymbolicVariableTest, MoveCopyPreserveId) {
  Variable x{"x"};
  const size_t x_id{x.get_id()};
  const size_t x_hash{x.get_hash()};
  const Variable x_copied{x};
  const Variable x_moved{move(x)};
  EXPECT_EQ(x_id, x_copied.get_id());
  EXPECT_EQ(x_hash, x_copied.get_hash());
  EXPECT_EQ(x_id, x_moved.get_id());
  EXPECT_EQ(x_hash, x_moved.get_hash());
}

TEST_F(SymbolicVariableTest, Lt) {
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

TEST_F(SymbolicVariableTest, Eq) {
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

TEST_F(SymbolicVariableTest, ToString) {
  EXPECT_EQ(x.to_string(), "x");
  EXPECT_EQ(y.to_string(), "y");
  EXPECT_EQ(z.to_string(), "z");
  EXPECT_EQ(w.to_string(), "w");
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
