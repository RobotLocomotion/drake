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
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
};

TEST_F(SymbolicVariableTest, GetId) {
  const Variable x_prime{"x"};
  EXPECT_NE(x_.get_id(), x_prime.get_id());
}

TEST_F(SymbolicVariableTest, GetName) {
  const Variable x_prime{"x"};
  EXPECT_EQ(x_.get_name(), x_prime.get_name());
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
  EXPECT_FALSE(x_ < x_);
  EXPECT_TRUE(x_ < y_);
  EXPECT_TRUE(x_ < z_);
  EXPECT_TRUE(x_ < w_);

  EXPECT_FALSE(y_ < x_);
  EXPECT_FALSE(y_ < y_);
  EXPECT_TRUE(y_ < z_);
  EXPECT_TRUE(y_ < w_);

  EXPECT_FALSE(z_ < x_);
  EXPECT_FALSE(z_ < y_);
  EXPECT_FALSE(z_ < z_);
  EXPECT_TRUE(z_ < w_);

  EXPECT_FALSE(w_ < x_);
  EXPECT_FALSE(w_ < y_);
  EXPECT_FALSE(w_ < z_);
  EXPECT_FALSE(w_ < w_);
}

TEST_F(SymbolicVariableTest, Eq) {
  EXPECT_TRUE(x_ == x_);
  EXPECT_FALSE(x_ == y_);
  EXPECT_FALSE(x_ == z_);
  EXPECT_FALSE(x_ == w_);

  EXPECT_FALSE(y_ == x_);
  EXPECT_TRUE(y_ == y_);
  EXPECT_FALSE(y_ == z_);
  EXPECT_FALSE(y_ == w_);

  EXPECT_FALSE(z_ == x_);
  EXPECT_FALSE(z_ == y_);
  EXPECT_TRUE(z_ == z_);
  EXPECT_FALSE(z_ == w_);

  EXPECT_FALSE(w_ == x_);
  EXPECT_FALSE(w_ == y_);
  EXPECT_FALSE(w_ == z_);
  EXPECT_TRUE(w_ == w_);
}

TEST_F(SymbolicVariableTest, ToString) {
  EXPECT_EQ(x_.to_string(), "x");
  EXPECT_EQ(y_.to_string(), "y");
  EXPECT_EQ(z_.to_string(), "z");
  EXPECT_EQ(w_.to_string(), "w");
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
