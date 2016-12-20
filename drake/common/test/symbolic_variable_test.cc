#include "drake/common/symbolic_variable.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace drake {
namespace symbolic {
namespace {

using std::move;
using std::unordered_map;
using std::unordered_set;
using std::vector;

// Provides common variables that are used by the following tests.
class SymbolicVariableTest : public ::testing::Test {
 protected:
  const Variable w_{"w"};
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicVariableTest, GetName) {
  const Variable x_prime{"x"};
  EXPECT_EQ(x_.name(), x_prime.name());
}

TEST_F(SymbolicVariableTest, MoveCopy) {
  Variable x{"x"};
  const size_t x_hash{x.hash()};
  const Variable x_copied{x};
  const Variable x_moved{move(x)};
  EXPECT_EQ(x_hash, x_copied.hash());
  EXPECT_EQ(x_hash, x_moved.hash());
}

TEST_F(SymbolicVariableTest, Lt) {
  EXPECT_FALSE(w_ < w_);
  EXPECT_TRUE(w_ < x_);
  EXPECT_TRUE(w_ < y_);
  EXPECT_TRUE(w_ < z_);

  EXPECT_FALSE(x_ < w_);
  EXPECT_FALSE(x_ < x_);
  EXPECT_TRUE(x_ < y_);
  EXPECT_TRUE(x_ < z_);

  EXPECT_FALSE(y_ < w_);
  EXPECT_FALSE(y_ < x_);
  EXPECT_FALSE(y_ < y_);
  EXPECT_TRUE(y_ < z_);

  EXPECT_FALSE(z_ < w_);
  EXPECT_FALSE(z_ < x_);
  EXPECT_FALSE(z_ < y_);
  EXPECT_FALSE(z_ < z_);
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

// This test checks whether symbolic::Variable is compatible with
// std::unordered_set.
TEST_F(SymbolicVariableTest, CompatibleWithUnorderedSet) {
  unordered_set<Variable, hash_value<Variable>> uset;
  uset.emplace(x_);
  uset.emplace(y_);
}

// This test checks whether symbolic::Variable is compatible with
// std::unordered_map.
TEST_F(SymbolicVariableTest, CompatibleWithUnorderedMap) {
  unordered_map<Variable, Variable, hash_value<Variable>> umap;
  umap.emplace(x_, y_);
}

// This test checks whether symbolic::Variable is compatible with
// std::vector.
TEST_F(SymbolicVariableTest, CompatibleWithVector) {
  vector<Variable> vec;
  vec.push_back(x_);
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
