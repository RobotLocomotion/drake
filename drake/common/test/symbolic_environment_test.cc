#include "drake/common/symbolic_environment.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
namespace {

using std::string;
using std::runtime_error;

// Provides common variables that are used by the following tests.
class EnvironmentTest : public ::testing::Test {
 protected:
  const Variable var_dummy_{};
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variable var_w_{"w"};
  const Variable var_v_{"v"};
};

TEST_F(EnvironmentTest, EmptySize) {
  const Environment env1{};
  EXPECT_TRUE(env1.empty());
  EXPECT_EQ(env1.size(), 0u);

  const Environment env2{{var_x_, 2}, {var_y_, 3}, {var_z_, 4}};
  EXPECT_FALSE(env2.empty());
  EXPECT_EQ(env2.size(), 3u);
}

TEST_F(EnvironmentTest, InitWithNan) {
  EXPECT_THROW((Environment{{var_x_, 10}, {var_y_, NAN}}), runtime_error);
}

TEST_F(EnvironmentTest, InitializerListWithoutValues) {
  const Environment env{var_x_, var_y_, var_z_};
  for (const auto& p : env) {
    EXPECT_EQ(p.second, 0.0);
  }
}

TEST_F(EnvironmentTest, insert_find) {
  Environment env1{{var_x_, 2}, {var_y_, 3}, {var_z_, 4}};
  const Environment env2{env1};

  env1.insert(var_w_, 5);
  EXPECT_EQ(env1.size(), 4u);
  EXPECT_EQ(env2.size(), 3u);

  const auto it1(env1.find(var_w_));
  ASSERT_TRUE(it1 != env1.end());
  EXPECT_EQ(it1->second, 5);

  const auto it2(env1.find(var_v_));
  EXPECT_TRUE(it2 == env1.end());

  env1.insert(var_v_, 6);
  EXPECT_EQ(env1.size(), 5u);
}

TEST_F(EnvironmentTest, ToString) {
  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  const string out{env.to_string()};
  EXPECT_TRUE(out.find("x -> 2") != string::npos);
  EXPECT_TRUE(out.find("y -> 3") != string::npos);
  EXPECT_TRUE(out.find("z -> 3") != string::npos);
}

TEST_F(EnvironmentTest, DummyVariable1) {
  EXPECT_THROW(Environment({var_dummy_, var_x_}), runtime_error);
  EXPECT_THROW(Environment({{var_dummy_, 1.0}, {var_x_, 2}}), runtime_error);
}

TEST_F(EnvironmentTest, DummyVariable2) {
  Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_THROW(env.insert(var_dummy_, 0.0), runtime_error);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
