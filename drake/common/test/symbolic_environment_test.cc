#include "drake/common/symbolic_environment.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
namespace {

using std::string;
using std::runtime_error;

// Provides common variables that are used by the following tests.
class SymbolicEnvironmentTest : public ::testing::Test {
 protected:
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Variable var_w{"w"};
  const Variable var_v{"v"};
};

TEST_F(SymbolicEnvironmentTest, EmptySize) {
  const Environment env1{};
  EXPECT_TRUE(env1.empty());
  EXPECT_EQ(env1.size(), 0u);

  const Environment env2{{var_x, 2}, {var_y, 3}, {var_z, 4}};
  EXPECT_FALSE(env2.empty());
  EXPECT_EQ(env2.size(), 3u);
}

TEST_F(SymbolicEnvironmentTest, InitWithNan) {
  EXPECT_THROW((Environment{{var_x, 10}, {var_y, NAN}}), runtime_error);
}

TEST_F(SymbolicEnvironmentTest, insert_find) {
  Environment env1{{var_x, 2}, {var_y, 3}, {var_z, 4}};
  const Environment env2{env1};

  env1.insert(var_w, 5);
  EXPECT_EQ(env1.size(), 4u);
  EXPECT_EQ(env2.size(), 3u);

  const auto it1{env1.find(var_w)};
  ASSERT_TRUE(it1 != env1.end());
  EXPECT_EQ(it1->second, 5);

  const auto it2{env1.find(var_v)};
  EXPECT_TRUE(it2 == env1.end());

  env1.insert(var_v, 6);
  EXPECT_EQ(env1.size(), 5u);
}

TEST_F(SymbolicEnvironmentTest, ToString) {
  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  const string out{env.to_string()};
  EXPECT_TRUE(out.find("x -> 2") != string::npos);
  EXPECT_TRUE(out.find("y -> 3") != string::npos);
  EXPECT_TRUE(out.find("z -> 3") != string::npos);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
