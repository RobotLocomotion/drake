#include "drake/common/symbolic_environment.h"

#include <string>

#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "gtest/gtest.h"

namespace drake {
namespace symbolic {
namespace {

using std::string;

GTEST_TEST(SymEnvTest, empty_size) {
  Environment const env1{};
  EXPECT_TRUE(env1.empty());
  EXPECT_EQ(env1.size(), 0u);

  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Environment const env2{{var_x, 2}, {var_y, 3}, {var_z, 4}};
  EXPECT_FALSE(env2.empty());
  EXPECT_EQ(env2.size(), 3u);
}

GTEST_TEST(SymEnvTest, insert_find) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Variable const var_w{"w"};
  Variable const var_v{"v"};
  Environment env1{{var_x, 2}, {var_y, 3}, {var_z, 4}};
  Environment const env2{env1};

  env1.insert(var_w, 5);
  EXPECT_EQ(env1.size(), 4u);
  EXPECT_EQ(env2.size(), 3u);

  auto const it1 = env1.find(var_w);
  EXPECT_TRUE(it1 != env1.end());
  EXPECT_EQ(it1->second, 5);

  auto const it2 = env1.find(var_v);
  EXPECT_TRUE(it2 == env1.end());

  env1.insert(var_v, 6);
  EXPECT_EQ(env1.size(), 5u);
}

GTEST_TEST(SymEnvTest, output_opreator) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};

  string const out{env.to_string()};
  EXPECT_TRUE(out.find("x -> 2") != string::npos);
  EXPECT_TRUE(out.find("y -> 3") != string::npos);
  EXPECT_TRUE(out.find("z -> 3") != string::npos);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
