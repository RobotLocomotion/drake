#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace {

using std::runtime_error;
using std::string;

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

TEST_F(EnvironmentTest, InitWithMap) {
  Environment::map m;
  m.emplace(var_x_, 3.0);
  m.emplace(var_y_, 4.0);
  const Expression e{var_x_ + var_y_};
  EXPECT_EQ(e.Evaluate(Environment{m}), 7.0);
}

TEST_F(EnvironmentTest, InitWithMapExceptionDummyVariable) {
  Environment::map m;
  const Variable dummy;
  m.emplace(dummy, 3.0);
  EXPECT_THROW(Environment{m}, runtime_error);
}

TEST_F(EnvironmentTest, InitWithMapExceptionNan) {
  Environment::map m;
  m.emplace(var_x_, NAN);
  EXPECT_THROW(Environment{m}, runtime_error);
}

TEST_F(EnvironmentTest, InsertFind) {
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

TEST_F(EnvironmentTest, InsertMultipleItemsFind) {
  const auto x = MakeMatrixContinuousVariable<3, 4>("x");
  Eigen::Matrix<double, 3, 4> v;
  v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0;

  Environment env;
  env.insert(x, v);
  EXPECT_EQ(env.size(), 12);

  for (int j = 0; j < 4; ++j) {
    for (int i = 0; i < 3; ++i) {
      const auto it(env.find(x(i, j)));
      ASSERT_TRUE(it != env.end());
      EXPECT_EQ(it->second, v(i, j));
    }
  }
}

TEST_F(EnvironmentTest, InsertMultipleItemsFindSizeMismatch) {
  const auto x = MakeMatrixContinuousVariable<4, 3>("x");
  Eigen::Matrix<double, 3, 4> v;
  v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0;
  Environment env;
  // size of x : 4 x 3.
  // size of v : 3 x 4.
  EXPECT_THROW(env.insert(x, v), std::runtime_error);
}

TEST_F(EnvironmentTest, domain) {
  const Environment env1{};
  const Environment env2{{var_x_, 2}, {var_y_, 3}, {var_z_, 4}};
  EXPECT_EQ(env1.domain(), Variables{});
  EXPECT_EQ(env2.domain(), Variables({var_x_, var_y_, var_z_}));
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

TEST_F(EnvironmentTest, LookupOperator) {
  Environment env{{var_x_, 2}};
  const Environment const_env{{var_x_, 2}};
  EXPECT_EQ(env[var_x_], 2);
  EXPECT_EQ(const_env[var_x_], 2);
  EXPECT_EQ(env[var_y_], 0);
  EXPECT_THROW(const_env[var_y_], runtime_error);
  EXPECT_EQ(env.size(), 2u);
  EXPECT_EQ(const_env.size(), 1u);
  EXPECT_THROW(env[var_dummy_], runtime_error);
  EXPECT_THROW(const_env[var_dummy_], runtime_error);
}

TEST_F(EnvironmentTest, PopulateRandomVariables) {
  const Variable uni{"uni", Variable::Type::RANDOM_UNIFORM};
  const Variable gau{"gau", Variable::Type::RANDOM_GAUSSIAN};
  const Variable exp{"exp", Variable::Type::RANDOM_EXPONENTIAL};
  const Environment env1{{var_x_, 2}};
  RandomGenerator g{};

  // PopulateRandomVariables should add entries for the three random variables.
  const Environment env2{
      PopulateRandomVariables(env1, {var_x_, uni, gau, exp}, &g)};
  EXPECT_EQ(env2.size(), 4);
  EXPECT_TRUE(env2.find(uni) != env1.end());
  EXPECT_TRUE(env2.find(gau) != env1.end());
  EXPECT_TRUE(env2.find(exp) != env1.end());

  // PopulateRandomVariables should add entries for the unassigned random
  // variables, gau and exp. But it should keep the original assignment `uni ↦
  // 10.0`.
  const Environment env3{{var_x_, 2}, {uni, 10.0}};
  const Environment env4{
      PopulateRandomVariables(env3, {var_x_, uni, gau, exp}, &g)};
  EXPECT_EQ(env4.size(), 4);
  EXPECT_TRUE(env4.find(uni) != env1.end());
  EXPECT_TRUE(env4.find(gau) != env1.end());
  EXPECT_TRUE(env4.find(exp) != env1.end());
  EXPECT_EQ(env4[uni], 10.0);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
