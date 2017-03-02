#include "drake/common/monomial.h"

#include <unordered_map>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"

using std::unordered_map;

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

class MonomialTest : public ::testing::Test {
 protected:
  const Variable var_w_{"w"};
  const Variable var_z_{"z"};
  const Variable var_y_{"y"};
  const Variable var_x_{"x"};

  const Expression w_{var_w_};
  const Expression z_{var_z_};
  const Expression y_{var_y_};
  const Expression x_{var_x_};

  void SetUp() override {
    EXPECT_TRUE(var_y_ < var_x_);
    EXPECT_TRUE(var_z_ < var_y_);
    EXPECT_TRUE(var_w_ < var_z_);
  }
};

TEST_F(MonomialTest, MonomialOne) {
  // Compares monomials all equal to 1, but with different variables.
  Monomial m1{};
  Monomial m2({{var_x_.get_id(), 0}});
  Monomial m3({{var_x_.get_id(), 0}, {var_y_.get_id(), 0}});
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m1, m3);
  EXPECT_EQ(m2, m3);
}

TEST_F(MonomialTest, MonomialWithZeroExponent) {
  // Compares monomials containing zero exponent, such as x^0 * y^2
  Monomial m1({{var_y_.get_id(), 2}});
  Monomial m2({{var_x_.get_id(), 0}, {var_y_.get_id(), 2}});
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m2.get_powers().size(), 1);
  std::map<Variable::Id, int> power_expected;
  power_expected.emplace(var_y_.get_id(), 2);
  EXPECT_EQ(m2.get_powers(), power_expected);
}

TEST_F(MonomialTest, Monomial) {
  // clang-format off
  EXPECT_PRED2(
      ExprEqual,
      GetMonomial(unordered_map<Variable, int, hash_value<Variable>>{}),
      Expression{1.0});

  EXPECT_PRED2(ExprEqual, GetMonomial({{var_x_, 1}}),
               x_);

  EXPECT_PRED2(ExprEqual, GetMonomial({{var_y_, 1}}),
               y_);

  EXPECT_PRED2(ExprEqual, GetMonomial({{var_x_, 1}, {var_y_, 1}}),
               x_ * y_);

  EXPECT_PRED2(ExprEqual, GetMonomial({{var_x_, 2}, {var_y_, 3}}),
               x_ * x_ * y_ * y_ * y_);

  EXPECT_PRED2(ExprEqual, GetMonomial({{var_x_, 1}, {var_y_, 2}, {var_z_, 3}}),
               pow(x_, 1) * pow(y_, 2) * pow(z_, 3));
  // clang-format on
}

TEST_F(MonomialTest, MonomialBasis_x_0) {
  const drake::Vector1<Expression> basis1{MonomialBasis({var_x_}, 0)};
  const auto basis2 = MonomialBasis<1, 0>({var_x_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 1);
  drake::Vector1<Expression> expected;

  // MonomialBasis({x}, 0)
  expected << Expression{1.0};

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_2) {
  const drake::Vector3<Expression> basis1{MonomialBasis({var_x_}, 2)};
  const auto basis2 = MonomialBasis<1, 2>({var_x_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 3);
  drake::Vector3<Expression> expected;

  // MonomialBasis({x}, 2)
  // clang-format off
  expected << x_ * x_,
              x_,
              1.0;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_0) {
  const drake::VectorX<Expression> basis1{MonomialBasis({var_x_, var_y_}, 0)};
  const auto basis2 = MonomialBasis<2, 0>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 1);
  drake::Vector1<Expression> expected;

  // MonomialBasis({x, y}, 0)
  expected << Expression{1.0};

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_1) {
  const drake::VectorX<Expression> basis1{MonomialBasis({var_x_, var_y_}, 1)};
  const auto basis2 = MonomialBasis<2, 1>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 3);
  drake::Vector3<Expression> expected;

  // MonomialBasis({x, y}, 1)
  // clang-format off
  expected << x_,
              y_,
              1.0;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_2) {
  const drake::VectorX<Expression> basis1{MonomialBasis({var_x_, var_y_}, 2)};
  const auto basis2 = MonomialBasis<2, 2>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 6);
  drake::Vector6<Expression> expected;

  // MonomialBasis({x, y}, 2)
  // clang-format off
  expected << x_ * x_,
              x_ * y_,
              y_ * y_,
              x_,
              y_,
              1.0;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_3) {
  const drake::VectorX<Expression> basis1{MonomialBasis({var_x_, var_y_}, 3)};
  const auto basis2 = MonomialBasis<2, 3>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 10);
  Eigen::Matrix<Expression, 10, 1> expected;

  // MonomialBasis({x, y}, 3)
  // clang-format off
  expected << pow(x_, 3),
              (pow(x_, 2) * y_),
              (x_ * pow(y_, 2)),
              pow(y_, 3),
              pow(x_, 2),
              (x_ * y_),
              pow(y_, 2),
              x_,
              y_,
              1;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_z_2) {
  const drake::VectorX<Expression> basis1{
      MonomialBasis({var_x_, var_y_, var_z_}, 2)};
  const auto basis2 = MonomialBasis<3, 2>({var_x_, var_y_, var_z_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 10);

  Eigen::Matrix<Expression, 10, 1> expected;

  // MonomialBasis({x, y, z}, 2)
  // clang-format off
  expected << pow(x_, 2),
              (x_ * y_),
              pow(y_, 2),
              (x_ * z_),
              (y_ * z_),
              pow(z_, 2),
              x_,
              y_,
              z_,
              1;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_z_3) {
  const drake::VectorX<Expression> basis1{
      MonomialBasis({var_x_, var_y_, var_z_}, 3)};
  const auto basis2 = MonomialBasis<3, 3>({var_x_, var_y_, var_z_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 20);
  Eigen::Matrix<Expression, 20, 1> expected;

  // MonomialBasis({x, y, z}, 3)
  // clang-format off
  expected << pow(x_, 3),
              (pow(x_, 2) * y_),
              (x_ * pow(y_, 2)),
              pow(y_, 3),
              (pow(x_, 2) * z_),
              (y_ * x_ * z_),
              (pow(y_, 2) * z_),
              (x_ * pow(z_, 2)),
              (y_ * pow(z_, 2)),
              pow(z_, 3),
              pow(x_, 2),
              (x_ * y_),
              pow(y_, 2),
              (x_ * z_),
              (y_ * z_),
              pow(z_, 2),
              x_,
              y_,
              z_,
              1;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_z_w_3) {
  const drake::VectorX<Expression> basis1{
      MonomialBasis({var_x_, var_y_, var_z_, var_w_}, 3)};
  const auto basis2 = MonomialBasis<4, 3>({var_x_, var_y_, var_z_, var_w_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 35);
  Eigen::Matrix<Expression, 35, 1> expected;

  // MonomialBasis({x, y, z, w}, 3)
  // clang-format off
  expected << pow(x_, 3),
              (pow(x_, 2) * y_),
              (x_ * pow(y_, 2)),
              pow(y_, 3),
              (pow(x_, 2) * z_),
              (x_ * y_ * z_),
              (pow(y_, 2) * z_),
              (x_ * pow(z_, 2)),
              (y_ * pow(z_, 2)),
              pow(z_, 3),
              (pow(x_, 2) * w_),
              (y_ * x_ * w_),
              (pow(y_, 2) * w_),
              (z_ * x_ * w_),
              (z_ * y_ * w_),
              (pow(z_, 2) * w_),
              (x_ * pow(w_, 2)),
              (y_ * pow(w_, 2)),
              (z_ * pow(w_, 2)),
              pow(w_, 3),
              pow(x_, 2),
              (x_ * y_),
              pow(y_, 2),
              (x_ * z_),
              (y_ * z_),
              pow(z_, 2),
              (x_ * w_),
              (y_ * w_),
              (z_ * w_),
              pow(w_, 2),
              x_,
              y_,
              z_,
              w_,
              1;
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

// This test shows that we can have a std::unordered_map whose key is of
// Monomial.
TEST_F(MonomialTest, UnorderedMapOfMonomial) {
  unordered_map<Monomial, double, hash_value<Monomial>>
      monomial_to_coeff_map;
  Monomial x_3{var_x_, 3};
  Monomial y_5{var_y_, 5};
  // Add 2 * x^3
  monomial_to_coeff_map.emplace(x_3, 2);
  // Add -7 * y^5
  monomial_to_coeff_map.emplace(y_5, -7);

  const auto it1 = monomial_to_coeff_map.find(x_3);
  ASSERT_TRUE(it1 != monomial_to_coeff_map.end());
  EXPECT_EQ(it1->second, 2);

  const auto it2 = monomial_to_coeff_map.find(y_5);
  ASSERT_TRUE(it2 != monomial_to_coeff_map.end());
  EXPECT_EQ(it2->second, -7);
}

// Converts a constant to monomial.
TEST_F(MonomialTest, ToMonomial0) {
  Monomial expected;
  EXPECT_EQ(Monomial(1), expected);
  EXPECT_EQ(Monomial(pow(x_, 0)), expected);
  EXPECT_THROW(Monomial(2), std::exception);
}

// Converts expression x to monomial.
TEST_F(MonomialTest, ToMonomial1) {
  Monomial expected(var_x_, 1);
  EXPECT_EQ(Monomial(x_), expected);
}

// Converts expression x * y to monomial.
TEST_F(MonomialTest, ToMonomial2) {
  std::map<Variable::Id, int> powers;
  powers.emplace(var_x_.get_id(), 1);
  powers.emplace(var_y_.get_id(), 1);
  Monomial expected(powers);
  EXPECT_EQ(Monomial(x_ * y_), expected);
}

// Converts expression x^3 to monomial.
TEST_F(MonomialTest, ToMonomial3) {
  Monomial expected(var_x_, 3);
  EXPECT_EQ(Monomial(pow(x_, 3)), expected);
  EXPECT_EQ(Monomial(pow(x_, 2) * x_), expected);
}

// Converts expression x^3 * y to monomial.
TEST_F(MonomialTest, ToMonomial4) {
  std::map<Variable::Id, int> powers;
  powers.emplace(var_x_.get_id(), 3);
  powers.emplace(var_y_.get_id(), 1);
  Monomial expected(powers);
  EXPECT_EQ(Monomial(pow(x_, 3) * y_), expected);
  EXPECT_EQ(Monomial(pow(x_, 2) * y_ * x_), expected);
}

// Converts expression x*(y+z) - x*y to monomial
TEST_F(MonomialTest, ToMonomial5) {
  std::map<Variable::Id, int> powers(
      {{var_x_.get_id(), 1}, {var_z_.get_id(), 1}});
  Monomial expected(powers);
  EXPECT_EQ(Monomial(x_ * z_), expected);
  EXPECT_EQ(Monomial(x_ * (y_ + z_) - x_ * y_), expected);
}

TEST_F(MonomialTest, Degree) {
  const Expression e0{42.0};
  EXPECT_EQ(Degree(e0, {var_x_, var_y_}), 0);
  EXPECT_EQ(Degree(e0), 0);

  const Expression e1{pow(x_, 2)};
  EXPECT_EQ(Degree(e1, {var_x_}), 2);
  EXPECT_EQ(Degree(e1, {var_y_}), 0);
  EXPECT_EQ(Degree(e1, {var_x_, var_y_}), 2);
  EXPECT_EQ(Degree(e1), 2);

  const Expression e2{pow(x_, 2) * pow(y_, 3)};
  EXPECT_EQ(Degree(e2, {var_x_}), 2);
  EXPECT_EQ(Degree(e2, {var_y_}), 3);
  EXPECT_EQ(Degree(e2, {var_z_}), 0);
  EXPECT_EQ(Degree(e2, {var_x_, var_y_}), 5);
  EXPECT_EQ(Degree(e2, {var_x_, var_z_}), 2);
  EXPECT_EQ(Degree(e2, {var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e2, {var_x_, var_y_, var_z_}), 5);
  EXPECT_EQ(Degree(e2), 5);

  const Expression e3{3 + x_ + y_ + z_};
  EXPECT_EQ(Degree(e3, {var_x_}), 1);
  EXPECT_EQ(Degree(e3, {var_y_}), 1);
  EXPECT_EQ(Degree(e3, {var_z_}), 1);
  EXPECT_EQ(Degree(e3, {var_x_, var_y_}), 1);
  EXPECT_EQ(Degree(e3, {var_x_, var_y_, var_z_}), 1);
  EXPECT_EQ(Degree(e3), 1);

  const Expression e4{1 + pow(x_, 2) + pow(y_, 3)};
  EXPECT_EQ(Degree(e4, {var_x_}), 2);
  EXPECT_EQ(Degree(e4, {var_y_}), 3);
  EXPECT_EQ(Degree(e4, {var_x_, var_y_}), 3);
  EXPECT_EQ(Degree(e4, {var_x_, var_z_}), 2);
  EXPECT_EQ(Degree(e4, {var_x_, var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e4), 3);

  const Expression e5{1 + pow(x_, 2) * y_ + x_ * y_ * y_ * z_};
  EXPECT_EQ(Degree(e5, {var_x_}), 2);
  EXPECT_EQ(Degree(e5, {var_y_}), 2);
  EXPECT_EQ(Degree(e5, {var_z_}), 1);
  EXPECT_EQ(Degree(e5, {var_x_, var_y_}), 3);
  EXPECT_EQ(Degree(e5, {var_x_, var_z_}), 2);
  EXPECT_EQ(Degree(e5, {var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e5, {var_x_, var_y_, var_z_}), 4);
  EXPECT_EQ(Degree(e5), 4);

  const Expression e6{pow(x_ + y_ + z_, 3)};
  EXPECT_EQ(Degree(e6, {var_x_}), 3);
  EXPECT_EQ(Degree(e6, {var_y_}), 3);
  EXPECT_EQ(Degree(e6, {var_z_}), 3);
  EXPECT_EQ(Degree(e6, {var_x_, var_y_}), 3);
  EXPECT_EQ(Degree(e6, {var_x_, var_z_}), 3);
  EXPECT_EQ(Degree(e6, {var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e6, {var_x_, var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e6), 3);

  const Expression e7{pow(x_ + x_ * y_ * y_, 2) * y_ +
                      pow(x_ + pow(y_ + x_ * z_, 2), 3)};
  EXPECT_EQ(Degree(e7, {var_x_}), 6);
  EXPECT_EQ(Degree(e7, {var_y_}), 6);
  EXPECT_EQ(Degree(e7, {var_z_}), 6);
  EXPECT_EQ(Degree(e7, {var_x_, var_y_}), 7);
  EXPECT_EQ(Degree(e7, {var_x_, var_z_}), 12);
  EXPECT_EQ(Degree(e7, {var_y_, var_z_}), 6);
  EXPECT_EQ(Degree(e7, {var_x_, var_y_, var_z_}), 12);
  EXPECT_EQ(Degree(e7), 12);

  const Expression e8{pow(x_ + y_ + z_, 3) / 10};
  EXPECT_EQ(Degree(e8, {var_x_}), 3);
  EXPECT_EQ(Degree(e8, {var_y_}), 3);
  EXPECT_EQ(Degree(e8, {var_z_}), 3);
  EXPECT_EQ(Degree(e8, {var_x_, var_y_}), 3);
  EXPECT_EQ(Degree(e8, {var_x_, var_z_}), 3);
  EXPECT_EQ(Degree(e8, {var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e8, {var_x_, var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e8), 3);

  const Expression e9{-pow(y_, 3)};
  EXPECT_EQ(Degree(e9, {var_x_}), 0);
  EXPECT_EQ(Degree(e9, {var_y_}), 3);
  EXPECT_EQ(Degree(e9, {var_z_}), 0);
  EXPECT_EQ(Degree(e9, {var_x_, var_y_}), 3);
  EXPECT_EQ(Degree(e9, {var_x_, var_z_}), 0);
  EXPECT_EQ(Degree(e9, {var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e9, {var_x_, var_y_, var_z_}), 3);
  EXPECT_EQ(Degree(e9), 3);

  const Expression e10{pow(pow(x_, 3), 1.0 / 3)};
  EXPECT_EQ(Degree(e10, {var_x_}), 1);
  EXPECT_EQ(Degree(e10, {var_y_}), 0);
  EXPECT_EQ(Degree(e10, {var_z_}), 0);
  EXPECT_EQ(Degree(e10, {var_x_, var_y_}), 1);
  EXPECT_EQ(Degree(e10, {var_x_, var_z_}), 1);
  EXPECT_EQ(Degree(e10, {var_y_, var_z_}), 0);
  EXPECT_EQ(Degree(e10, {var_x_, var_y_, var_z_}), 1);
  EXPECT_EQ(Degree(e10), 1);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
