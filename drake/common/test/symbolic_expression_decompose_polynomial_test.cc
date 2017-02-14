#include "drake/common/symbolic_expression.h"

#include "gtest/gtest.h"

#include "drake/common/hash.h"

namespace drake {
namespace symbolic {
namespace {

// Provides common variables that are used by the following tests.
class SymbolicExpressionDecomposePolynomialTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
};

void CheckMonomialToCoeffMap(const symbolic::Expression& e, const Variables& vars, const Expression::MonomialToCoeffMap& map_expected) {
  Expression::MonomialToCoeffMap map = e.DecomposePolynomial(vars);
  EXPECT_EQ(map.size(), map_expected.size());
  symbolic::Expression e_expected(0);
  for (const auto& p : map) {
    const auto it = map_expected.find(p.first);
    EXPECT_NE(it, map_expected.end());
    EXPECT_TRUE(p.second.EqualTo(it->second));
    e_expected += p.first * p.second;
  }
  EXPECT_TRUE(e.EqualTo(e_expected));
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial0) {
  Expression::MonomialToCoeffMap map_expected;
  map_expected.emplace(x_, 1);
  CheckMonomialToCoeffMap(x_, {var_x_}, map_expected);
  CheckMonomialToCoeffMap(x_, {var_x_, var_y_}, map_expected);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial1) {
  Expression::MonomialToCoeffMap map_expected;
  map_expected.emplace(1, x_);
  CheckMonomialToCoeffMap(x_, {var_y_}, map_expected);
  CheckMonomialToCoeffMap(x_, {var_y_, var_z_}, map_expected);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial2) {
  Expression::MonomialToCoeffMap map_expected;
  map_expected.emplace(1, 2);
  CheckMonomialToCoeffMap(2, {var_x_}, map_expected);
  CheckMonomialToCoeffMap(2, {var_x_, var_y_}, map_expected);

  map_expected.clear();
  map_expected.emplace(1, 3.14159);
  CheckMonomialToCoeffMap(3.14159, {var_x_}, map_expected);
  CheckMonomialToCoeffMap(3.14159, {var_x_, var_y_}, map_expected);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial3) {
  Expression::MonomialToCoeffMap map_expected1;
  Expression::MonomialToCoeffMap map_expected2;
  map_expected1.emplace(x_, 1);
  map_expected1.emplace(1, y_);
  map_expected2.emplace(x_, 1);
  map_expected2.emplace(y_, 1);
  CheckMonomialToCoeffMap(x_ + y_, {var_x_}, map_expected1);
  CheckMonomialToCoeffMap(x_ + y_, {var_x_, var_y_}, map_expected2);
  CheckMonomialToCoeffMap(x_ + y_, {var_x_, var_y_, var_z_}, map_expected2);

  map_expected1.clear();
  map_expected1.emplace(x_, 1);
  map_expected1.emplace(y_, 1);
  map_expected1.emplace(1, 2);
  CheckMonomialToCoeffMap(x_ + y_ + 2, {var_x_, var_y_}, map_expected1);
  CheckMonomialToCoeffMap(x_ + y_ + 2, {var_x_, var_y_, var_z_}, map_expected1);
  CheckMonomialToCoeffMap((x_ + y_) + 2, {var_x_, var_y_}, map_expected1);
  CheckMonomialToCoeffMap((x_ + y_) + 2, {var_x_, var_y_, var_z_}, map_expected1);
  map_expected1[Expression(1)] = 4;
  CheckMonomialToCoeffMap((x_ + 1) + (y_ + 1) + 2, {var_x_, var_y_}, map_expected1);
  CheckMonomialToCoeffMap((x_ + 1) + (y_ + 1) + 2, {var_x_, var_y_, var_z_}, map_expected1);
}

void CheckDecomposePolynomial4(const Expression &e,
                               const Variable &x,
                               const Variable &y) {
  // Decompose 2 * x * x
  EXPECT_TRUE(e.EqualTo(2 * x * x));

  Expression::MonomialToCoeffMap map_expected1;
  map_expected1.emplace(x * x, 2);
  CheckMonomialToCoeffMap(e, {x}, map_expected1);
  CheckMonomialToCoeffMap(e, {x, y}, map_expected1);

  Expression::MonomialToCoeffMap map_expected2;
  map_expected2.emplace(1, 2 * x * x);
  CheckMonomialToCoeffMap(e, {y}, map_expected2);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial4) {
  // Decompose 2 * x * x
  CheckDecomposePolynomial4(2 * x_ * x_, var_x_, var_y_);

  CheckDecomposePolynomial4(2 * pow(x_, 2), var_x_, var_y_);

  CheckDecomposePolynomial4(2 * (x_ * x_), var_x_, var_y_);

  CheckDecomposePolynomial4(x_ * (2 * x_), var_x_, var_y_);
}

void CheckDecomposePolynomial5(const Expression &e,
                               const Variable &x,
                               const Variable &y) {
  // Decompose 6 * x * y
  EXPECT_TRUE(e.EqualTo(6 * x * y));

  Expression::MonomialToCoeffMap map_expected1;
  map_expected1.emplace(x * y, 6);
  CheckMonomialToCoeffMap(e, {x, y}, map_expected1);

  Expression::MonomialToCoeffMap map_expected2;
  map_expected2.emplace(Expression(x), 6 * Expression(y));
  CheckMonomialToCoeffMap(e, {x}, map_expected2);

  Expression::MonomialToCoeffMap map_expected3;
  map_expected3.emplace(Expression(y), 6 * Expression(x));
  CheckMonomialToCoeffMap(e, {y}, map_expected3);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial5) {
  CheckDecomposePolynomial5(6 * x_ * y_, var_x_, var_y_);

  CheckDecomposePolynomial5(6 * (x_ * y_), var_x_, var_y_);

  CheckDecomposePolynomial5((2 * x_) * (3 * y_), var_x_, var_y_);

  CheckDecomposePolynomial5(6 * x_ * y_ * pow(z_, 0), var_x_, var_y_);
}

void CheckDecomposePolynomial6(const Expression& e, const Variable& x, const Variable& y, const Variable& z) {
  // Decomposes 3*x*x*y + 4*y^3*z + 2
  Expression e_expected{3 * x * x *y + 4 * pow(+y, 3)*z + 2};
  EXPECT_TRUE(e.EqualTo(e_expected));

  Expression::MonomialToCoeffMap map_expected1;
  map_expected1.emplace(x * x, 3 * y);
  map_expected1.emplace(1, 4 * pow(+y, 3) * z + 2);
  CheckMonomialToCoeffMap(e, {x}, map_expected1);

  Expression::MonomialToCoeffMap map_expected2;
  map_expected2.emplace(+y, 3 * x * x);
  map_expected2.emplace(pow(+y, 3), 4 * z);
  map_expected2.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {y}, map_expected2);

  Expression::MonomialToCoeffMap map_expected3;
  map_expected3.emplace(+z, 4 * pow(+y, 3));
  map_expected3.emplace(1, 3 * x * x *y + 2);
  CheckMonomialToCoeffMap(e, {z}, map_expected3);

  Expression::MonomialToCoeffMap map_expected4;
  map_expected4.emplace(x * x * y, 3);
  map_expected4.emplace(pow(+y, 3), 4 * z);
  map_expected4.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {x, y}, map_expected4);

  Expression::MonomialToCoeffMap map_expected5;
  map_expected5.emplace(x * x, 3 * y);
  map_expected5.emplace(+z, 4 * pow(+y, 3));
  map_expected5.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {x, z}, map_expected5);

  Expression::MonomialToCoeffMap map_expected6;
  map_expected6.emplace(+y, 3 * x * x);
  map_expected6.emplace(pow(+y, 3) * z, 4);
  map_expected6.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {y, z}, map_expected6);

  Expression::MonomialToCoeffMap map_expected7;
  map_expected7.emplace(x * x * y, 3);
  map_expected7.emplace(pow(+y, 3) * z, 4);
  map_expected7.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {x, y, z}, map_expected7);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial6) {
  CheckDecomposePolynomial6(3 * x_ * x_ * y_ + 4 * pow(y_, 3) * z_ + 2, var_x_, var_y_, var_z_);

  CheckDecomposePolynomial6(y_ * (3 * x_ * x_ + 4 * y_ * y_ * z_) + 2, var_x_, var_y_, var_z_);
}

void CheckDecomposePolynomial7(const Expression& e, const Variable& x, const Variable& y) {
  EXPECT_TRUE(e.EqualTo(6 * pow(+x, 3) * y * y));

  Expression::MonomialToCoeffMap map_expected1;
  map_expected1.emplace(pow(+x, 3), 6 * y * y);
  CheckMonomialToCoeffMap(e, {x}, map_expected1);

  Expression::MonomialToCoeffMap map_expected2;
  map_expected2.emplace(pow(+y, 2), 6 * pow(+x, 3));
  CheckMonomialToCoeffMap(e, {y}, map_expected2);

  Expression::MonomialToCoeffMap map_expected3;
  map_expected3.emplace(pow(+x, 3) * pow(+y, 2), 6);
  CheckMonomialToCoeffMap(e, {x, y}, map_expected3);
}

TEST_F(SymbolicExpressionDecomposePolynomialTest, DecomposePolynomial7) {
  // Decomposes 6 * x^3 * y^2
  CheckDecomposePolynomial7(6 * pow(x_, 3) * pow(y_, 2), var_x_, var_y_);

  CheckDecomposePolynomial7(2 * pow(x_, 3) * 3 *pow(y_, 2), var_x_, var_y_);

  CheckDecomposePolynomial7(2 * pow(x_, 2) * (3 * y_ * y_ * x_), var_x_, var_y_);

  CheckDecomposePolynomial7(2 * (x_ * x_) * (3 * y_ * y_ * pow(x_, 1)), var_x_, var_y_);

  CheckDecomposePolynomial7(6 * pow(x_, 3) * pow(y_, 2) * pow(z_, 0), var_x_, var_y_);
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
