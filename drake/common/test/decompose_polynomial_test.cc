#include "drake/common/symbolic_expression.h"

#include "gtest/gtest.h"

#include "drake/common/hash.h"
#include "drake/common/monomial.h"

namespace drake {
namespace symbolic {
namespace {

// Provides common variables that are used by the following tests.
class DecomposePolynomialTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variables var_xy_{var_x_, var_y_};
  const Variables var_xyz_{var_x_, var_y_, var_z_};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

 public:
  void CheckDecomposePolynomial4(const Expression& e);
  void CheckDecomposePolynomial5(const Expression& e);
  void CheckDecomposePolynomial6(const Expression& e,
                                 bool check_expression_equal = true);
  void CheckDecomposePolynomial7(const Expression& e);
  void CheckDecomposePolynomial8(const Expression& e,
                                 bool check_expression_equal = true);
};

// Decomposes the expression `e` w.r.t `vars`, compares the decomposed map
// which maps the monomial to the coefficient, to `map_expected`.
// If `check_expression_equal` is true, checks if the summation of
// map[monomial] * monomial is equal to `e`.
// TODO(hongkai.dai) : Expression::EqualTo only checks structural equality. When
// we can compare two expressions reliably beyond structural equality, then
// always set `check_expression_equal` to true.
void CheckMonomialToCoeffMap(const symbolic::Expression& e,
                             const Variables& vars,
                             const MonomialToCoefficientMap& map_expected,
                             bool check_expression_equal = true) {
  const auto& map = DecomposePolynomial(e, vars);
  EXPECT_EQ(map.size(), map_expected.size());
  symbolic::Expression e_expected(0);
  for (const auto& p : map) {
    const auto it = map_expected.find(p.first);
    ASSERT_NE(it, map_expected.end());
    EXPECT_TRUE(p.second.EqualTo(it->second));
    e_expected += p.first * p.second;
  }
  if (check_expression_equal) {
    EXPECT_TRUE(e.EqualTo(e_expected));
  }
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial0) {
  MonomialToCoefficientMap map_expected({{x_, 1}});
  CheckMonomialToCoeffMap(x_, {var_x_}, map_expected);
  CheckMonomialToCoeffMap(x_, var_xy_, map_expected);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial1) {
  MonomialToCoefficientMap map_expected({{1, x_}});
  CheckMonomialToCoeffMap(x_, {var_y_}, map_expected);
  CheckMonomialToCoeffMap(x_, {var_y_, var_z_}, map_expected);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial2) {
  MonomialToCoefficientMap map_expected({{1, 2}});
  CheckMonomialToCoeffMap(2, {var_x_}, map_expected);
  CheckMonomialToCoeffMap(2, var_xy_, map_expected);

  map_expected.clear();
  map_expected.emplace(1, 3.14159);
  CheckMonomialToCoeffMap(3.14159, {var_x_}, map_expected);
  CheckMonomialToCoeffMap(3.14159, var_xy_, map_expected);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial3) {
  MonomialToCoefficientMap map_expected1;
  MonomialToCoefficientMap map_expected2;
  map_expected1.emplace(x_, 1);
  map_expected1.emplace(1, y_);
  map_expected2.emplace(x_, 1);
  map_expected2.emplace(y_, 1);
  CheckMonomialToCoeffMap(x_ + y_, {var_x_}, map_expected1);
  CheckMonomialToCoeffMap(x_ + y_, var_xy_, map_expected2);
  CheckMonomialToCoeffMap(x_ + y_, var_xyz_, map_expected2);

  map_expected1.clear();
  map_expected1.emplace(x_, 1);
  map_expected1.emplace(y_, 1);
  map_expected1.emplace(1, 2);
  CheckMonomialToCoeffMap(x_ + y_ + 2, var_xy_, map_expected1);
  CheckMonomialToCoeffMap(x_ + y_ + 2, var_xyz_, map_expected1);
  CheckMonomialToCoeffMap((x_ + y_) + 2, var_xy_, map_expected1);
  CheckMonomialToCoeffMap((x_ + y_) + 2, var_xyz_,
                          map_expected1);
  map_expected1[Expression(1)] = 4;
  CheckMonomialToCoeffMap((x_ + 1) + (y_ + 1) + 2, var_xy_,
                          map_expected1);
  CheckMonomialToCoeffMap((x_ + 1) + (y_ + 1) + 2, var_xyz_,
                          map_expected1);
}

void DecomposePolynomialTest::CheckDecomposePolynomial4(const Expression& e) {
  // Decomposes 2 * x * x.
  EXPECT_TRUE(e.EqualTo(2 * x_ * x_));

  MonomialToCoefficientMap map_expected1({{x_ * x_, 2}});
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1);
  CheckMonomialToCoeffMap(e, var_xy_, map_expected1);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(1, 2 * x_ * x_);
  CheckMonomialToCoeffMap(e, {var_y_}, map_expected2);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial4) {
  // Decomposes 2 * x * x.
  CheckDecomposePolynomial4(2 * x_ * x_);

  CheckDecomposePolynomial4(2 * pow(x_, 2));

  CheckDecomposePolynomial4(2 * (x_ * x_));

  CheckDecomposePolynomial4(x_ * (2 * x_));
}

void DecomposePolynomialTest::CheckDecomposePolynomial5(const Expression& e) {
  // Decomposes 6 * x * y.
  EXPECT_TRUE(e.EqualTo(6 * x_ * y_));

  MonomialToCoefficientMap map_expected1({{x_ * y_, 6}});
  CheckMonomialToCoeffMap(e, var_xy_, map_expected1);

  MonomialToCoefficientMap map_expected2({{x_, 6 * y_}});
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected2);

  MonomialToCoefficientMap map_expected3({{y_, 6 * x_}});
  CheckMonomialToCoeffMap(e, {var_y_}, map_expected3);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial5) {
  CheckDecomposePolynomial5(6 * x_ * y_);

  CheckDecomposePolynomial5(6 * (x_ * y_));

  CheckDecomposePolynomial5((2 * x_) * (3 * y_));

  CheckDecomposePolynomial5(6 * x_ * y_ * pow(z_, 0));
}

void DecomposePolynomialTest::CheckDecomposePolynomial6(
    const Expression& e, bool check_expression_equal) {
  // Decomposes 3*x*x*y + 4*y^3*z + 2.

  if (check_expression_equal) {
    Expression e_expected{3 * x_ * x_ * y_ + 4 * pow(y_, 3) * z_ + 2};
    EXPECT_TRUE(e.EqualTo(e_expected));
  }

  MonomialToCoefficientMap map_expected1(
      {{x_ * x_, 3 * y_}, {1, 4 * pow(y_, 3) * z_ + 2}});
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1, check_expression_equal);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(y_, 3 * x_ * x_);
  map_expected2.emplace(pow(y_, 3), 4 * z_);
  map_expected2.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {var_y_}, map_expected2, check_expression_equal);

  MonomialToCoefficientMap map_expected3(
      {{z_, 4 * pow(y_, 3)}, {1, 3 * x_ * x_ * y_ + 2}});
  CheckMonomialToCoeffMap(e, {var_z_}, map_expected3, check_expression_equal);

  MonomialToCoefficientMap map_expected4;
  map_expected4.emplace(x_ * x_ * y_, 3);
  map_expected4.emplace(pow(y_, 3), 4 * z_);
  map_expected4.emplace(1, 2);
  CheckMonomialToCoeffMap(e, var_xy_, map_expected4,
                          check_expression_equal);

  MonomialToCoefficientMap map_expected5;
  map_expected5.emplace(x_ * x_, 3 * y_);
  map_expected5.emplace(z_, 4 * pow(y_, 3));
  map_expected5.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {var_x_, var_z_}, map_expected5,
                          check_expression_equal);

  MonomialToCoefficientMap map_expected6;
  map_expected6.emplace(y_, 3 * x_ * x_);
  map_expected6.emplace(pow(y_, 3) * z_, 4);
  map_expected6.emplace(1, 2);
  CheckMonomialToCoeffMap(e, {var_y_, var_z_}, map_expected6,
                          check_expression_equal);

  MonomialToCoefficientMap map_expected7;
  map_expected7.emplace(x_ * x_ * y_, 3);
  map_expected7.emplace(pow(y_, 3) * z_, 4);
  map_expected7.emplace(1, 2);
  CheckMonomialToCoeffMap(e, var_xyz_, map_expected7,
                          check_expression_equal);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial6) {
  CheckDecomposePolynomial6(3 * x_ * x_ * y_ + 4 * pow(y_, 3) * z_ + 2);

  // (x * (y + z)).EqualTo(x*y + x*z) returns false because structurally they
  // are different, so ignore the expression equality check.
  CheckDecomposePolynomial6(y_ * (3 * x_ * x_ + 4 * y_ * y_ * z_) + 2, false);
}

void DecomposePolynomialTest::CheckDecomposePolynomial7(const Expression& e) {
  EXPECT_TRUE(e.EqualTo(6 * pow(x_, 3) * y_ * y_));

  MonomialToCoefficientMap map_expected1({{pow(x_, 3), 6 * y_ * y_}});
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1);

  MonomialToCoefficientMap map_expected2({{pow(y_, 2), 6 * pow(x_, 3)}});
  CheckMonomialToCoeffMap(e, {var_y_}, map_expected2);

  MonomialToCoefficientMap map_expected3({{pow(x_, 3) * pow(y_, 2), 6}});
  CheckMonomialToCoeffMap(e, var_xy_, map_expected3);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial7) {
  // Decomposes 6 * x^3 * y^2.
  CheckDecomposePolynomial7(6 * pow(x_, 3) * pow(y_, 2));

  CheckDecomposePolynomial7(2 * pow(x_, 3) * 3 * pow(y_, 2));

  CheckDecomposePolynomial7(2 * pow(x_, 2) * (3 * y_ * y_ * x_));

  CheckDecomposePolynomial7(2 * (x_ * x_) * (3 * y_ * y_ * pow(x_, 1)));

  CheckDecomposePolynomial7(6 * pow(x_, 3) * pow(y_, 2) * pow(z_, 0));
}

void DecomposePolynomialTest::CheckDecomposePolynomial8(
    const Expression& e, bool check_expression_equal) {
  // Decomposes x^3 - 4*x*y^2 + 2*x^2*y - 8*y^3.
  if (check_expression_equal) {
    EXPECT_TRUE(e.EqualTo(pow(x_, 3) - 4 * x_ * y_ * y_ + 2 * x_ * x_ * y_ -
                          8 * pow(y_, 3)));
  }

  MonomialToCoefficientMap map_expected1;
  map_expected1.emplace(pow(x_, 3), 1);
  map_expected1.emplace(pow(x_, 2), 2 * y_);
  map_expected1.emplace(x_, -4 * y_ * y_);
  map_expected1.emplace(1, -8 * pow(y_, 3));
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1, check_expression_equal);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(pow(y_, 3), -8);
  map_expected2.emplace(y_ * y_, -4 * x_);
  map_expected2.emplace(y_, 2 * x_ * x_);
  map_expected2.emplace(1, pow(x_, 3));
  CheckMonomialToCoeffMap(e, {var_y_}, map_expected2, check_expression_equal);

  MonomialToCoefficientMap map_expected3;
  map_expected3.emplace(pow(x_, 3), 1);
  map_expected3.emplace(x_ * y_ * y_, -4);
  map_expected3.emplace(x_ * x_ * y_, 2);
  map_expected3.emplace(pow(y_, 3), -8);
  CheckMonomialToCoeffMap(e, var_xy_, map_expected3,
                          check_expression_equal);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial8) {
  CheckDecomposePolynomial8(pow(x_, 3) - 4 * x_ * y_ * y_ + 2 * x_ * x_ * y_ -
                            8 * pow(y_, 3));

  CheckDecomposePolynomial8(pow(x_ + 2 * y_, 2) * (x_ - 2 * y_), false);

  CheckDecomposePolynomial8((x_ + 2 * y_) * (x_ * x_ - 4 * y_ * y_), false);

  CheckDecomposePolynomial8(
      (x_ * x_ + 4 * x_ * y_ + 4 * y_ * y_) * (x_ - 2 * y_), false);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial9) {
  // Decomposes -x.
  MonomialToCoefficientMap map_expected1({{x_, -1}});
  CheckMonomialToCoeffMap(-x_, {var_x_}, map_expected1);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(1, -x_);
  CheckMonomialToCoeffMap(-x_, {var_y_}, map_expected2);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial10) {
  // Decomposes (x + y + 1)^4.
  Expression e = pow(x_ + y_ + 1, 4);
  MonomialToCoefficientMap map_expected1;
  map_expected1.emplace(pow(x_, 4), 1);
  map_expected1.emplace(pow(x_, 3), 4 * (y_ + 1));
  map_expected1.emplace(pow(x_, 2), 6 * pow(y_ + 1, 2));
  map_expected1.emplace(x_, 4 * pow(y_ + 1, 3));
  map_expected1.emplace(1, pow(y_ + 1, 4));
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1, false);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(pow(x_, 4), 1);
  map_expected2.emplace(pow(x_, 3) * y_, 4);
  map_expected2.emplace(pow(x_, 2) * pow(y_, 2), 6);
  map_expected2.emplace(x_ * pow(y_, 3), 4);
  map_expected2.emplace(pow(y_, 4), 1);
  map_expected2.emplace(pow(x_, 3), 4);
  map_expected2.emplace(pow(x_, 2) * y_, 12);
  map_expected2.emplace(x_ * pow(y_, 2), 12);
  map_expected2.emplace(pow(y_, 3), 4);
  map_expected2.emplace(pow(x_, 2), 6);
  map_expected2.emplace(x_ * y_, 12);
  map_expected2.emplace(pow(y_, 2), 6);
  map_expected2.emplace(x_, 4);
  map_expected2.emplace(y_, 4);
  map_expected2.emplace(1, 1);
  CheckMonomialToCoeffMap(e, var_xy_, map_expected2, false);
}

TEST_F(DecomposePolynomialTest, DecomposePolynomial11) {
  // Decomposes (x + y + 1)^3.
  Expression e = pow(x_ + y_ + 1, 3);
  MonomialToCoefficientMap map_expected1;
  map_expected1.emplace(pow(x_, 3), 1);
  // TODO(hongkai.dai): change this to 3 * (y_ + 1) when Expand() method
  // is available.
  map_expected1.emplace(pow(x_, 2), 1 + y_ + 2 * (y_ + 1));
  map_expected1.emplace(x_, 3 * pow(y_ + 1, 2));
  map_expected1.emplace(1, pow(y_ + 1, 3));
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1, false);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(pow(x_, 3), 1);
  map_expected2.emplace(pow(x_, 2) * y_, 3);
  map_expected2.emplace(x_ * pow(y_, 2), 3);
  map_expected2.emplace(pow(y_, 3), 1);
  map_expected2.emplace(pow(x_, 2), 3);
  map_expected2.emplace(x_ * y_, 6);
  map_expected2.emplace(pow(y_, 2), 3);
  map_expected2.emplace(x_, 3);
  map_expected2.emplace(y_, 3);
  map_expected2.emplace(1, 1);
  CheckMonomialToCoeffMap(e, var_xy_, map_expected2, false);
}
// TODO(hongkai.dai): enable the following tests, when ((x^2 * y +
// x^3)/(x^2)).is_polynomial() returns true.


TEST_F(DecomposePolynomialTest, DISABLED_DecomposePolynomial12) {
  // Decomposes x^2 * y / (x * y)
  MonomialToCoefficientMap map_expected1;
  map_expected1.emplace(x_, 1);
  CheckMonomialToCoeffMap((x_ * x_ * y_) / (x_ * y_), {var_x_}, map_expected1,
false);

  MonomialToCoefficientMap map_expected2;
  // TODO(hongkai.dai) : Currently (pow(x_, 2) / x_).EqualTo(x_) returns false,
  // revisit this code when we can simplify the division result.
  map_expected2.emplace(1, pow(x_, 2) / x_);
  CheckMonomialToCoeffMap((x_ * x_ * y_) / (x_ * y_), {var_y_}, map_expected2,
false);

  CheckMonomialToCoeffMap((x_ * x_ * y_) / (x_ * y_), var_xy_,
map_expected1, false);
}

TEST_F(DecomposePolynomialTest, DISABLED_DecomposePolynomial13) {
  // Decomposes (3 * x^2 * y^3 + 2 * x^3 * y^2) / (3 * x * y)
  Expression e = (3 * x_ * x_ * pow(y_, 3) + 2 * pow(x_, 3) * y_ * y_) / (3 * x_
* y_);

  MonomialToCoefficientMap map_expected1;
  map_expected1.emplace(x_, 3 * pow(y_, 3) / (3 * y_));
  map_expected1.emplace(x_ * x_, (2 * y_ * y_) / (3 * y_));
  CheckMonomialToCoeffMap(e, {var_x_}, map_expected1, false);

  MonomialToCoefficientMap map_expected2;
  map_expected2.emplace(y_ * y_, (3 * x_ * x_) / (3 * x_));
  map_expected2.emplace(y_, (2 * pow(x_, 3))/(3 * x_));
  CheckMonomialToCoeffMap(e, {var_y_}, map_expected2, false);

  MonomialToCoefficientMap map_expected3;
  map_expected3.emplace(x_ * y_ * y_, 1);
  map_expected3.emplace(x_ * x_ * y_, 2.0 / 3);
  CheckMonomialToCoeffMap(e, var_xy_, map_expected3, false);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
