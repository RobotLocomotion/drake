#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {
using test::PolyEqual;
using test::PolyFractionEqual;

class SymbolicPolynomialFractionTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Variable var_a_{"a"};
  const Variable var_b_{"b"};
  const Variable var_c_{"c"};

  const Variables var_xy_{var_x_, var_y_};
  const Variables var_xyz_{var_x_, var_y_, var_z_};
  const Variables var_abc_{var_a_, var_b_, var_c_};

  const Polynomial polynomial_zero_{0};
  const Polynomial polynomial_one_{1};
  const Polynomial p1_{var_x_ * var_x_ * 2 + var_y_};
  const Polynomial p2_{var_x_ * var_x_ + 2 * var_y_};
  const Polynomial p3_{var_a_ * var_x_ + var_b_ + var_y_, var_xy_};
  const Polynomial p4_{2 * var_a_ * var_x_ + var_b_ + var_x_ * var_y_, var_xy_};
};

TEST_F(SymbolicPolynomialFractionTest, DefaultConstructor) {
  const PolynomialFraction p;
  EXPECT_PRED2(PolyEqual, p.numerator(), polynomial_zero_);
  EXPECT_PRED2(PolyEqual, p.denominator(), polynomial_one_);
}

TEST_F(SymbolicPolynomialFractionTest, Constructor) {
  PolynomialFraction f(polynomial_zero_, polynomial_one_);
  EXPECT_PRED2(PolyEqual, f.numerator(), polynomial_zero_);
  EXPECT_PRED2(PolyEqual, f.denominator(), polynomial_one_);
  const symbolic::Polynomial p1(var_x_ * var_a_, {var_x_});
  const symbolic::Polynomial p2(var_a_ * var_x_ * var_x_ + var_y_, var_xy_);
  PolynomialFraction f_p1_p2(p1, p2);
  EXPECT_PRED2(PolyEqual, f_p1_p2.numerator(), p1);
  EXPECT_PRED2(PolyEqual, f_p1_p2.denominator(), p2);
}

TEST_F(SymbolicPolynomialFractionTest, ConstructorWithError) {
  // Test throwing error in the constructor.
  // Denominator is 0.
  EXPECT_THROW(PolynomialFraction(polynomial_one_, polynomial_zero_),
               std::logic_error);
  // The indeterminate in the denominator is a decision variable in the
  // numerator.
  const Polynomial p1(var_x_ * var_a_, {var_x_});
  const Polynomial p2(var_x_ * var_b_ + var_y_, {var_b_, var_y_});
  EXPECT_THROW(PolynomialFraction(p2, p1), std::logic_error);
  // The indeterminate in the numerator is a decision variable in the
  // denominator.
  const symbolic::Polynomial p3(var_x_ * var_y_, {var_y_});
  EXPECT_THROW(PolynomialFraction(p1, p3), std::logic_error);
}

TEST_F(SymbolicPolynomialFractionTest, EqualTo) {
  const Polynomial p1(var_x_ * var_x_ + var_y_);
  const Polynomial p2(var_x_ + var_y_);
  const PolynomialFraction f(p1, p2);
  EXPECT_TRUE(f.EqualTo(PolynomialFraction(p1, p2)));
  EXPECT_FALSE(f.EqualTo(PolynomialFraction(2 * p1, 2 * p2)));
  EXPECT_FALSE(f.EqualTo(PolynomialFraction(p1, 2 * p2)));
  EXPECT_FALSE(f.EqualTo(PolynomialFraction(2 * p1, p2)));
}

TEST_F(SymbolicPolynomialFractionTest, UnaryMinus) {
  const Polynomial p1(var_x_ * var_x_ * 2 + var_y_);
  const Polynomial p2(var_x_ * var_x_ + 2 * var_y_);
  const PolynomialFraction f(p1, p2);
  const PolynomialFraction f_minus = -f;
  EXPECT_PRED2(PolyEqual, -f_minus.numerator(), p1);
  EXPECT_PRED2(PolyEqual, f_minus.denominator(), p2);
}

TEST_F(SymbolicPolynomialFractionTest, Addition) {
  const PolynomialFraction f1(p1_, p2_);
  const PolynomialFraction f2(p3_, p4_);
  const PolynomialFraction f1_f2_sum_expected(p1_ * p4_ + p2_ * p3_, p2_ * p4_);
  EXPECT_PRED2(PolyFractionEqual, f1 + f2, f1_f2_sum_expected);
  EXPECT_PRED2(PolyFractionEqual, f2 + f1, f1_f2_sum_expected);
  PolynomialFraction f1_f2_sum(p1_, p2_);
  f1_f2_sum += f2;
  EXPECT_PRED2(PolyFractionEqual, f1_f2_sum, f1_f2_sum_expected);

  const PolynomialFraction f1_p3_sum_expected(p1_ + p2_ * p3_, p2_);
  EXPECT_PRED2(PolyFractionEqual, f1 + p3_, f1_p3_sum_expected);
  EXPECT_PRED2(PolyFractionEqual, p3_ + f1, f1_p3_sum_expected);
  PolynomialFraction f1_p3_sum(p1_, p2_);
  f1_p3_sum += p3_;
  EXPECT_PRED2(PolyFractionEqual, f1_p3_sum, f1_p3_sum_expected);

  const double c = 2;
  const PolynomialFraction f1_c_sum_expected(p1_ + c * p2_, p2_);
  EXPECT_PRED2(PolyFractionEqual, f1 + c, f1_c_sum_expected);
  EXPECT_PRED2(PolyFractionEqual, c + f1, f1_c_sum_expected);
  PolynomialFraction f1_c_sum(p1_, p2_);
  f1_c_sum += c;
  EXPECT_PRED2(PolyFractionEqual, f1_c_sum, f1_c_sum_expected);
}

TEST_F(SymbolicPolynomialFractionTest, Subtraction) {
  const PolynomialFraction f1(p1_, p2_);
  const PolynomialFraction f2(p3_, p4_);
  const PolynomialFraction f1_minus_f2_expected(p1_ * p4_ - p2_ * p3_,
                                                p2_ * p4_);
  EXPECT_PRED2(PolyFractionEqual, f1 - f2, f1_minus_f2_expected);
  EXPECT_PRED2(PolyFractionEqual, f2 - f1, -f1_minus_f2_expected);
  PolynomialFraction f1_minus_f2 = f1;
  f1_minus_f2 -= f2;
  EXPECT_PRED2(PolyFractionEqual, f1_minus_f2, f1_minus_f2_expected);

  const PolynomialFraction f1_minus_p3_expected(p1_ - p2_ * p3_, p2_);
  EXPECT_PRED2(PolyFractionEqual, f1 - p3_, f1_minus_p3_expected);
  EXPECT_PRED2(PolyFractionEqual, p3_ - f1, -f1_minus_p3_expected);
  PolynomialFraction f1_minus_p3 = f1;
  f1_minus_p3 -= p3_;
  EXPECT_PRED2(PolyFractionEqual, f1_minus_p3, f1_minus_p3_expected);

  const double c = 2;
  const PolynomialFraction f1_minus_c_expected(p1_ - p2_ * c, p2_);
  EXPECT_PRED2(PolyFractionEqual, f1 - c, f1_minus_c_expected);
  EXPECT_PRED2(PolyFractionEqual, c - f1, -f1_minus_c_expected);
  PolynomialFraction f1_minus_c = f1;
  f1_minus_c -= c;
  EXPECT_PRED2(PolyFractionEqual, f1_minus_c, f1_minus_c_expected);
}

TEST_F(SymbolicPolynomialFractionTest, Product) {
  const PolynomialFraction f1(p1_, p2_);
  const PolynomialFraction f2(p3_, p4_);
  const PolynomialFraction f1_times_f2_expected(p1_ * p3_, p2_ * p4_);
  EXPECT_PRED2(PolyFractionEqual, f1 * f2, f1_times_f2_expected);
  EXPECT_PRED2(PolyFractionEqual, f2 * f1, f1_times_f2_expected);
  PolynomialFraction f1_times_f2 = f1;
  f1_times_f2 *= f2;
  EXPECT_PRED2(PolyFractionEqual, f1_times_f2, f1_times_f2_expected);

  const PolynomialFraction f1_times_p3_expected(p1_ * p3_, p2_);
  EXPECT_PRED2(PolyFractionEqual, f1 * p3_, f1_times_p3_expected);
  EXPECT_PRED2(PolyFractionEqual, p3_ * f1, f1_times_p3_expected);
  PolynomialFraction f1_times_p3 = f1;
  f1_times_p3 *= p3_;
  EXPECT_PRED2(PolyFractionEqual, f1_times_p3, f1_times_p3_expected);

  const double c = 2;
  const PolynomialFraction f1_times_c_expected(p1_ * c, p2_);
  EXPECT_PRED2(PolyFractionEqual, f1 * c, f1_times_c_expected);
  EXPECT_PRED2(PolyFractionEqual, c * f1, f1_times_c_expected);
  PolynomialFraction f1_times_c = f1;
  f1_times_c *= c;
  EXPECT_PRED2(PolyFractionEqual, f1_times_c, f1_times_c_expected);
}

TEST_F(SymbolicPolynomialFractionTest, Division) {
  const PolynomialFraction f1(p1_, p2_);
  const PolynomialFraction f2(p3_, p4_);
  const PolynomialFraction f1_divides_f2_expected(p3_ * p2_, p1_ * p4_);
  EXPECT_PRED2(PolyFractionEqual, f2 / f1, f1_divides_f2_expected);
  EXPECT_PRED2(PolyFractionEqual, f1 / f2, 1 / f1_divides_f2_expected);
  PolynomialFraction f1_divides_f2 = f2;
  f1_divides_f2 /= f1;
  EXPECT_PRED2(PolyFractionEqual, f1_divides_f2, f1_divides_f2_expected);

  const PolynomialFraction p3_divides_f1_expected(p1_, p2_ * p3_);
  EXPECT_PRED2(PolyFractionEqual, f1 / p3_, p3_divides_f1_expected);
  EXPECT_PRED2(PolyFractionEqual, p3_ / f1, 1 / p3_divides_f1_expected);
  PolynomialFraction p3_divides_f1 = f1;
  p3_divides_f1 /= p3_;
  EXPECT_PRED2(PolyFractionEqual, p3_divides_f1, p3_divides_f1_expected);

  const double c = 2;
  const PolynomialFraction c_divides_f1_expected(p1_, p2_ * c);
  EXPECT_PRED2(PolyFractionEqual, f1 / c, c_divides_f1_expected);
  EXPECT_PRED2(PolyFractionEqual, c / f1, 1 / c_divides_f1_expected);
  PolynomialFraction c_divides_f1 = f1;
  c_divides_f1 /= c;
  EXPECT_PRED2(PolyFractionEqual, c_divides_f1, c_divides_f1_expected);

  EXPECT_THROW(f1 / 0, std::logic_error);
  EXPECT_THROW(f1 / polynomial_zero_, std::logic_error);
  const PolynomialFraction polynomial_fraction_zero;
  EXPECT_THROW(f1 / polynomial_fraction_zero, std::logic_error);
}

TEST_F(SymbolicPolynomialFractionTest, pow) {
  const PolynomialFraction f(p1_, p2_);
  EXPECT_PRED2(PolyFractionEqual, pow(f, 0),
               PolynomialFraction(polynomial_one_, polynomial_one_));
  EXPECT_PRED2(PolyFractionEqual, pow(f, 1), f);
  EXPECT_PRED2(PolyFractionEqual, pow(f, 2),
               PolynomialFraction(pow(p1_, 2), pow(p2_, 2)));
  EXPECT_PRED2(PolyFractionEqual, pow(f, -1), PolynomialFraction(p2_, p1_));
  EXPECT_PRED2(PolyFractionEqual, pow(f, -2),
               PolynomialFraction(pow(p2_, 2), pow(p1_, 2)));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
