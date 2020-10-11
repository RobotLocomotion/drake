#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {
using test::PolyEqual;
using test::RationalFunctionEqual;

class SymbolicRationalFunctionTest : public ::testing::Test {
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
  const Polynomial p1_{var_x_ * var_x_ * 2 + var_y_, var_xy_};
  const Polynomial p2_{var_x_ * var_x_ + 2 * var_y_};
  const Polynomial p3_{var_a_ * var_x_ + var_b_ + var_y_, var_xy_};
  const Polynomial p4_{2 * var_a_ * var_x_ + var_b_ + var_x_ * var_y_, var_xy_};
  // Both var_a_ and var_x_ are indeterminates in p5, p6.
  const Polynomial p5_{var_a_ * var_x_};
  const Polynomial p6_{var_a_ * var_x_ + var_y_};

  const std::string polynomial_invariant_error_{
      "Polynomial .* does not satisfy the invariant [^]*"};
  const std::string rational_function_indeterminates_error_{
      "RationalFunction .* is invalid.[^]*"};
};

TEST_F(SymbolicRationalFunctionTest, DefaultConstructor) {
  const RationalFunction p;
  EXPECT_PRED2(PolyEqual, p.numerator(), polynomial_zero_);
  EXPECT_PRED2(PolyEqual, p.denominator(), polynomial_one_);
}

TEST_F(SymbolicRationalFunctionTest, Constructor) {
  RationalFunction f(polynomial_zero_, polynomial_one_);
  EXPECT_PRED2(PolyEqual, f.numerator(), polynomial_zero_);
  EXPECT_PRED2(PolyEqual, f.denominator(), polynomial_one_);
  const symbolic::Polynomial p1(var_x_ * var_a_, {var_x_});
  const symbolic::Polynomial p2(var_a_ * var_x_ * var_x_ + var_y_, var_xy_);
  RationalFunction f_p1_p2(p1, p2);
  EXPECT_PRED2(PolyEqual, f_p1_p2.numerator(), p1);
  EXPECT_PRED2(PolyEqual, f_p1_p2.denominator(), p2);
}

TEST_F(SymbolicRationalFunctionTest, ConstructorWithPolynomial) {
  // Constructor with numerator only.
  RationalFunction f1(p1_);
  EXPECT_PRED2(PolyEqual, f1.numerator(), p1_);
  EXPECT_PRED2(PolyEqual, f1.denominator(), polynomial_one_);
}

TEST_F(SymbolicRationalFunctionTest, ConstructorWithDouble) {
  // Constructor with a double scalar.
  const double c = 5;
  RationalFunction f1(c);
  EXPECT_PRED2(PolyEqual, f1.numerator(), c * polynomial_one_);
  EXPECT_PRED2(PolyEqual, f1.denominator(), polynomial_one_);
}

TEST_F(SymbolicRationalFunctionTest, ConstructorWithError) {
  // Test throwing error in the constructor.
  // Denominator is 0.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RationalFunction(polynomial_one_, polynomial_zero_),
      std::invalid_argument,
      "RationalFunction: the denominator should not be 0.");
  // The indeterminate in the denominator is a decision variable in the
  // numerator.
  const Polynomial p1(var_x_ * var_a_, {var_x_});
  const Polynomial p2(var_x_ * var_b_ + var_y_, {var_b_, var_y_});
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        RationalFunction(p2, p1), std::logic_error,
        "[^]* are used as decision variables in the numerator [^]*");
  }
  // The indeterminate in the numerator is a decision variable in the
  // denominator.
  const symbolic::Polynomial p3(var_x_ * var_y_, {var_y_});
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        RationalFunction(p1, p3), std::logic_error,
        "[^]* are used as indeterminates in the numerator [^]*");
  }

  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        RationalFunction(Polynomial(var_a_ * var_x_, {var_x_}),
                         Polynomial(var_a_ * var_x_, {var_a_})),
        std::logic_error,
        "[^]* are used as indeterminates in the numerator [^]* are used as "
        "decision variables in the numerator [^]*");
  }
}

TEST_F(SymbolicRationalFunctionTest, EqualTo) {
  const Polynomial p1(var_x_ * var_x_ + var_y_);
  const Polynomial p2(var_x_ + var_y_);
  const RationalFunction f(p1, p2);
  EXPECT_TRUE(f.EqualTo(RationalFunction(p1, p2)));
  EXPECT_FALSE(f.EqualTo(RationalFunction(2 * p1, 2 * p2)));
  EXPECT_FALSE(f.EqualTo(RationalFunction(p1, 2 * p2)));
  EXPECT_FALSE(f.EqualTo(RationalFunction(2 * p1, p2)));
}

TEST_F(SymbolicRationalFunctionTest, OperatorEqual) {
  EXPECT_EQ(RationalFunction(p1_, p2_), RationalFunction(p1_, p2_));
  EXPECT_EQ(RationalFunction(p1_, p2_), RationalFunction(2 * p1_, 2 * p2_));
}

TEST_F(SymbolicRationalFunctionTest, OperatorNotEqual) {
  EXPECT_NE(RationalFunction(p1_, p2_), RationalFunction(p1_ + 1, p2_));
  EXPECT_NE(RationalFunction(p1_, p2_), RationalFunction(p1_, p2_ + 1));
}

TEST_F(SymbolicRationalFunctionTest, UnaryMinus) {
  const Polynomial p1(var_x_ * var_x_ * 2 + var_y_);
  const Polynomial p2(var_x_ * var_x_ + 2 * var_y_);
  const RationalFunction f(p1, p2);
  const RationalFunction f_minus = -f;
  EXPECT_PRED2(PolyEqual, -f_minus.numerator(), p1);
  EXPECT_PRED2(PolyEqual, f_minus.denominator(), p2);
}

TEST_F(SymbolicRationalFunctionTest, Addition) {
  const RationalFunction f1(p1_, p2_);
  const RationalFunction f2(p3_, p4_);
  const RationalFunction f1_f2_sum_expected(p1_ * p4_ + p2_ * p3_, p2_ * p4_);
  EXPECT_PRED2(RationalFunctionEqual, f1 + f2, f1_f2_sum_expected);
  EXPECT_PRED2(RationalFunctionEqual, f2 + f1, f1_f2_sum_expected);
  RationalFunction f1_f2_sum(p1_, p2_);
  f1_f2_sum += f2;
  EXPECT_PRED2(RationalFunctionEqual, f1_f2_sum, f1_f2_sum_expected);
  // p5, p6 contains variable a in its indeterminates.
  const RationalFunction f3(p1_, p3_);
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 + RationalFunction(p5_, p6_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 + RationalFunction(p5_, p2_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 + RationalFunction(p2_, p5_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) + f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) + f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) + f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) += f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) += f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) += f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
  }

  const RationalFunction f1_p3_sum_expected(p1_ + p2_ * p3_, p2_);
  EXPECT_PRED2(RationalFunctionEqual, f1 + p3_, f1_p3_sum_expected);
  EXPECT_PRED2(RationalFunctionEqual, p3_ + f1, f1_p3_sum_expected);
  RationalFunction f1_p3_sum(p1_, p2_);
  f1_p3_sum += p3_;
  EXPECT_PRED2(RationalFunctionEqual, f1_p3_sum, f1_p3_sum_expected);
  // p5 contains variable a in its indeterminates.
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 + p5_, std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(p5_ + f3, std::runtime_error,
                                polynomial_invariant_error_);
  }

  const double c = 2;
  const RationalFunction f1_c_sum_expected(p1_ + c * p2_, p2_);
  EXPECT_PRED2(RationalFunctionEqual, f1 + c, f1_c_sum_expected);
  EXPECT_PRED2(RationalFunctionEqual, c + f1, f1_c_sum_expected);
  RationalFunction f1_c_sum(p1_, p2_);
  f1_c_sum += c;
  EXPECT_PRED2(RationalFunctionEqual, f1_c_sum, f1_c_sum_expected);
}

TEST_F(SymbolicRationalFunctionTest, Subtraction) {
  const RationalFunction f1(p1_, p2_);
  const RationalFunction f2(p3_, p4_);
  const RationalFunction f1_minus_f2_expected(p1_ * p4_ - p2_ * p3_, p2_ * p4_);
  EXPECT_PRED2(RationalFunctionEqual, f1 - f2, f1_minus_f2_expected);
  EXPECT_PRED2(RationalFunctionEqual, f2 - f1, -f1_minus_f2_expected);
  RationalFunction f1_minus_f2 = f1;
  f1_minus_f2 -= f2;
  EXPECT_PRED2(RationalFunctionEqual, f1_minus_f2, f1_minus_f2_expected);
  // p5, p6 contains variable a in its indeterminates.
  const RationalFunction f3(p1_, p3_);
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 - RationalFunction(p5_, p6_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 - RationalFunction(p5_, p2_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 - RationalFunction(p2_, p5_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) - f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) - f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) - f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) -= f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) -= f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) -= f3,
                                std::runtime_error,
                                polynomial_invariant_error_);
  }

  const RationalFunction f1_minus_p3_expected(p1_ - p2_ * p3_, p2_);
  EXPECT_PRED2(RationalFunctionEqual, f1 - p3_, f1_minus_p3_expected);
  EXPECT_PRED2(RationalFunctionEqual, p3_ - f1, -f1_minus_p3_expected);
  RationalFunction f1_minus_p3 = f1;
  f1_minus_p3 -= p3_;
  EXPECT_PRED2(RationalFunctionEqual, f1_minus_p3, f1_minus_p3_expected);
  // p5 contains variable a in its indeterminates.
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 - p5_, std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(p5_ - f3, std::runtime_error,
                                polynomial_invariant_error_);
  }

  const double c = 2;
  const RationalFunction f1_minus_c_expected(p1_ - p2_ * c, p2_);
  EXPECT_PRED2(RationalFunctionEqual, f1 - c, f1_minus_c_expected);
  EXPECT_PRED2(RationalFunctionEqual, c - f1, -f1_minus_c_expected);
  RationalFunction f1_minus_c = f1;
  f1_minus_c -= c;
  EXPECT_PRED2(RationalFunctionEqual, f1_minus_c, f1_minus_c_expected);
}

TEST_F(SymbolicRationalFunctionTest, Product) {
  const RationalFunction f1(p1_, p2_);
  const RationalFunction f2(p3_, p4_);
  const RationalFunction f1_times_f2_expected(p1_ * p3_, p2_ * p4_);
  EXPECT_PRED2(RationalFunctionEqual, f1 * f2, f1_times_f2_expected);
  EXPECT_PRED2(RationalFunctionEqual, f2 * f1, f1_times_f2_expected);
  RationalFunction f1_times_f2 = f1;
  f1_times_f2 *= f2;
  EXPECT_PRED2(RationalFunctionEqual, f1_times_f2, f1_times_f2_expected);
  // p5, p6 contains variable a in its indeterminates.
  const RationalFunction f3(p1_, p3_);
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 * RationalFunction(p5_, p6_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 * RationalFunction(p5_, p2_),
                                std::logic_error,
                                rational_function_indeterminates_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(f3 * RationalFunction(p2_, p5_),
                              std::runtime_error, polynomial_invariant_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) * f3,
                              std::runtime_error, polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) * f3,
                                std::logic_error,
                                rational_function_indeterminates_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) * f3,
                              std::runtime_error, polynomial_invariant_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) *= f3,
                              std::runtime_error, polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) *= f3,
                                std::logic_error,
                                rational_function_indeterminates_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) *= f3,
                              std::runtime_error, polynomial_invariant_error_);
  }

  const RationalFunction f1_times_p3_expected(p1_ * p3_, p2_);
  EXPECT_PRED2(RationalFunctionEqual, f1 * p3_, f1_times_p3_expected);
  EXPECT_PRED2(RationalFunctionEqual, p3_ * f1, f1_times_p3_expected);
  RationalFunction f1_times_p3 = f1;
  f1_times_p3 *= p3_;
  EXPECT_PRED2(RationalFunctionEqual, f1_times_p3, f1_times_p3_expected);
  // p5 contains variable a in its indeterminates.
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 * p5_, std::logic_error,
                                rational_function_indeterminates_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(p5_ * f3, std::logic_error,
                                rational_function_indeterminates_error_);
  }

  const double c = 2;
  const RationalFunction f1_times_c_expected(p1_ * c, p2_);
  EXPECT_PRED2(RationalFunctionEqual, f1 * c, f1_times_c_expected);
  EXPECT_PRED2(RationalFunctionEqual, c * f1, f1_times_c_expected);
  RationalFunction f1_times_c = f1;
  f1_times_c *= c;
  EXPECT_PRED2(RationalFunctionEqual, f1_times_c, f1_times_c_expected);
}

TEST_F(SymbolicRationalFunctionTest, Division) {
  const RationalFunction f1(p1_, p2_);
  const RationalFunction f2(p3_, p4_);
  const RationalFunction f1_divides_f2_expected(p3_ * p2_, p1_ * p4_);
  EXPECT_PRED2(RationalFunctionEqual, f2 / f1, f1_divides_f2_expected);
  EXPECT_PRED2(RationalFunctionEqual, f1 / f2, 1 / f1_divides_f2_expected);
  RationalFunction f1_divides_f2 = f2;
  f1_divides_f2 /= f1;
  EXPECT_PRED2(RationalFunctionEqual, f1_divides_f2, f1_divides_f2_expected);
  // p5, p6 contains variable a in its indeterminates.
  const RationalFunction f3(p1_, p3_);
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 / RationalFunction(p5_, p6_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 / RationalFunction(p5_, p2_),
                                std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(f3 / RationalFunction(p2_, p5_),
                                std::logic_error,
                                rational_function_indeterminates_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) / f3,
                              std::runtime_error, polynomial_invariant_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) / f3,
                              std::runtime_error, polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) / f3,
                                std::logic_error,
                                rational_function_indeterminates_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p6_) /= f3,
                              std::runtime_error, polynomial_invariant_error_);
  DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p5_, p2_) /= f3,
                              std::runtime_error, polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p2_, p5_) /= f3,
                                std::logic_error,
                                rational_function_indeterminates_error_);
  }

  const RationalFunction p3_divides_f1_expected(p1_, p2_ * p3_);
  EXPECT_PRED2(RationalFunctionEqual, f1 / p3_, p3_divides_f1_expected);
  EXPECT_PRED2(RationalFunctionEqual, p3_ / f1, 1 / p3_divides_f1_expected);
  RationalFunction p3_divides_f1 = f1;
  p3_divides_f1 /= p3_;
  EXPECT_PRED2(RationalFunctionEqual, p3_divides_f1, p3_divides_f1_expected);
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(f3 / p5_, std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(p5_ / f3, std::runtime_error,
                                polynomial_invariant_error_);
    DRAKE_EXPECT_THROWS_MESSAGE(RationalFunction(p3_, p1_) / p5_,
                                std::logic_error,
                                rational_function_indeterminates_error_);
  }

  const double c = 2;
  const RationalFunction c_divides_f1_expected(p1_, p2_ * c);
  EXPECT_PRED2(RationalFunctionEqual, f1 / c, c_divides_f1_expected);
  EXPECT_PRED2(RationalFunctionEqual, c / f1, 1 / c_divides_f1_expected);
  RationalFunction c_divides_f1 = f1;
  c_divides_f1 /= c;
  EXPECT_PRED2(RationalFunctionEqual, c_divides_f1, c_divides_f1_expected);

  const std::string zero_divider_error{
      "RationalFunction: operator/=: The divider is 0."};
  DRAKE_EXPECT_THROWS_MESSAGE(f1 / 0, std::logic_error, zero_divider_error);
  DRAKE_EXPECT_THROWS_MESSAGE(f1 / polynomial_zero_, std::logic_error,
                              zero_divider_error);
  const RationalFunction polynomial_fraction_zero;
  DRAKE_EXPECT_THROWS_MESSAGE(f1 / polynomial_fraction_zero, std::logic_error,
                              zero_divider_error);
}

TEST_F(SymbolicRationalFunctionTest, Exponentiation) {
  const RationalFunction f(p1_, p2_);
  EXPECT_PRED2(RationalFunctionEqual, pow(f, 0),
               RationalFunction(polynomial_one_, polynomial_one_));
  EXPECT_PRED2(RationalFunctionEqual, pow(f, 1), f);
  EXPECT_PRED2(RationalFunctionEqual, pow(f, 2),
               RationalFunction(pow(p1_, 2), pow(p2_, 2)));
  EXPECT_PRED2(RationalFunctionEqual, pow(f, -1), RationalFunction(p2_, p1_));
  EXPECT_PRED2(RationalFunctionEqual, pow(f, -2),
               RationalFunction(pow(p2_, 2), pow(p1_, 2)));
}

TEST_F(SymbolicRationalFunctionTest, ProductAndAddition) {
  // Test f1 * f2 + f3 * f4 where f's are all rational functions. This
  // prouduct-and-addition operation is used in matrix product.
  const RationalFunction f1(p1_, p2_);
  const RationalFunction f2(p3_, p4_);
  const RationalFunction f3(p2_, p4_);
  const RationalFunction f4(p3_, p1_);
  // (p1 / p2) * (p3 / p4) + (p2 / p4) + (p3 / p1) = (p1*p1*p3*p4 +
  // p2*p2*p3*p4)/(p1*p2*p4*p4)
  const RationalFunction result = f1 * f2 + f3 * f4;
  const RationalFunction result_expected(
      p1_ * p1_ * p3_ * p4_ + p2_ * p2_ * p3_ * p4_, p1_ * p2_ * p4_ * p4_);
  EXPECT_PRED2(test::PolyEqualAfterExpansion, result.numerator(),
               result_expected.numerator());
  EXPECT_PRED2(test::PolyEqualAfterExpansion, result.denominator(),
               result_expected.denominator());
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
