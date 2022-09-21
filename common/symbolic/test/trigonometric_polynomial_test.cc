#include "drake/common/symbolic/trigonometric_polynomial.h"

#include <iostream>
#include <sstream>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

GTEST_TEST(SymbolicLatex, BasicTest) {
  Variable x{"x"}, y{"y"}, z{"z"};
  Variable sx{"sx"}, sy{"sy"}, cx{"cx"}, cy{"cy"};
  SinCosSubstitution subs;
  subs.emplace(x, SinCos(sx, cx));
  subs.emplace(y, SinCos(sy, cy));

  // Expressions.
  EXPECT_PRED2(ExprEqual, Substitute(x, subs), x);
  EXPECT_PRED2(ExprEqual, Substitute(sin(x), subs), sx);
  EXPECT_PRED2(ExprEqual, Substitute(cos(x), subs), cx);
  // Addition.
  EXPECT_PRED2(ExprEqual, Substitute(sin(x + y), subs), sx * cy + cx * sy);
  EXPECT_PRED2(ExprEqual, Substitute(sin(x + z), subs),
               sx * cos(z) + cx * sin(z));
  EXPECT_PRED2(ExprEqual, Substitute(sin(z + x), subs),
               sx * cos(z) + cx * sin(z));
  EXPECT_PRED2(ExprEqual, Substitute(sin(x + 0.15), subs),
               sx * cos(0.15) + cx * sin(0.15));
  EXPECT_PRED2(ExprEqual, Substitute(cos(x + y), subs), cx * cy - sx * sy);
  EXPECT_PRED2(ExprEqual, Substitute(cos(x + z), subs),
               cx * cos(z) - sx * sin(z));
  EXPECT_PRED2(ExprEqual, Substitute(cos(z + x), subs),
               cx * cos(z) - sx * sin(z));
  EXPECT_PRED2(ExprEqual, Substitute(cos(x + 0.15), subs),
               cx * cos(0.15) - sx * sin(0.15));
  EXPECT_PRED2(ExprEqual, Substitute(2 + 3 * x + 4 * y, subs),
               2 + 3 * x + 4 * y);
  // Multiplication.
  EXPECT_PRED2(ExprEqual, Substitute(sin(2 * x), subs), 2 * sx * cx);
  EXPECT_PRED2(ExprEqual, Substitute(sin(-2 * x), subs), -2 * sx * cx);
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(2.4 * x), subs),
                              ".*only support sin.* where c is an integer.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(x * x), subs),
                              ".*'status == kNotSinCos' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(x * y), subs),
                              ".*only support sin.* where c is an integer.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(x * z), subs),
                              ".*only support sin.* where c is an integer.*");
  EXPECT_PRED2(ExprEqual, Substitute(cos(2 * x), subs), cx * cx - sx * sx);
  EXPECT_PRED2(ExprEqual, Substitute(cos(-2 * x), subs), cx * cx - sx * sx);
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(2.4 * x), subs),
                              ".*only support cos.* where c is an integer.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(x * x), subs),
                              ".*'status == kNotSinCos' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(x * y), subs),
                              ".*only support cos.* where c is an integer.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(x * z), subs),
                              ".*only support cos.* where c is an integer.*");
  EXPECT_PRED2(ExprEqual, Substitute(x * sin(x) * cos(x), subs), x * sx * cx);
  EXPECT_PRED2(ExprEqual, Substitute(sin(2 * x + y), subs),
               2 * sx * cx * cy + (cx * cx - sx * sx) * sy);
  // Other Expressions.
  const auto TestUnary =
      [&x, &sx, &cx,
       &subs](const std::function<Expression(const Expression&)>& pred) {
        EXPECT_PRED2(ExprEqual, Substitute(pred(x), subs), pred(x));
        EXPECT_PRED2(ExprEqual, Substitute(pred(sin(x)), subs), pred(sx));
        EXPECT_PRED2(ExprEqual, Substitute(pred(cos(x)), subs), pred(cx));
        DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(pred(x)), subs),
                                    ".*'status == kNotSinCos' failed.*");
        DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(pred(x)), subs),
                                    ".*'status == kNotSinCos' failed.*");
      };
  const auto TestBinary =
      [&x, &y, &sx, &cx, &sy, &cy, &subs](
          const std::function<Expression(const Expression&, const Expression&)>&
              pred) {
        EXPECT_PRED2(ExprEqual, Substitute(pred(x, y), subs), pred(x, y));
        EXPECT_PRED2(ExprEqual, Substitute(pred(sin(x), y), subs), pred(sx, y));
        EXPECT_PRED2(ExprEqual, Substitute(pred(cos(x), y), subs), pred(cx, y));
        EXPECT_PRED2(ExprEqual, Substitute(pred(x, sin(y)), subs), pred(x, sy));
        EXPECT_PRED2(ExprEqual, Substitute(pred(x, cos(y)), subs), pred(x, cy));
        DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(pred(x, y)), subs),
                                    ".*'status == kNotSinCos' failed.*");
        DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(pred(x, y)), subs),
                                    ".*'status == kNotSinCos' failed.*");
      };

  const auto mypow = [](const Expression& e1, const Expression e2) {
    return pow(e1, e2);
  };
  TestBinary(mypow);  // TestBinary(pow) doesn't compile for some reason.
  EXPECT_PRED2(ExprEqual, Substitute(x / y, subs), x / y);
  EXPECT_PRED2(ExprEqual, Substitute(sin(x) / sin(y), subs), sx / sy);
  EXPECT_PRED2(ExprEqual, Substitute(cos(x) / cos(y), subs), cx / cy);
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(x / y), subs),
                              ".*'status == kNotSinCos' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(x / y), subs),
                              ".*'status == kNotSinCos' failed.*");
  TestUnary(abs);
  TestUnary(log);
  TestUnary(exp);
  TestUnary(sqrt);
  EXPECT_PRED2(ExprEqual, Substitute(tan(x), subs), sx / cx);
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(tan(x)), subs),
                              ".*'status == kNotSinCos' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(tan(x)), subs),
                              ".*'status == kNotSinCos' failed.*");
  TestUnary(asin);
  TestUnary(acos);
  TestUnary(atan);
  TestBinary(atan2);
  TestUnary(sinh);
  TestUnary(cosh);
  TestUnary(tanh);
  TestBinary(min);
  TestBinary(max);
  TestUnary(ceil);
  TestUnary(floor);
  EXPECT_PRED2(ExprEqual, Substitute(if_then_else(z > 1, z, 1), subs),
               if_then_else(z > 1, z, 1));
  DRAKE_EXPECT_THROWS_MESSAGE(
      Substitute(if_then_else(x > y, x, y), subs),
      "Substituting sin/cos into formulas is not supported yet");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(if_then_else(x > y, x, y)), subs),
                              ".*'status == kNotSinCos' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(if_then_else(x > y, x, y)), subs),
                              ".*'status == kNotSinCos' failed.*");

  // Limited half-angle support.
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(sin(0.5*x), subs),
                              ".*only support sin.* where c is an integer.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Substitute(cos(0.5*x), subs),
                              ".*only support cos.* where c is an integer.*");

  // Matrix<Expression>
  Eigen::Matrix<Expression, 2, 2> m;
  m << x, sin(x), sin(y), sin(x + y);
  const Eigen::Matrix<Expression, 2, 2> m2 = Substitute(m, subs);
  EXPECT_PRED2(ExprEqual, m2(0, 0), x);
  EXPECT_PRED2(ExprEqual, m2(0, 1), sx);
  EXPECT_PRED2(ExprEqual, m2(1, 0), sy);
  EXPECT_PRED2(ExprEqual, m2(1, 1), sx * cy + sy * cx);
}

GTEST_TEST(SymbolicLatex, HalfAngleTest) {
  Variable x{"x"}, y{"y"}, z{"z"};
  Variable sx{"sx"}, sy{"sy"}, cx{"cx"}, cy{"cy"};
  SinCosSubstitution subs;
  subs.emplace(x, SinCos(sx, cx, SinCosSubstitutionType::kHalfAnglePreferSin));
  subs.emplace(y, SinCos(sy, cy, SinCosSubstitutionType::kHalfAnglePreferCos));

  EXPECT_PRED2(ExprEqual, Substitute(sin(x), subs), 2 * sx * cx);
  EXPECT_PRED2(ExprEqual, Substitute(sin(1 * x), subs), 2 * sx * cx);
  EXPECT_PRED2(ExprEqual, Substitute(sin(2 * x), subs),
               4 * sx * cx * (1 - 2 * sx * sx));
  EXPECT_PRED2(ExprEqual, Substitute(cos(x), subs), 1- 2 * sx * sx);
  EXPECT_PRED2(ExprEqual, Substitute(sin(y), subs), 2 * sy * cy);
  EXPECT_PRED2(ExprEqual, Substitute(cos(y), subs), 2 * cy * cy - 1);
  EXPECT_PRED2(ExprEqual, Substitute(sin(0.5*x), subs), sx);
  EXPECT_PRED2(ExprEqual, Substitute(cos(0.5*x), subs), cx);
  EXPECT_PRED2(ExprEqual, Substitute(sin(-0.5*x), subs), -sx);
  EXPECT_PRED2(ExprEqual, Substitute(cos(-0.5*x), subs), cx);
  // Note: We don't support division yet, but calling Expand makes this work:
  EXPECT_PRED2(ExprEqual, Substitute(sin(x / 2.0).Expand(), subs), sx);

  // Matrix half-angle.
  Eigen::Matrix<Expression, 2, 2> m;
  m << x, sin(x), sin(y), sin(x + y);
  const Eigen::Matrix<Expression, 2, 2> m2 = Substitute(m, subs);
  EXPECT_PRED2(ExprEqual, m2(0, 0), x);
  EXPECT_PRED2(ExprEqual, m2(0, 1), 2 * sx * cx);
  EXPECT_PRED2(ExprEqual, m2(1, 0), 2 * sy * cy);
  EXPECT_PRED2(
      ExprEqual, m2(1, 1),
      (2 * sx * cx) * (2 * cy * cy - 1) + (2 * sy * cy) * (1 - 2 * sx * sx));
}

void CheckSubstituteStereographicProjection(
    const symbolic::Expression& e,
    const std::unordered_map<symbolic::Variable, symbolic::Variable>& subs,
    const symbolic::Polynomial& e_poly, const std::vector<SinCos>& sin_cos,
    const VectorX<symbolic::Variable>& t_angle,
    const symbolic::RationalFunction& e_rational_expected) {
  // Test the overloaded version that takes e as a symbolic::Expression.
  const symbolic::RationalFunction e_rational1 =
      SubstituteStereographicProjection(e, subs);
  EXPECT_PRED2(symbolic::test::PolyEqualAfterExpansion, e_rational1.numerator(),
               e_rational_expected.numerator());
  EXPECT_PRED2(symbolic::test::PolyEqualAfterExpansion,
               e_rational1.denominator(), e_rational_expected.denominator());
  // Test the overloaded version that takes e as a symbolic Polynomial.
  const symbolic::RationalFunction e_rational2 =
      SubstituteStereographicProjection(e_poly, sin_cos, t_angle);
  EXPECT_PRED2(symbolic::test::PolyEqualAfterExpansion, e_rational2.numerator(),
               e_rational_expected.numerator());
  EXPECT_PRED2(symbolic::test::PolyEqualAfterExpansion,
               e_rational2.denominator(), e_rational_expected.denominator());
}

GTEST_TEST(RationalForwardKinematics, ReplaceCosAndSinWithRationalFunction) {
  VectorX<symbolic::Variable> cos_vars(3);
  VectorX<symbolic::Variable> sin_vars(3);
  VectorX<symbolic::Variable> t_angle(3);
  VectorX<symbolic::Variable> theta(3);
  std::vector<SinCos> sin_cos;
  for (int i = 0; i < 3; ++i) {
    theta(i) = symbolic::Variable("theta" + std::to_string(i));
    cos_vars(i) = symbolic::Variable("cos(theta(" + std::to_string(i) + "))");
    sin_vars(i) = symbolic::Variable("sin(theta(" + std::to_string(i) + "))");
    t_angle(i) = symbolic::Variable("t_angle(" + std::to_string(i) + ")");
    sin_cos.emplace_back(sin_vars(i), cos_vars(i));
  }
  symbolic::Variables cos_sin_vars;
  cos_sin_vars.insert(symbolic::Variables(cos_vars));
  cos_sin_vars.insert(symbolic::Variables(sin_vars));

  symbolic::Variable a("a");
  symbolic::Variable b("b");

  symbolic::Variables t(t_angle);

  std::unordered_map<symbolic::Variable, symbolic::Variable> subs;
  for (int i = 0; i < 3; ++i) {
    subs.emplace(theta(i), t_angle(i));
  }

  // test cos(theta(0))
  CheckSubstituteStereographicProjection(
      cos(theta(0)), subs, symbolic::Polynomial(cos_vars(0)), sin_cos, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(1 - t_angle(0) * t_angle(0)),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0))));
  // test sin(theta(0))
  CheckSubstituteStereographicProjection(
      sin(theta(0)), subs, symbolic::Polynomial(sin_vars(0)), sin_cos, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(2 * t_angle(0)),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0))));
  // test 1.
  CheckSubstituteStereographicProjection(1, subs, symbolic::Polynomial(1),
                                         sin_cos, t_angle,
                                         symbolic::RationalFunction(1));

  // test a + b
  CheckSubstituteStereographicProjection(
      a + b, subs, symbolic::Polynomial({{symbolic::Monomial(), a + b}}),
      sin_cos, t_angle,
      symbolic::RationalFunction(symbolic::Polynomial(
          a + b,
          t) /* A constant polynomial with a+b being the constant term */));

  // test 1 + cos(theta(0))
  CheckSubstituteStereographicProjection(
      1 + cos(theta(0)), subs, symbolic::Polynomial(1 + cos_vars(0)), sin_cos,
      t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(2),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0))));

  // test a + b*cos(delta_q(0)) + sin(delta_q(1))
  CheckSubstituteStereographicProjection(
      a + b * cos(theta(0)) + sin(theta(1)), subs,
      symbolic::Polynomial(a + b * cos_vars(0) + sin_vars(1), cos_sin_vars),
      sin_cos, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              a * (1 + t_angle(0) * t_angle(0)) *
                      (1 + t_angle(1) * t_angle(1)) +
                  b * (1 - t_angle(0) * t_angle(0)) *
                      (1 + t_angle(1) * t_angle(1)) +
                  2 * t_angle(1) * (1 + t_angle(0) * t_angle(0)),
              t),
          symbolic::Polynomial((1 + t_angle(0) * t_angle(0)) *
                               (1 + t_angle(1) * t_angle(1)))));

  // test a + b * cos(theta(0) * sin(theta(1)) + sin(theta(0))
  CheckSubstituteStereographicProjection(
      a + b * cos(theta(0)) * sin(theta(1)) + sin(theta(0)), subs,
      symbolic::Polynomial(a + b * cos_vars(0) * sin_vars(1) + sin_vars(0),
                           cos_sin_vars),
      sin_cos, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              a * (1 + t_angle(0) * t_angle(0)) *
                      (1 + t_angle(1) * t_angle(1)) +
                  b * (1 - t_angle(0) * t_angle(0)) * 2 * t_angle(1) +
                  2 * t_angle(0) * (1 + t_angle(1) * t_angle(1)),
              t),
          symbolic::Polynomial((1 + t_angle(0) * t_angle(0)) *
                               (1 + t_angle(1) * t_angle(1)))));

  // test a + b * cos(theta(0)) * sin(theta(1)) + sin(theta(0)) * cos(theta(2))
  CheckSubstituteStereographicProjection(
      a + b * cos(theta(0)) * sin(theta(1)) + sin(theta(0)) * cos(theta(2)),
      subs,
      symbolic::Polynomial(
          a + b * cos_vars(0) * sin_vars(1) + sin_vars(0) * cos_vars(2),
          cos_sin_vars),
      sin_cos, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              a * (1 + t_angle(0) * t_angle(0)) *
                      (1 + t_angle(1) * t_angle(1)) *
                      (1 + t_angle(2) * t_angle(2)) +
                  b * (1 - t_angle(0) * t_angle(0)) * 2 * t_angle(1) *
                      (1 + t_angle(2) * t_angle(2)) +
                  2 * t_angle(0) * (1 + t_angle(1) * t_angle(1)) *
                      (1 - t_angle(2) * t_angle(2)),
              t),
          symbolic::Polynomial((1 + t_angle(0) * t_angle(0)) *
                               (1 + t_angle(1) * t_angle(1)) *
                               (1 + t_angle(2) * t_angle(2)))));

  // test t_angle(0)
  CheckSubstituteStereographicProjection(
      t_angle(0), subs,
      symbolic::Polynomial({{symbolic::Monomial(), t_angle(0)}}), sin_cos,
      t_angle, symbolic::RationalFunction(symbolic::Polynomial(t_angle(0), t)));

  // test t_angle(0) * cos(theta(0))
  CheckSubstituteStereographicProjection(
      t_angle(0) * cos(theta(0)), subs,
      symbolic::Polynomial(t_angle(0) * cos_vars(0), cos_sin_vars), sin_cos,
      t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              t_angle(0) - t_angle(0) * t_angle(0) * t_angle(0), t),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0), t)));

  // test t_angle(0) * sin(theta(0))
  CheckSubstituteStereographicProjection(
      t_angle(0) * sin(theta(0)), subs,
      symbolic::Polynomial(t_angle(0) * sin_vars(0), cos_sin_vars), sin_cos,
      t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(2 * t_angle(0) * t_angle(0), t),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0), t)));

  // test (t_angle(0) * a + t_angle(1) * b) * sin(delta_q(0)) * cos(delta_q(1))
  // + 2 * t_angle(0) * b
  CheckSubstituteStereographicProjection(
      (t_angle(0) * a + t_angle(1) * b) * sin(theta(0)) * cos(theta(1)) +
          2 * t_angle(0) * b,
      subs,
      symbolic::Polynomial(
          (t_angle(0) * a + t_angle(1) * b) * sin_vars(0) * cos_vars(1) +
              2 * t_angle(0) * b,
          cos_sin_vars),
      sin_cos, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              (a * t_angle(0) + b * t_angle(1)) * 2 * t_angle(0) *
                      (1 - t_angle(1) * t_angle(1)) +
                  2 * t_angle(0) * b * (1 + t_angle(0) * t_angle(0)) *
                      (1 + t_angle(1) * t_angle(1)),
              t),
          symbolic::Polynomial(
              (1 + t_angle(0) * t_angle(0)) * (1 + t_angle(1) * t_angle(1)),
              t)));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
