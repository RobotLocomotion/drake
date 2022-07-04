#include "drake/multibody/dev/c_iris/rational_forward_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace multibody {
namespace c_iris {
void CheckReplaceCosAndSinWithRationalFunction(
    const symbolic::Expression& e, const VectorX<symbolic::Variable>& cos_vars,
    const VectorX<symbolic::Variable>& sin_vars,
    const VectorX<symbolic::Variable>& t_angle,
    const symbolic::RationalFunction& e_rational_expected) {
  VectorX<symbolic::Variable> cos_sin_vars(cos_vars.rows() + sin_vars.rows());
  cos_sin_vars << cos_vars, sin_vars;
  const symbolic::Variables cos_sin_vars_variables(cos_sin_vars);
  const symbolic::Polynomial e_poly(e, cos_sin_vars_variables);
  symbolic::RationalFunction e_rational;
  ReplaceCosAndSinWithRationalFunction(e_poly, cos_vars, sin_vars, t_angle,
                                       &e_rational);
  EXPECT_PRED2(symbolic::test::PolyEqualAfterExpansion, e_rational.numerator(),
               e_rational_expected.numerator());
  EXPECT_PRED2(symbolic::test::PolyEqualAfterExpansion,
               e_rational.denominator(), e_rational_expected.denominator());
}

GTEST_TEST(RationalForwardKinematics, ReplaceCosAndSinWithRationalFunction) {
  VectorX<symbolic::Variable> cos_vars(3);
  VectorX<symbolic::Variable> sin_vars(3);
  VectorX<symbolic::Variable> t_angle(3);
  for (int i = 0; i < 3; ++i) {
    cos_vars(i) = symbolic::Variable("cos(delta_q(" + std::to_string(i) + "))");
    sin_vars(i) = symbolic::Variable("sin(delta_q(" + std::to_string(i) + "))");
    t_angle(i) = symbolic::Variable("t_angle(" + std::to_string(i) + ")");
  }

  symbolic::Variable a("a");
  symbolic::Variable b("b");

  symbolic::Variables t(t_angle);

  // test cos(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      cos_vars(0), cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(1 - t_angle(0) * t_angle(0)),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0))));
  // test sin(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      sin_vars(0), cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(2 * t_angle(0)),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0))));
  // test 1.
  CheckReplaceCosAndSinWithRationalFunction(1, cos_vars, sin_vars, t_angle,
                                            symbolic::RationalFunction(1));

  // test a + b
  CheckReplaceCosAndSinWithRationalFunction(
      a + b, cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(symbolic::Polynomial(a + b, t)));

  // test 1 + cos(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      1 + cos_vars(0), cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(2),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0))));

  // test a + b*cos(delta_q(0)) + sin(delta_q(1))
  CheckReplaceCosAndSinWithRationalFunction(
      a + b * cos_vars(0) + sin_vars(1), cos_vars, sin_vars, t_angle,
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

  // test a + b * cos(delta_q(0) * sin(delta_q(1)) + sin(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      a + b * cos_vars(0) * sin_vars(1) + sin_vars(0), cos_vars, sin_vars,
      t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              a * (1 + t_angle(0) * t_angle(0)) *
                      (1 + t_angle(1) * t_angle(1)) +
                  b * (1 - t_angle(0) * t_angle(0)) * 2 * t_angle(1) +
                  2 * t_angle(0) * (1 + t_angle(1) * t_angle(1)),
              t),
          symbolic::Polynomial((1 + t_angle(0) * t_angle(0)) *
                               (1 + t_angle(1) * t_angle(1)))));

  // test a + b * cos(delta_q(0)) * sin(delta_q(1)) + sin(delta_q(0)) *
  // cos(delta_q(2))
  CheckReplaceCosAndSinWithRationalFunction(
      a + b * cos_vars(0) * sin_vars(1) + sin_vars(0) * cos_vars(2), cos_vars,
      sin_vars, t_angle,
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
  CheckReplaceCosAndSinWithRationalFunction(
      t_angle(0), cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(symbolic::Polynomial(t_angle(0), t)));

  // test t_angle(0) * cos(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      t_angle(0) * cos_vars(0), cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(
              t_angle(0) - t_angle(0) * t_angle(0) * t_angle(0), t),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0), t)));

  // test t_angle(0) * sin(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      t_angle(0) * sin_vars(0), cos_vars, sin_vars, t_angle,
      symbolic::RationalFunction(
          symbolic::Polynomial(2 * t_angle(0) * t_angle(0), t),
          symbolic::Polynomial(1 + t_angle(0) * t_angle(0), t)));

  // test (t_angle(0) * a + t_angle(1) * b) * sin(delta_q(0)) * cos(delta_q(1))
  // + 2 * t_angle(0) * b
  CheckReplaceCosAndSinWithRationalFunction(
      (t_angle(0) * a + t_angle(1) * b) * sin_vars(0) * cos_vars(1) +
          2 * t_angle(0) * b,
      cos_vars, sin_vars, t_angle,
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

}  // namespace c_iris
}  // namespace multibody
}  // namespace drake
