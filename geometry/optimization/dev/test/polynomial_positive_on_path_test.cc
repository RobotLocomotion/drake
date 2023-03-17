#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace geometry {
namespace optimization {

using solvers::MathematicalProgram;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

namespace {

// Check everything besides the expression for p_ in poly_prog.
void validate_parametrized_polynomial_prog(
    const ParametrizedPolynomialPositiveOnUnitInterval& poly_prog,
    const Polynomial& poly, const int num_psd, const int num_bounding_box = 0) {
  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};

  // p should have the same degree as poly
  EXPECT_EQ(p.TotalDegree(), poly.TotalDegree());
  // p should have the same indeterminates as poly.
  EXPECT_TRUE(p.indeterminates() == poly.indeterminates());
  for (const auto& var : p.indeterminates()) {
    if (!var.equal_to(poly_prog.get_mu())) {
      EXPECT_EQ(p.Degree(var), 2);
      EXPECT_EQ(lambda.Degree(var), 2);
            EXPECT_TRUE(nu.TotalDegree() == 0 || nu.Degree(var) ==  2);
    }
  }

  const MathematicalProgram& psatz_variables_and_constraints{
      poly_prog.get_psatz_variables_and_psd_constraints()};

  for (const auto& var : poly.indeterminates()) {
    unused(var);
    // All the indeterminates of poly should be in the program.
    EXPECT_NO_THROW(
        unused(psatz_variables_and_constraints.FindIndeterminateIndex(var)));
  }
  for (const auto& var : p.decision_variables()) {
    if (lambda.decision_variables().include(var) ||
        nu.decision_variables().include(var)) {
      // Only the decision variables of lambda and nu should be in the program
      // for now.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints.FindDecisionVariableIndex(var)));
    } else {
      // All other decision variables of p_ should not be in
      // psatz_variables_and_constraints.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints.FindDecisionVariableIndex(var)),
          std::runtime_error);
    }
  }

  EXPECT_EQ(psatz_variables_and_constraints.positive_semidefinite_constraints()
                .size(),
            num_psd);
  EXPECT_EQ(psatz_variables_and_constraints.bounding_box_constraints().size(),
            num_bounding_box);
  EXPECT_EQ(psatz_variables_and_constraints.GetAllConstraints().size(),
            num_psd + num_bounding_box);
}

}  // namespace

GTEST_TEST(ParametrizedPolyPositive, CtorTotalDegreeZero) {
  const Variable coeff{"coeff"};
  const Variables parameters{coeff};
  // A polynomial with no indeterminates.
  const Polynomial poly{coeff * (*parameters.begin()), Variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{poly, Variable(),
                                                               parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};

  EXPECT_TRUE(p.EqualTo(poly - lambda));
  EXPECT_TRUE(nu.EqualTo(Polynomial(0)));

  validate_parametrized_polynomial_prog(
      poly_prog, poly, 0,
      1  // The only constraint should be the bounding box non-negativity of
         // lambda.
  );
}

GTEST_TEST(ParametrizedPolyPositive, CtorIntervalDegreeZero) {
  const Variable interval_variable{"mu"};
  const Variable coeff{"coeff"};
  const Variables parameters{Variable("param")};
  const Variable y{"y"};
  // A polynomial without the interval variable
  const Polynomial poly{coeff * (*parameters.begin()) * y * y, {y}};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};

  EXPECT_TRUE(p.EqualTo(poly - lambda));
  EXPECT_TRUE(nu.EqualTo(Polynomial(0)));

  validate_parametrized_polynomial_prog(
      poly_prog, poly,
      1,  // The only constraints should be the non-negativity of lambda.
      0);
}

GTEST_TEST(ParametrizedPolyPositive, CtorScalarDegreeOdd) {
  const int deg{3};
  const Variable interval_variable{"mu"};
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  Polynomial::MapType poly_map;

  Variables parameters{};
  Variables coeffs{};

  for (int i = 0; i < basis.size(); ++i) {
    const Variable param{symbolic::Variable(fmt::format("s{}", i))};
    const Variable coeff{symbolic::Variable(fmt::format("a{}", i))};
    parameters.insert(param);
    coeffs.insert(coeff);
    poly_map.emplace(basis(i), param * coeff);
  }

  const Polynomial poly{poly_map};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      symbolic::Polynomial(interval_variable) * lambda +
      symbolic::Polynomial((1 - interval_variable)) * nu};

  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));

  validate_parametrized_polynomial_prog(
      poly_prog, poly,
      2,  // The only constraints should be the non-negativity of lambda and nu.
      0);
}

GTEST_TEST(ParametrizedPolyPositive, CtorScalarDegreeEven) {
  const int deg{4};
  const Variable interval_variable{"mu"};
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  Polynomial::MapType poly_map;

  Variables parameters{};
  Variables coeffs{};

  for (int i = 0; i < basis.size(); ++i) {
    const Variable param{symbolic::Variable(fmt::format("s{}", i))};
    const Variable coeff{symbolic::Variable(fmt::format("a{}", i))};
    parameters.insert(param);
    coeffs.insert(coeff);
    poly_map.emplace(basis(i), param * coeff);
  }

  const Polynomial poly{poly_map};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      lambda +
      symbolic::Polynomial(interval_variable * (1 - interval_variable)) * nu};

  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));
  validate_parametrized_polynomial_prog(
      poly_prog, poly,
      2,  // The only constraints should be the non-negativity of lambda and nu.
      0);
}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix2DegreeOdd) {
  const int deg{3};
  const Variable interval_variable{"mu"};
  const Variable y0{"y0"};
  const Variable y1{"y1"};
  const Variables auxillary_variables{y0, y1};

  VectorX<symbolic::Expression> poly_mat_upper(3);

  Variables parameters{};
  Variables coeffs{};

  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  for (int i = 0; i < 3; ++i) {
    Polynomial::MapType poly_map;
    for (int j = 0; j < basis.size(); ++j) {
      const Variable param{symbolic::Variable(fmt::format("s{}{}", i, j))};
      const Variable coeff{symbolic::Variable(fmt::format("a{}{}", i, j))};
      parameters.insert(param);
      coeffs.insert(coeff);
      poly_map.emplace(basis(j), param * coeff);
    }
    poly_mat_upper(i) = Polynomial{poly_map}.ToExpression();
  }
  const Polynomial poly{poly_mat_upper(0) * y0 * y0 +
                            2 * poly_mat_upper(1) * y0 * y1 +
                            poly_mat_upper(2) * y1 * y1,
                        Variables({interval_variable, y0, y1})};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      symbolic::Polynomial(interval_variable) * lambda +
      symbolic::Polynomial((1 - interval_variable)) * nu};

  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));
  validate_parametrized_polynomial_prog(
      poly_prog, poly,
      2,  // The only constraints should be the non-negativity of lambda and nu.
      0);
}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix2DegreeEven) {
  const int deg{4};
  const Variable interval_variable{"mu"};
  const Variable y0{"y0"};
  const Variable y1{"y1"};

  VectorX<symbolic::Expression> poly_mat_upper(3);

  Variables parameters{};
  Variables coeffs{};

  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  for (int i = 0; i < 3; ++i) {
    Polynomial::MapType poly_map;
    for (int j = 0; j < basis.size(); ++j) {
      const Variable param{symbolic::Variable(fmt::format("s{}{}", i, j))};
      const Variable coeff{symbolic::Variable(fmt::format("a{}{}", i, j))};
      parameters.insert(param);
      coeffs.insert(coeff);
      poly_map.emplace(basis(j), param * coeff);
    }
    poly_mat_upper(i) = Polynomial{poly_map}.ToExpression();
  }
  const Polynomial poly{poly_mat_upper(0) * y0 * y0 +
                            2 * poly_mat_upper(1) * y0 * y1 +
                            poly_mat_upper(2) * y1 * y1,
                        Variables({interval_variable, y0, y1})};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      lambda +
      symbolic::Polynomial(interval_variable * (1 - interval_variable)) * nu};

  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));
  validate_parametrized_polynomial_prog(
      poly_prog, poly,
      2,  // The only constraints should be the non-negativity of lambda and nu.
      0);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
