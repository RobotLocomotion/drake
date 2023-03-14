#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

#include <optional>

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

GTEST_TEST(ParametrizedPolyPositive, CtorDegreeZero) {
  const Variable interval_variable{"mu"};
  const Variable coeff{"coeff"};
  const Variables parameters{coeff};
  // A polynomial with no indeterminates.
  const Polynomial poly{coeff, symbolic::Variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};

  EXPECT_EQ(p.TotalDegree(), 0);
  EXPECT_TRUE(p.EqualTo(poly - lambda));
  EXPECT_TRUE(nu.EqualTo(Polynomial(0)));

  const MathematicalProgram* psatz_variables_and_constraints =
      poly_prog.get_psatz_variables_and_psd_constraints();

  EXPECT_EQ(psatz_variables_and_constraints->indeterminates().size(), 1);
  EXPECT_TRUE(psatz_variables_and_constraints->indeterminate(0).equal_to(
      interval_variable));
  for (const auto& var : p.decision_variables()) {
    if (parameters.include(var)) {
      // Parameters should not be in the decision variables.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints->FindDecisionVariableIndex(var)),
          std::runtime_error);
    } else {
      // Ensure that all the decision variables of p are in the program.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints->FindDecisionVariableIndex(var)));
    }
  }

  // The only constraint should be the bounding box non-negativity of lambda.
  EXPECT_EQ(psatz_variables_and_constraints->bounding_box_constraints().size(),
            1);
  EXPECT_EQ(psatz_variables_and_constraints->GetAllConstraints().size(), 1);
}

GTEST_TEST(ParametrizedPolyPositive, CtorScalarDegreeOdd) {
  const int deg{3};
  const Variable interval_variable{"mu"};
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  Polynomial::MapType poly_map;
  for (int i = 0; i < basis.size(); ++i) {
    poly_map.emplace(basis(i), symbolic::Variable(fmt::format("s{}", i)));
  }

  const Polynomial poly{poly_map};
  const Variables parameters{poly.decision_variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      symbolic::Polynomial(interval_variable) * lambda +
      symbolic::Polynomial((1 - interval_variable)) * nu};

  EXPECT_EQ(p.Degree(interval_variable), deg);
  EXPECT_EQ(lambda.Degree(interval_variable), deg - 1);
  EXPECT_EQ(nu.Degree(interval_variable), deg - 1);
  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));

  const MathematicalProgram* psatz_variables_and_constraints =
      poly_prog.get_psatz_variables_and_psd_constraints();

  EXPECT_EQ(psatz_variables_and_constraints->indeterminates().size(), 1);
  EXPECT_TRUE(psatz_variables_and_constraints->indeterminate(0).equal_to(
      interval_variable));
  for (const auto& var : p.decision_variables()) {
    if (parameters.include(var)) {
      // Parameters should not be in the decision variables.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints->FindDecisionVariableIndex(var)),
          std::runtime_error);
    } else {
      // Ensure that all the decision variables of p are in the program.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints->FindDecisionVariableIndex(var)));
    }
  }

  // The only constraints should be the PSD constraints on λ and ν.
  EXPECT_EQ(psatz_variables_and_constraints->positive_semidefinite_constraints()
                .size(),
            2);
  EXPECT_EQ(psatz_variables_and_constraints->GetAllConstraints().size(), 2);
}

GTEST_TEST(ParametrizedPolyPositive, CtorScalarDegreeEven) {
  const int deg{4};
  const Variable interval_variable{"mu"};
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  Polynomial::MapType poly_map;
  for (int i = 0; i < basis.size(); ++i) {
    poly_map.emplace(basis(i), symbolic::Variable(fmt::format("s{}", i)));
  }

  const Polynomial poly{poly_map};
  const Variables parameters{poly.decision_variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      lambda +
      symbolic::Polynomial(interval_variable * (1 - interval_variable)) * nu};

  EXPECT_EQ(p.Degree(interval_variable), deg);
  EXPECT_EQ(lambda.Degree(interval_variable), deg);
  EXPECT_EQ(nu.Degree(interval_variable), deg - 2);
  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));

  const MathematicalProgram* psatz_variables_and_constraints =
      poly_prog.get_psatz_variables_and_psd_constraints();

  EXPECT_EQ(psatz_variables_and_constraints->indeterminates().size(), 1);
  EXPECT_TRUE(psatz_variables_and_constraints->indeterminate(0).equal_to(
      interval_variable));
  for (const auto& var : p.decision_variables()) {
    if (parameters.include(var)) {
      // Parameters should not be in the decision variables.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints->FindDecisionVariableIndex(var)),
          std::runtime_error);
    } else {
      // Ensure that all the decision variables of p are in the program.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints->FindDecisionVariableIndex(var)));
    }
  }

  // The only constraints should be the PSD constraints on λ and ν.
  EXPECT_EQ(psatz_variables_and_constraints->positive_semidefinite_constraints()
                .size(),
            2);
  EXPECT_EQ(psatz_variables_and_constraints->GetAllConstraints().size(), 2);
}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix2DegreeOdd) {
  const int deg{3};
  const Variable interval_variable{"mu"};
  const Variable y0{"y0"};
  const Variable y1{"y1"};
  const Variables auxillary_variables{y0, y1};
  const Variables all_indets{y0, y1, interval_variable};

  VectorX<symbolic::Expression> poly_mat_upper(3);
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  for (int i = 0; i < 3; ++i) {
    Polynomial::MapType poly_map;
    for (int j = 0; j < basis.size(); ++j) {
      poly_map.emplace(basis(j),
                       symbolic::Variable(fmt::format("s_{}_{}", i, j)));
    }
    poly_mat_upper(i) = Polynomial{poly_map}.ToExpression();
  }
  const Polynomial poly{poly_mat_upper(0) * y0 * y0 +
                            2 * poly_mat_upper(1) * y0 * y1 +
                            poly_mat_upper(2) * y1 * y1,
                        Variables({interval_variable, y0, y1})};


  const Variables parameters{poly.decision_variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      symbolic::Polynomial(interval_variable) * lambda +
      symbolic::Polynomial((1 - interval_variable)) * nu};

  EXPECT_EQ(p.Degree(interval_variable), deg);
  EXPECT_EQ(lambda.Degree(interval_variable), deg - 1);
  EXPECT_EQ(nu.Degree(interval_variable), deg - 1);

  EXPECT_EQ(p.Degree(y0), 2);
  EXPECT_EQ(lambda.Degree(y0), 2);
  EXPECT_EQ(nu.Degree(y0), 2);

  EXPECT_EQ(p.Degree(y1), 2);
  EXPECT_EQ(lambda.Degree(y1), 2);
  EXPECT_EQ(nu.Degree(y1), 2);

  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));

  const MathematicalProgram* psatz_variables_and_constraints =
      poly_prog.get_psatz_variables_and_psd_constraints();

  // psatz_variables_and_constraints contains interval variables and auxillary
  // variables in its indeterminates.
  EXPECT_EQ(psatz_variables_and_constraints->indeterminates().size(), 3);
  const auto indeterminates_index =
      psatz_variables_and_constraints->indeterminates_index();
  for (const auto& var : all_indets) {
    const auto it = indeterminates_index.find(var.get_id());
    ASSERT_TRUE(it != indeterminates_index.end());
  }

  for (const auto& var : p.decision_variables()) {
    if (parameters.include(var)) {
      // Parameters should not be in the decision variables.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints->FindDecisionVariableIndex(var)),
          std::runtime_error);
    } else {
      // Ensure that all the decision variables of p are in the program.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints->FindDecisionVariableIndex(var)));
    }
  }

  // The only constraints should be the PSD constraints on λ and ν.
  EXPECT_EQ(psatz_variables_and_constraints->positive_semidefinite_constraints()
                .size(),
            2);
  EXPECT_EQ(psatz_variables_and_constraints->GetAllConstraints().size(), 2);
}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix2DegreeEven) {
  const int deg{4};
  const Variable interval_variable{"mu"};
  const Variable y0{"y0"};
  const Variable y1{"y1"};
  const Variables auxillary_variables{y0, y1};
  const Variables all_indets{y0, y1, interval_variable};

  VectorX<symbolic::Expression> poly_mat_upper(3);
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  for (int i = 0; i < 3; ++i) {
    Polynomial::MapType poly_map;
    for (int j = 0; j < basis.size(); ++j) {
      poly_map.emplace(basis(j),
                       symbolic::Variable(fmt::format("s_{}_{}", i, j)));
    }
    poly_mat_upper(i) = Polynomial{poly_map}.ToExpression();
  }
  const Polynomial poly{poly_mat_upper(0) * y0 * y0 +
                            2 * poly_mat_upper(1) * y0 * y1 +
                            poly_mat_upper(2) * y1 * y1,
                        Variables({interval_variable, y0, y1})};


  const Variables parameters{poly.decision_variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};
  const Polynomial rhs_expected{
      lambda +
      symbolic::Polynomial(interval_variable * (1 - interval_variable)) * nu};

  EXPECT_EQ(p.Degree(interval_variable), deg);
  EXPECT_EQ(lambda.Degree(interval_variable), deg);
  EXPECT_EQ(nu.Degree(interval_variable), deg - 2);
  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));

  EXPECT_EQ(p.Degree(y0), 2);
  EXPECT_EQ(lambda.Degree(y0), 2);
  EXPECT_EQ(nu.Degree(y0), 2);

  EXPECT_EQ(p.Degree(y1), 2);
  EXPECT_EQ(lambda.Degree(y1), 2);
  EXPECT_EQ(nu.Degree(y1), 2);

  EXPECT_TRUE(p.EqualTo(poly - rhs_expected));

  const MathematicalProgram* psatz_variables_and_constraints =
      poly_prog.get_psatz_variables_and_psd_constraints();

  // psatz_variables_and_constraints contains interval variables and auxillary
  // variables in its indeterminates.
  EXPECT_EQ(psatz_variables_and_constraints->indeterminates().size(), 3);
  const auto indeterminates_index =
      psatz_variables_and_constraints->indeterminates_index();
  for (const auto& var : all_indets) {
    const auto it = indeterminates_index.find(var.get_id());
    ASSERT_TRUE(it != indeterminates_index.end());
  }

  for (const auto& var : p.decision_variables()) {
    if (parameters.include(var)) {
      // Parameters should not be in the decision variables.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints->FindDecisionVariableIndex(var)),
          std::runtime_error);
    } else {
      // Ensure that all the decision variables of p are in the program.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints->FindDecisionVariableIndex(var)));
    }
  }

  // The only constraints should be the PSD constraints on λ and ν.
  EXPECT_EQ(psatz_variables_and_constraints->positive_semidefinite_constraints()
                .size(),
            2);
  EXPECT_EQ(psatz_variables_and_constraints->GetAllConstraints().size(), 2);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
