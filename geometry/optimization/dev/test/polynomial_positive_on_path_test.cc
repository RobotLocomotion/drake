#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

#include <iostream>
#include <optional>

#include <gtest/gtest.h>

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
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};

  EXPECT_EQ(p.TotalDegree(), 0);
  EXPECT_TRUE(p.EqualTo(poly - lambda));
  EXPECT_TRUE(nu.EqualTo(symbolic::Polynomial(0)));

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
}

GTEST_TEST(ParametrizedPolyPositive, CtorScalarDegreeOdd) {
  const int deg{3};
  const Variable interval_variable{"mu"};
  const Variable coeff{"coeff"};
  const Variables parameters{coeff};
  // A polynomial with no indeterminates.
  const Polynomial poly{coeff, symbolic::Variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  const Polynomial lambda{poly_prog.get_lambda()};
  const Polynomial nu{poly_prog.get_nu()};
  const Polynomial p{poly_prog.get_p()};

  EXPECT_EQ(p.TotalDegree(), 0);
  EXPECT_TRUE(p.EqualTo(poly - lambda));
  EXPECT_TRUE(nu.EqualTo(symbolic::Polynomial(0)));

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

}

GTEST_TEST(ParametrizedPolyPositive, CtorScalarDegreeEven) {}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix2DegreeOdd) {}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix2DegreeEven) {}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix3DegreeOdd) {}

GTEST_TEST(ParametrizedPolyPositive, CtorMatrix3DegreeEven) {}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake