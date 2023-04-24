#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/solve.h"

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

  // poly member of ParametrizedPolynomialPositiveOnUnitInterval should be
  // exactly the polynomial poly.
  EXPECT_TRUE(poly.EqualTo(poly_prog.get_poly()));

  // p should have the same degree as poly
  EXPECT_EQ(p.TotalDegree(), poly.TotalDegree());
  // p should have the same indeterminates as poly.
  EXPECT_TRUE(p.indeterminates() == poly.indeterminates());
  for (const auto& var : p.indeterminates()) {
    if (!var.equal_to(poly_prog.get_mu())) {
      EXPECT_EQ(p.Degree(var), 2);
      EXPECT_EQ(lambda.Degree(var), 2);
      EXPECT_TRUE(nu.TotalDegree() == 0 || nu.Degree(var) == 2);
    } else {
      const int deg = poly.Degree(var);
      EXPECT_EQ(p.Degree(var), deg);
      if (deg % 2 == 0) {
        EXPECT_EQ(lambda.Degree(var), deg);
        EXPECT_EQ(nu.Degree(var), deg - 2);
      } else {
        EXPECT_EQ(lambda.Degree(var), deg - 1);
        EXPECT_EQ(nu.Degree(var), deg - 1);
      }
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
    if (poly_prog.get_parameters().include(var)) {
      // The parameters of p_ should not be in psatz_variables_and_constraints.
      EXPECT_THROW(
          unused(
              psatz_variables_and_constraints.FindDecisionVariableIndex(var)),
          std::runtime_error);

    } else {
      // All other decision variables of p_ should be in
      // psatz_variables_and_constraints.
      EXPECT_NO_THROW(unused(
          psatz_variables_and_constraints.FindDecisionVariableIndex(var)));
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

class AddPositivityConstraintToProgramTest : public ::testing::Test {
 public:
  AddPositivityConstraintToProgramTest()
      : prog_{},
        // add some random decision variables
        initial_decision_variables_{prog_.NewContinuousVariables(3, "a")},
        // add some useless indeterminates.
        initial_indeterminates_(prog_.NewIndeterminates(2, "x")),
        // Add a cost
        cost_(prog_.AddLinearCost(initial_decision_variables_(0) +
                                  initial_decision_variables_(1) +
                                  initial_decision_variables_(2))),
        // Add a constraint
        initial_constraint_(prog_.AddLinearConstraint(
            initial_decision_variables_(0) + initial_decision_variables_(1) +
                initial_decision_variables_(2) >=
            2)) {}

  void CheckAddPositivityConstraintToProgram(
      const symbolic::Environment& env,
      const ParametrizedPolynomialPositiveOnUnitInterval* poly_prog) {
    // Create a test prog to avoid mutating the member and be able to call this
    // method multiple times in one fixture.
    auto test_prog = prog_.Clone();
    test_prog->AddIndeterminates(poly_prog->get_p().indeterminates());
    const int expected_indeterminates_size = test_prog->indeterminates().size();

    const int expected_num_decision_variables =
        test_prog->decision_variables().size() +
        poly_prog->get_psatz_variables_and_psd_constraints()
            .decision_variables()
            .size();
    const int expected_num_constraints =
        test_prog->GetAllConstraints().size() +  // number of constraints
                                                 // already in the program.
        poly_prog->get_psatz_variables_and_psd_constraints()
            .GetAllConstraints()
            .size() +  // number of multiplier positivity constraints.
        poly_prog->get_p()
            .monomial_to_coefficient_map()
            .size();  // number of constraints for coefficient matching
                      // condition

    poly_prog->AddPositivityConstraintToProgram(env, test_prog.get());

    EXPECT_EQ(test_prog->indeterminates().size(), expected_indeterminates_size);
    // No costs added or changed.
    EXPECT_EQ(test_prog->GetAllCosts().size(), 1);
    EXPECT_EQ(*test_prog->GetAllCosts().begin(), cost_);
    EXPECT_EQ(test_prog->decision_variables().size(),
              expected_num_decision_variables);
    EXPECT_EQ(test_prog->GetAllConstraints().size(), expected_num_constraints);
    // The only semidefinite variables in this test program are the psatz
    // variables.
    EXPECT_EQ(test_prog->positive_semidefinite_constraints().size(),
              poly_prog->get_psatz_variables_and_psd_constraints()
                  .positive_semidefinite_constraints()
                  .size());
    auto result = solvers::Solve(*test_prog);
    EXPECT_TRUE(result.is_success());

    // Test that p_ evaluates to 0 with the optimization solution and the given
    // environment.
    symbolic::Environment env_with_solution;
    for (const auto& coeff : poly_prog->get_p().decision_variables()) {
      auto parameter_val = env.find(coeff);
      if (parameter_val != env.end()) {
        env_with_solution.insert(parameter_val->first, parameter_val->second);
      } else {
        env_with_solution.insert(coeff, result.GetSolution(coeff));
      }
    }
    const Polynomial p_sol{
        poly_prog->get_p().EvaluatePartial(env_with_solution)};
    EXPECT_TRUE(p_sol.CoefficientsAlmostEqual(symbolic::Polynomial({}), 1E-6));
  }

 protected:
  MathematicalProgram prog_;

  const solvers::VectorXDecisionVariable initial_decision_variables_;
  const solvers::VectorXIndeterminate initial_indeterminates_;

  const solvers::Binding<solvers::LinearCost> cost_;
  const solvers::Binding<solvers::LinearConstraint> initial_constraint_;
};

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
  );     // NOLINT
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

TEST_F(AddPositivityConstraintToProgramTest,
       AddPositivityConstraintTotalDegreeZero) {
  const Variable coeff{"coeff"};
  const Variables parameters{coeff};
  // A polynomial with no indeterminates.
  const Polynomial poly{coeff * (*parameters.begin()), Variables()};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{poly, Variable(),
                                                               parameters};

  symbolic::Environment env{{*parameters.begin(), 3}};
  CheckAddPositivityConstraintToProgram(env, &poly_prog);

  symbolic::Environment env2{{*parameters.begin(), 5}};
  CheckAddPositivityConstraintToProgram(env2, &poly_prog);
}

TEST_F(AddPositivityConstraintToProgramTest,
       AddPositivityConstraintScalarPolynomial) {
  const int deg{3};
  const Variable interval_variable{"mu"};
  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  Polynomial::MapType poly_map;

  Variables parameters{};
  Variables coeffs{};

  symbolic::Environment env1;
  symbolic::Environment env2;
  for (int i = 0; i < basis.size(); ++i) {
    const Variable param{symbolic::Variable(fmt::format("s{}", i))};
    const Variable coeff{symbolic::Variable(fmt::format("a{}", i))};
    parameters.insert(param);
    coeffs.insert(coeff);
    env1.insert(param, i);
    env2.insert(param, -i);
    poly_map.emplace(basis(i), param * coeff);
  }

  const Polynomial poly{poly_map};

  const ParametrizedPolynomialPositiveOnUnitInterval poly_prog{
      poly, interval_variable, parameters};

  CheckAddPositivityConstraintToProgram(env1, &poly_prog);
  CheckAddPositivityConstraintToProgram(env2, &poly_prog);
}

TEST_F(AddPositivityConstraintToProgramTest,
       AddPositivityConstraintMatrixSOSPolynomial) {
  const int deg{4};
  const Variable interval_variable{"mu"};
  const Variable y0{"y0"};
  const Variable y1{"y1"};
  const Variables auxillary_variables{y0, y1};

  VectorX<symbolic::Expression> poly_mat_upper(3);

  Variables parameters{};
  Variables coeffs{};

  const VectorX<symbolic::Monomial> basis{
      symbolic::MonomialBasis({interval_variable}, deg)};
  symbolic::Environment env1;
  symbolic::Environment env2;
  for (int i = 0; i < 3; ++i) {
    Polynomial::MapType poly_map;
    for (int j = 0; j < basis.size(); ++j) {
      const Variable param{symbolic::Variable(fmt::format("s{}{}", i, j))};
      env1.insert(param, i + j);
      env2.insert(param, -i + j);

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
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
