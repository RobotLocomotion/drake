#include "drake/solvers/symbolic_extraction.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;

using std::numeric_limits;
using std::string;
using std::unordered_map;

using MapVarToIndex = unordered_map<Variable::Id, int>;
using VectorXe = drake::VectorX<Expression>;

namespace drake {
namespace solvers {
namespace internal {
namespace {

constexpr double kTol = numeric_limits<double>::epsilon();

GTEST_TEST(SymbolicExtraction, AppendToVector) {
  const Variable x("x");
  const Variable y("y");
  const Variable z("z");
  const VectorDecisionVariable<3> vars_expected(x, y, z);

  VectorXDecisionVariable vars;
  AppendToVector(x, &vars);
  AppendToVector(y, &vars);
  AppendToVector(z, &vars);
  EXPECT_EQ(vars_expected, vars);
}

// Check expected invariance
void ExpectValidMapVarToIndex(const VectorXDecisionVariable& vars,
                              const MapVarToIndex& map_var_to_index) {
  EXPECT_EQ(vars.size(), map_var_to_index.size());
  for (int i = 0; i < vars.size(); ++i) {
    const auto& var = vars(i);
    EXPECT_EQ(i, map_var_to_index.at(var.get_id()));
  }
}

GTEST_TEST(SymbolicExtraction, ExtractVariables) {
  const Variable x("x");
  const Variable y("y");
  Expression e = x + y;
  VectorXDecisionVariable vars_expected(2);
  vars_expected << x, y;

  MapVarToIndex map_var_to_index;
  VectorXDecisionVariable vars;
  std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);

  const Variable z("z");
  e += x * (z - y);
  AppendToVector(z, &vars_expected);

  ExtractAndAppendVariablesFromExpression(e, &vars, &map_var_to_index);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);
}

// Make off-diagonal terms symmetric for a given input matrix
template <typename Derived>
void AverageOffDiagonalTerms(const Eigen::MatrixBase<Derived>& Q_asym,
                             Eigen::MatrixBase<Derived>* pQ_sym) {
  DRAKE_ASSERT(Q_asym.rows() == Q_asym.cols());
  Derived& Q_sym = pQ_sym->derived();
  Q_sym = Q_asym;
  int size = Q_sym.rows();
  for (int r = 0; r < size - 1; ++r) {
    for (int c = r + 1; c < size; ++c) {
      Q_sym(r, c) = 0.5 * (Q_asym(r, c) + Q_asym(c, r));
      Q_sym(c, r) = Q_sym(r, c);
    }
  }
}

GTEST_TEST(SymbolicExtraction, DecomposeQuadraticExpression) {
  const int num_variables = 3;
  const Variable x("x");
  const Variable y("y");
  const Variable z("z");
  const VectorDecisionVariable<3> vars_expected(x, y, z);

  Matrix3d Q_diagonal, Q_symmetric, Q_asymmetric;
  Q_diagonal.setZero().diagonal() = Vector3d(1, 2, 3);
  Q_symmetric <<
      3, 2, 1,
      2, 4, 5,
      1, 5, 6;
  Q_asymmetric <<
    3, 2, 1,
    4, 6, 5,
    7, 8, 9;

  const Vector3d b_expected(10, 11, 12);
  const double c_expected = 13;

  for (const Matrix3d& Q_in : {Q_diagonal, Q_symmetric, Q_asymmetric}) {
    const Expression e =
        vars_expected.dot(0.5 * Q_in * vars_expected + b_expected) + c_expected;

    const auto pair = ExtractVariablesFromExpression(e);
    const VectorXDecisionVariable& vars = pair.first;
    const MapVarToIndex& map_var_to_index = pair.second;
    EXPECT_EQ(vars_expected, vars);

    const symbolic::Polynomial poly{e};

    MatrixXd Q(num_variables, num_variables);
    VectorXd b(num_variables);
    double c;
    DecomposeQuadraticPolynomial(poly, map_var_to_index, &Q, &b, &c);
    Matrix3d Q_expected;
    AverageOffDiagonalTerms(Q_in, &Q_expected);
    EXPECT_TRUE(CompareMatrices(Q_expected, Q, kTol));
    EXPECT_TRUE(CompareMatrices(b_expected, b, kTol));
    EXPECT_EQ(c_expected, c);
  }
}

// Return scalar value from an effectively scalar Eigen expression
template <typename Derived>
const typename Derived::Scalar& AsScalar(const Eigen::EigenBase<Derived>& X) {
  DRAKE_ASSERT(X.rows() == 1 && X.cols() == 1);
  return X.derived().coeffRef(0, 0);
}

GTEST_TEST(SymbolicExtraction, DecomposeLinearExpression) {
  const int num_variables = 3;
  const Variable x("x");
  const Variable y("y");
  const Variable z("z");

  const VectorDecisionVariable<3> vars_expected(x, y, z);

  // Scalar value case
  {
    MatrixXd coeffs_expected(1, num_variables);
    coeffs_expected << 1, 2, 3;
    double c_expected = 4;
    const Expression e = AsScalar(coeffs_expected * vars_expected) + c_expected;

    const auto pair = ExtractVariablesFromExpression(e);
    const VectorXDecisionVariable& vars = pair.first;
    const MapVarToIndex& map_var_to_index = pair.second;
    EXPECT_EQ(vars_expected, vars);

    MatrixXd coeffs(1, num_variables);
    double c;
    DecomposeLinearExpression(e, map_var_to_index, coeffs, &c);

    EXPECT_TRUE(CompareMatrices(coeffs_expected, coeffs, kTol));
    EXPECT_EQ(c_expected, c);
  }

  // Vector value case
  {
    const int num_eq = 3;
    MatrixXd coeffs_expected(num_eq, num_variables);
    coeffs_expected <<
        1, 2, 3,
        4, 5, 6,
        7, 8, 9;
    VectorXd c_expected(num_eq);
    c_expected << 10, 11, 12;

    const VectorXe ev = coeffs_expected * vars_expected + c_expected;

    MatrixXd coeffs(num_eq, num_variables);
    VectorXd c(num_eq);
    VectorXDecisionVariable vars;
    DecomposeLinearExpression(ev, &coeffs, &c, &vars);

    EXPECT_EQ(vars_expected, vars);
    EXPECT_TRUE(CompareMatrices(coeffs_expected, coeffs, kTol));
    EXPECT_TRUE(CompareMatrices(c_expected, c, kTol));
  }
}

}  // anonymous namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
