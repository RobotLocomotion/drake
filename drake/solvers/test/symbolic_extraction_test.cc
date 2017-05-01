#include "drake/solvers/symbolic_extraction.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/monomial.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;

using std::all_of;
using std::cref;
using std::enable_if;
using std::endl;
using std::is_permutation;
using std::is_same;
using std::make_shared;
using std::map;
using std::move;
using std::numeric_limits;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

using MapVarToIndex = unordered_map<Variable::Id, int>;
using VectorXe = drake::VectorX<Expression>;

namespace drake {
namespace solvers {
namespace internal {
namespace {

const double tol = 1e-14;

GTEST_TEST(SymbolicExtraction, AppendToVector) {
  Variable x("x");
  Variable y("y");
  Variable z("z");
  VectorXDecisionVariable vars_expected(3);
  vars_expected << x, y, z;

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
  Variable x("x");
  Variable y("y");
  Expression e = x + y;
  VectorXDecisionVariable vars_expected(2);
  vars_expected << x, y;

  MapVarToIndex map_var_to_index;
  VectorXDecisionVariable vars;
  std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);

  Variable z("z");
  e += x * (z - y);
  AppendToVector(z, &vars_expected);

  ExtractAndAppendVariablesFromExpression(e, &vars, &map_var_to_index);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);
}

// Make off-diagonal terms symmetric for a given input matrix
void AverageOffDiagonalTerms(const MatrixXd& Q_asym, MatrixXd* pQ_sym) {
  DRAKE_ASSERT(Q_asym.rows() == Q_asym.cols());
  MatrixXd& Q_sym = *pQ_sym;
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
  const int num_var = 3;
  Variable x("x");
  Variable y("y");
  Variable z("z");

  VectorXDecisionVariable vars_expected(num_var);
  vars_expected << x, y, z;
  MatrixXd Q_in(num_var, num_var);
  Q_in <<
             3, 2, 1,
             4, 6, 5,
             7, 8, 9;
  VectorXd b_expected(num_var);
  b_expected << 10, 11, 12;
  double c_expected = 13;
  Expression e =
      vars_expected.dot(0.5 * Q_in * vars_expected + b_expected) + c_expected;

  MapVarToIndex map_var_to_index;
  VectorXDecisionVariable vars;
  std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
  EXPECT_EQ(vars_expected, vars);

  const auto monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, e.GetVariables());

  MatrixXd Q(num_var, num_var);
  VectorXd b(num_var);
  double c;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, map_var_to_index, num_var, &Q, &b, &c);
  MatrixXd Q_expected(num_var, num_var);
  AverageOffDiagonalTerms(Q_in, &Q_expected);
  EXPECT_TRUE(CompareMatrices(Q_expected, Q, tol));
  EXPECT_TRUE(CompareMatrices(b_expected, b, tol));
  EXPECT_EQ(c_expected, c);
}

// Return scalar value from an effectively scalar Eigen expression
template <typename Derived>
const typename Derived::Scalar& AsScalar(const Eigen::EigenBase<Derived>& X) {
  DRAKE_ASSERT(X.rows() == 1 && X.cols() == 1);
  return X.derived().coeffRef(0, 0);
}

GTEST_TEST(SymbolicExtraction, DecomposeLinearExpression) {
  const int num_var = 3;
  Variable x("x");
  Variable y("y");
  Variable z("z");

  VectorXDecisionVariable vars_expected(num_var);
  vars_expected << x, y, z;

  // Scalar value case
  {
    MatrixXd coeffs_expected(1, num_var);
    coeffs_expected << 1, 2, 3;
    double c_expected = 4;
    Expression e = AsScalar(coeffs_expected * vars_expected) + c_expected;

    MapVarToIndex map_var_to_index;
    VectorXDecisionVariable vars;
    std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
    EXPECT_EQ(vars_expected, vars);

    MatrixXd coeffs(1, num_var);
    double c;
    DecomposeLinearExpression(e, map_var_to_index, coeffs, &c);

    EXPECT_TRUE(CompareMatrices(coeffs_expected, coeffs, tol));
    EXPECT_EQ(c_expected, c);
  }

  // Vector value case
  {
    const int num_eq = 3;
    MatrixXd coeffs_expected(num_eq, num_var);
    coeffs_expected <<
        1, 2, 3,
        4, 5, 6,
        7, 8, 9;
    VectorXd c_expected(num_eq);
    c_expected << 10, 11, 12;

    VectorXe ev = coeffs_expected * vars_expected + c_expected;

    MatrixXd coeffs(num_eq, num_var);
    VectorXd c(num_eq);
    VectorXDecisionVariable vars;
    DecomposeLinearExpression(ev, &coeffs, &c, &vars);

    EXPECT_EQ(vars_expected, vars);
    EXPECT_TRUE(CompareMatrices(coeffs_expected, coeffs, tol));
    EXPECT_TRUE(CompareMatrices(c_expected, c, tol));
  }
}

}  // anonymous namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
