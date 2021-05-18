#include "drake/solvers/create_cost.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "drake/common/polynomial.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/common/unused.h"

namespace drake {
namespace solvers {
namespace internal {

using std::make_shared;
using std::numeric_limits;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::unordered_map;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;


namespace {

Binding<QuadraticCost> DoParseQuadraticCost(
    const symbolic::Polynomial& poly, const VectorXDecisionVariable& vars_vec,
    const unordered_map<Variable::Id, int>& map_var_to_index,
    std::optional<bool> is_convex) {
  // We want to write the expression e in the form 0.5 * x' * Q * x + b' * x + c
  // TODO(hongkai.dai): use a sparse matrix to represent Q and b.
  Eigen::MatrixXd Q(vars_vec.size(), vars_vec.size());
  Eigen::VectorXd b(vars_vec.size());
  double constant_term;
  symbolic::DecomposeQuadraticPolynomial(poly, map_var_to_index, &Q, &b,
                                         &constant_term);
  return CreateBinding(
      make_shared<QuadraticCost>(Q, b, constant_term, is_convex), vars_vec);
}

Binding<LinearCost> DoParseLinearCost(
    const Expression &e,
    const VectorXDecisionVariable& vars_vec,
    const unordered_map<Variable::Id, int>& map_var_to_index) {
  Eigen::RowVectorXd c(vars_vec.size());
  double constant_term{};
  symbolic::DecomposeAffineExpression(e, map_var_to_index, c, &constant_term);
  return CreateBinding(make_shared<LinearCost>(c.transpose(), constant_term),
                       vars_vec);
}

}  // anonymous namespace

Binding<LinearCost> ParseLinearCost(const Expression& e) {
  auto p = symbolic::ExtractVariablesFromExpression(e);
  return DoParseLinearCost(e, p.first, p.second);
}

Binding<QuadraticCost> ParseQuadraticCost(const Expression& e,
                                          std::optional<bool> is_convex) {
  // First build an Eigen vector, that contains all the bound variables.
  auto p = symbolic::ExtractVariablesFromExpression(e);
  const auto& vars_vec = p.first;
  const auto& map_var_to_index = p.second;

  // Now decomposes the expression into coefficients and monomials.
  const symbolic::Polynomial poly{e};
  return DoParseQuadraticCost(poly, vars_vec, map_var_to_index, is_convex);
}

Binding<PolynomialCost> ParsePolynomialCost(const symbolic::Expression& e) {
  if (!e.is_polynomial()) {
    ostringstream oss;
    oss << "Expression" << e
        << " is not a polynomial. ParsePolynomialCost"
           " only supports polynomial expression.\n";
    throw runtime_error(oss.str());
  }
  const symbolic::Variables& vars = e.GetVariables();
  const Polynomiald polynomial = Polynomiald::FromExpression(e);
  vector<Polynomiald::VarType> polynomial_vars(vars.size());
  VectorXDecisionVariable var_vec(vars.size());
  int polynomial_var_count = 0;
  for (const auto& var : vars) {
    polynomial_vars[polynomial_var_count] = var.get_id();
    var_vec[polynomial_var_count] = var;
    ++polynomial_var_count;
  }
  return CreateBinding(make_shared<PolynomialCost>(
                           Vector1<Polynomiald>(polynomial), polynomial_vars),
                       var_vec);
}

Binding<Cost> ParseCost(const symbolic::Expression& e) {
  if (!e.is_polynomial()) {
    ostringstream oss;
    oss << "Expression " << e << " is not a polynomial. ParseCost does not"
        << " support non-polynomial expression.\n";
    throw runtime_error(oss.str());
  }
  const symbolic::Polynomial poly{e};
  const int total_degree{poly.TotalDegree()};
  auto e_extracted = symbolic::ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& vars_vec = e_extracted.first;
  const auto& map_var_to_index = e_extracted.second;

  if (total_degree > 2) {
    return ParsePolynomialCost(e);
  } else if (total_degree == 2) {
    return DoParseQuadraticCost(poly, vars_vec, map_var_to_index, std::nullopt);
  } else {
    return DoParseLinearCost(e, vars_vec, map_var_to_index);
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
