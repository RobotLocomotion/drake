#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

// TODO(soonho-tri): Migrate the functions in drake/solvers:symbolic_extract to
// this file.

namespace drake {
namespace symbolic {

/// Decomposes @p expressions into @p M * @p vars.
///
/// @throws std::runtime_error if @p expressions is not linear in @p vars.
/// @pre M.rows() == expressions.rows() && M.cols() == vars.rows().
void DecomposeLinearExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M);

/// Decomposes @p expressions into @p M * @p vars + @p v.
///
/// @throws std::runtime_error if @p expressions is not affine in @p vars.
/// @pre M.rows() == expressions.rows() && M.cols() == vars.rows().
/// @pre v.rows() == expressions.rows().
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M, EigenPtr<Eigen::VectorXd> v);

/*
 * Given an expression `e`, extract all variables inside `e`, append these
 * variables to `vars` if they are not included in `vars` yet.
 * @param[in] e  A symbolic expression.
 * @param[in,out] vars  As an input, `vars` contain the variables before
 * extracting expression `e`. As an output, the variables in `e` that were not
 * included in `vars`, will be appended to the end of `vars`.
 * @param[in,out] map_var_to_index. map_var_to_index is of the same size as
 * `vars`, and map_var_to_index[vars(i).get_id()] = i. This invariance holds
 * for map_var_to_index both as the input and as the output.
 */
void ExtractAndAppendVariablesFromExpression(
    const symbolic::Expression& e, VectorX<Variable>* vars,
    std::unordered_map<symbolic::Variable::Id, int>* map_var_to_index);

/*
 * Given an expression `e`, extracts all variables inside `e`.
 * @param[in] e A symbolic expression.
 * @retval pair pair.first is the variables in `e`. pair.second is the mapping
 * from the variable ID to the index in pair.first, such that
 * pair.second[pair.first(i).get_id()] = i
 */
std::pair<VectorX<Variable>, std::unordered_map<symbolic::Variable::Id, int>>
ExtractVariablesFromExpression(const symbolic::Expression& e);

/*
 * Given a quadratic polynomial @p poly, decomposes it into the form 0.5 * x' *
 * Q * x + b' * x + c
 *
 * @param[in] poly Quadratic polynomial to decompose.
 * @param[in] map_var_to_index maps variables in `poly.GetVariables()` to the
 * index in the vector `x`.
 * @param Q[out] The Hessian of the quadratic expression. @pre The size of Q
 * should be `num_variables x num_variables`. Q is a symmetric matrix.
 * @param b[out] The linear term of the quadratic expression. @pre The size of
 * `b` should be `num_variables`.
 * @param c[out] The constant term of the quadratic expression.
 */
void DecomposeQuadraticPolynomial(
    const symbolic::Polynomial& poly,
    const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c);

/*
 * Given a vector of affine expressions v, decompose it to
 * \f$ v = A vars + b \f$
 * @param[in] v A vector of affine expressions
 * @param[out] A The matrix containing the linear coefficients.
 * @param[out] b The vector containing all the constant terms.
 * @param[out] vars All variables.
 */
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    Eigen::MatrixXd* A, Eigen::VectorXd* b, VectorX<Variable>* vars);

/*
 * Decomposes an affine combination @p e = c0 + c1 * v1 + ... cn * vn into
 * the following:
 *
 *     constant term      : c0
 *     coefficient vector : [c1, ..., cn]
 *     variable vector    : [v1, ..., vn]
 *
 *  Then, it extracts the coefficient and the constant term.
 *  A map from variable ID to int, @p map_var_to_index, is used to decide a
 *  variable's index in a linear combination.
 *
 *  \pre{1. @c coeffs is a row vector of double, whose length matches with the
 *          size of @c map_var_to_index.
 *       2. e.is_polynomial() is true.
 *       3. e is a linear expression.}
 * @tparam Derived An Eigen row vector type with Derived::Scalar == double.
 * @param[in] e The symbolic linear expression
 * @param[in] map_var_to_index A mapping from variable ID to variable index,
 * such that map_var_to_index[vi.get_ID()] = i.
 * @param[out] coeffs A row vector. coeffs(i) = ci.
 * @param[out] constant_term c0 in the equation above.
 * @return num_variable. Number of variables in the expression. 2 * x(0) + 3
 * has 1 variable, 2 * x(0) + 3 * x(1) - 2 * x(0) has 1 variable.
 */
template <typename Derived>
typename std::enable_if<std::is_same<typename Derived::Scalar, double>::value,
                        int>::type
DecomposeAffineExpression(
    const symbolic::Expression& e,
    const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
    const Eigen::MatrixBase<Derived>& coeffs, double* constant_term) {
  DRAKE_DEMAND(coeffs.rows() == 1);
  DRAKE_DEMAND(coeffs.cols() == static_cast<int>(map_var_to_index.size()));
  if (!e.is_polynomial()) {
    std::ostringstream oss;
    oss << "Expression " << e << "is not a polynomial.\n";
    throw std::runtime_error(oss.str());
  }
  const symbolic::Polynomial poly{e};
  int num_variable = 0;
  for (const auto& p : poly.monomial_to_coefficient_map()) {
    const auto& p_monomial = p.first;
    DRAKE_ASSERT(is_constant(p.second));
    const double p_coeff = symbolic::get_constant_value(p.second);
    if (p_monomial.total_degree() > 1) {
      std::stringstream oss;
      oss << "Expression " << e << " is non-linear.";
      throw std::runtime_error(oss.str());
    } else if (p_monomial.total_degree() == 1) {
      // Linear coefficient.
      const auto& p_monomial_powers = p_monomial.get_powers();
      DRAKE_DEMAND(p_monomial_powers.size() == 1);
      const symbolic::Variable::Id var_id =
          p_monomial_powers.begin()->first.get_id();
      // TODO(eric.cousineau): Avoid using const_cast.
      const_cast<Eigen::MatrixBase<Derived>&>(coeffs)(
          map_var_to_index.at(var_id)) = p_coeff;
      if (p_coeff != 0) {
        ++num_variable;
      }
    } else {
      // Constant term.
      *constant_term = p_coeff;
    }
  }
  return num_variable;
}
}  // namespace symbolic
}  // namespace drake
