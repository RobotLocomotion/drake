#pragma once

#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
namespace internal {

/*
 * Append a scalar value to an Eigen Vector.
 */
template<typename Derived,
    typename = typename std::enable_if<Derived::ColsAtCompileTime == 1>::type>
void AppendToVector(const typename Derived::Scalar& s,
                    Eigen::MatrixBase<Derived>* px) {
  Derived& derived = px->derived();
  int initial_size = derived.size();
  // TODO(eric.cousineau): Relax to single-argument conservativeResize once
  // we resolve the following issue (at least for symbolic:: types):
  // https://github.com/RobotLocomotion/drake/issues/5974
  // For now, just use the workaround to force explicit copying / moving for
  // Eigen 3.3.3.
  derived.conservativeResize(initial_size + 1, Eigen::NoChange);
  // derived.conservativeResize(derived.size() + 1);
  derived(initial_size) = s;
}

/*
 * Adds context to symbolic expression-related errors.
 */
class SymbolicError : public std::runtime_error {
 public:
  SymbolicError(const symbolic::Expression& e, const std::string& msg);
  SymbolicError(const symbolic::Expression& e, double lb, double ub,
                const std::string& msg);

 private:
  static std::string make_string(const symbolic::Expression& e,
                                 const std::string& msg);
  static std::string make_string(const symbolic::Expression& e, double lb,
                                 double ub, const std::string& msg);
};

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
    const symbolic::Expression& e, VectorXDecisionVariable* vars,
    std::unordered_map<symbolic::Variable::Id, int>* map_var_to_index);

/*
 * Given an expression `e`, extracts all variables inside `e`.
 * @param[in] e A symbolic expression.
 * @retval pair pair.first is the variables in `e`. pair.second is the mapping
 * from the variable ID to the index in pair.first, such that
 * pair.second[pair.first(i).get_id()] = i
 */
std::pair<VectorXDecisionVariable,
          std::unordered_map<symbolic::Variable::Id, int>>
ExtractVariablesFromExpression(const symbolic::Expression& e);

/*
 * Given a quadratic polynomial @p poly, decomposes it into the form 0.5 * x' *
 * Q * x + b' * x + c
 *
 * @param[in] poly Quadratic polynomial to decompose.
 * @param[in] map_var_to_index maps variables in `poly.GetVariables()` to the
 * index in the vector `x`.
 * @param Q[out] The Hessian of the quadratic expression. @pre The size of Q
 * should be `num_variables * num_variables`.
 * @param b[out] The linear term of the quadratic expression. @pre The size of
 * `b` should be `num_variables * 1`.
 * @param c[out] The constant term of the quadratic expression.
 */
void DecomposeQuadraticPolynomial(
    const symbolic::Polynomial& poly,
    const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c);

/*
 * Given a vector of linear expressions v, decompose it to
 * \f$ v = A vars + b \f$
 * @param[in] v A vector of linear expressions
 * @param[out] A The matrix containing the linear coefficients.
 * @param[out] b The vector containing all the constant terms.
 * @param[out] vars All variables.
 */
void DecomposeLinearExpression(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    Eigen::MatrixXd* A, Eigen::VectorXd* b, VectorXDecisionVariable* vars);

/*
 * Decomposes a linear combination @p e = c0 + c1 * v1 + ... cn * vn into
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
DecomposeLinearExpression(
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

}  // namespace internal
}  // namespace solvers
}  // namespace drake
