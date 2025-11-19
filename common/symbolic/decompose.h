#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"

// TODO(soonho-tri): Migrate the functions in drake/solvers:symbolic_extract to
// this file.

namespace drake {
namespace symbolic {

/** Checks if every element in `m` is affine in `vars`.
@note If `m` is an empty matrix, it returns true. */
bool IsAffine(const Eigen::Ref<const MatrixX<Expression>>& m,
              const Variables& vars);

/** Checks if every element in `m` is affine.
@note If `m` is an empty matrix, it returns true. */
bool IsAffine(const Eigen::Ref<const MatrixX<Expression>>& m);

/** Decomposes @p expressions into @p M * @p vars.

@throws std::exception if @p expressions is not linear in @p vars.
@pre M.rows() == expressions.rows() && M.cols() == vars.rows(). */
void DecomposeLinearExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M);

/** Decomposes @p expressions into @p M * @p vars + @p v.

@throws std::exception if @p expressions is not affine in @p vars.
@pre M.rows() == expressions.rows() && M.cols() == vars.rows().
@pre v.rows() == expressions.rows(). */
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M, EigenPtr<Eigen::VectorXd> v);

/** Given an expression `e`, extracts all variables inside `e`, appends these
variables to `vars` if they are not included in `vars` yet.

@param[in] e  A symbolic expression.
@param[in,out] vars  As an input, `vars` contain the variables before
extracting expression `e`. As an output, the variables in `e` that were not
included in `vars`, will be appended to the end of `vars`.
@param[in,out] map_var_to_index is of the same size as
`vars`, and map_var_to_index[vars(i).get_id()] = i. This invariance holds for
map_var_to_index both as the input and as the output.
*/
void ExtractAndAppendVariablesFromExpression(
    const symbolic::Expression& e, std::vector<Variable>* vars,
    std::unordered_map<symbolic::Variable::Id, int>* map_var_to_index);

/** Given an expression `e`, extracts all variables inside `e`.

@param[in] e A symbolic expression. @retval pair pair.first is the variables in
`e`. pair.second is the mapping from the variable ID to the index in
pair.first, such that pair.second[pair.first(i).get_id()] = i */
std::pair<VectorX<Variable>, std::unordered_map<symbolic::Variable::Id, int>>
ExtractVariablesFromExpression(const symbolic::Expression& e);

/**
 * Overloads ExtractVariablesFromExpression but with a vector of expressions.
 */
std::pair<VectorX<Variable>, std::unordered_map<Variable::Id, int>>
ExtractVariablesFromExpression(
    const Eigen::Ref<const VectorX<Expression>>& expressions);

/** Given a quadratic polynomial @p poly, decomposes it into the form 0.5 * x'
* Q * x + b' * x + c

@param[in] poly Quadratic polynomial to decompose.
@param[in] map_var_to_index maps variables in `poly.GetVariables()` to the
index in the vector `x`.
@param[out] Q The Hessian of the quadratic expression. @pre The size of Q
should be `num_variables x num_variables`. Q is a symmetric matrix.
@param[out] b linear term of the quadratic expression. @pre The size of `b`
should be `num_variables`.
@param[out] c The constant term of the quadratic expression. */
void DecomposeQuadraticPolynomial(
    const symbolic::Polynomial& poly,
    const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c);

/** Given a vector of affine expressions v, decompose it to \f$ v = A vars + b
\f$

@param[in] v A vector of affine expressions
@param[out] A The matrix containing the linear coefficients.
@param[out] b The vector containing all the constant terms.
@param[out] vars All variables.

@throws std::exception if the input expressions are not affine.
*/
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    Eigen::MatrixXd* A, Eigen::VectorXd* b, VectorX<Variable>* vars);

/** Decomposes an affine combination @p e = c0 + c1 * v1 + ... cn * vn into the
following:

     constant term      : c0
     coefficient vector : [c1, ..., cn]
     variable vector    : [v1, ..., vn]

Then, it extracts the coefficient and the constant term. A map from variable ID
to int, @p map_var_to_index, is used to decide a variable's index in a linear
combination.

\pre
1. @c coeffs is a row vector of double, whose length matches with the size of
   @c map_var_to_index.
2. e.is_polynomial() is true.
3. e is an affine expression.
4. all values in `map_var_to_index` should be in the range [0,
map_var_to_index.size())

@param[in] e The symbolic affine expression
@param[in] map_var_to_index A mapping from variable ID to variable index, such
that map_var_to_index[vi.get_ID()] = i.
@param[out] coeffs A row vector. coeffs(i) = ci.
@param[out] constant_term c0 in the equation above.
@return num_variable. Number of variables in the expression. 2 * x(0) + 3 has 1
variable; 2 * x(0) + 3 * x(1) - 2 * x(0) has 1 variable, since the x(0) term
cancels.

@throws std::exception if the input expression is not affine.
*/
int DecomposeAffineExpression(
    const symbolic::Expression& e,
    const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
    EigenPtr<Eigen::RowVectorXd> coeffs, double* constant_term);

/** Given a vector of Expressions @p f and a list of @p parameters we define
all additional variables in @p f to be a vector of "non-parameter variables",
n.  This method returns a factorization of @p f into an equivalent "data
matrix", W, which depends only on the non-parameter variables, and a "lumped
parameter vector", α, which depends only on @p parameters: f =
W(n)*α(parameters) + w0(n).

@note The current implementation makes some simple attempts to minimize the
number of lumped parameters, but more simplification could be implemented
relatively easily.  Optimal simplification, however, involves the complexity of
comparing two arbitrary Expressions (see Expression::EqualTo for more details).

@throw std::exception if @p f is not decomposable in this way (cells containing
@p parameters may only be added or multiplied with cells containing
non-parameter variables).

@returns W(n), α(parameters), and w0(n). */
std::tuple<MatrixX<Expression>, VectorX<Expression>, VectorX<Expression>>
DecomposeLumpedParameters(
    const Eigen::Ref<const VectorX<Expression>>& f,
    const Eigen::Ref<const VectorX<Variable>>& parameters);

/** Decomposes an L2 norm @p e = |Ax+b|₂ into A, b, and the variable vector x
(or returns false if the decomposition is not possible).

In order for the decomposition to succeed, the following conditions must be met:
1. e is a sqrt expression.
2. e.get_argument() is a polynomial of degree 2, which can be expressed as a
   quadratic form (Ax+b)ᵀ(Ax+b).

@param e The symbolic affine expression
@param psd_tol The tolerance for checking positive semidefiniteness.
Eigenvalues less that this threshold are considered to be zero. Matrices with
negative eigenvalues less than this threshold are considered to be not positive
semidefinite, and will cause the decomposition to fail.
@param coefficient_tol The absolute tolerance for checking that the
coefficients of the expression inside the sqrt match the coefficients of
|Ax+b|₂².

@return [is_l2norm, A, b, vars] where is_l2norm is true iff the decomposition
was successful, and if is_l2norm is true then |A*vars + b|₂ = e.
*/
std::tuple<bool, Eigen::MatrixXd, Eigen::VectorXd, VectorX<Variable>>
DecomposeL2NormExpression(const symbolic::Expression& e, double psd_tol = 1e-8,
                          double coefficient_tol = 1e-8);

}  // namespace symbolic
}  // namespace drake
