#include "drake/common/symbolic_decompose.h"

#include <stdexcept>
#include <string>

namespace drake {
namespace symbolic {

using std::runtime_error;
using std::string;

namespace {
void ThrowError(const string& type, const string& expression) {
  throw runtime_error("While decomposing an expression, we detects that a " +
                      type + " expression: " + expression + ".");
}

// A helper function to implement DecomposeLinearExpressions and
// DecomposeAffineExpressions functions. It finds the coefficient of the
// monomial `m` in the `map` and fills `M(i)` with the value.  If the monomial
// `m` does not appear in `map`, it uses `0.0` instead. If the coefficient is
// not a constant, it throws a runtime_error.
template <typename Derived>
void FindCoefficientAndFill(const Polynomial::MapType& map, const Monomial& m,
                            const int i, const Eigen::MatrixBase<Derived>& M) {
  const auto it = map.find(m);
  // Here, we use const_cast hack. See
  // https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html for
  // details.
  Eigen::MatrixBase<Derived>& M_dummy =
      const_cast<Eigen::MatrixBase<Derived>&>(M);
  if (it != map.end()) {
    // m should have a constant coefficient.
    if (!is_constant(it->second)) {
      ThrowError("non-constant", it->second.to_string());
    }
    M_dummy(i) = get_constant_value(it->second);
  } else {
    M_dummy(i) = 0.0;
  }
}
}  // namespace

// TODO(soonho-tri): Refactor DecomposeAffineExpressions and
// DecomposeLinearExpressions to factor common parts out.
void DecomposeLinearExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M) {
  DRAKE_DEMAND(M->rows() == expressions.rows() && M->cols() == vars.rows());
  for (int i = 0; i < expressions.size(); ++i) {
    const Expression& e{expressions(i)};
    if (!e.is_polynomial()) {
      ThrowError("non-polynomial", e.to_string());  // e should be a polynomial.
    }
    const Polynomial p{e, Variables{vars}};
    if (p.TotalDegree() > 1) {
      ThrowError("non-linear", e.to_string());  // e should be linear.
    }
    const Polynomial::MapType& map{p.monomial_to_coefficient_map()};
    if (map.count(Monomial{}) > 0) {
      // e should not have a constant term.
      ThrowError("non-linear", e.to_string());
    }
    // Fill M(i, j).
    for (int j = 0; j < vars.size(); ++j) {
      FindCoefficientAndFill(map, Monomial{vars.coeff(j)}, j, M->row(i));
    }
  }
}

void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M, EigenPtr<Eigen::VectorXd> v) {
  DRAKE_DEMAND(M->rows() == expressions.rows() && M->cols() == vars.rows());
  DRAKE_DEMAND(v->rows() == expressions.rows());
  for (int i = 0; i < expressions.size(); ++i) {
    const Expression& e{expressions(i)};
    if (!e.is_polynomial()) {
      ThrowError("non-polynomial", e.to_string());  // e should be a polynomial.
    }
    const Polynomial p{e, Variables{vars}};
    if (p.TotalDegree() > 1) {
      ThrowError("non-linear", e.to_string());  // e should be linear.
    }
    const Polynomial::MapType& map{p.monomial_to_coefficient_map()};
    // Fill M(i, j).
    for (int j = 0; j < vars.size(); ++j) {
      FindCoefficientAndFill(map, Monomial{vars.coeff(j)}, j, M->row(i));
    }
    // Fill v(i).
    FindCoefficientAndFill(map, Monomial{}, i, *v);
  }
}
}  // namespace symbolic
}  // namespace drake
