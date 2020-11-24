#include "drake/common/symbolic_decompose.h"

#include <stdexcept>
#include <string>

namespace drake {
namespace symbolic {

using std::ostringstream;
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
  DRAKE_DEMAND(M != nullptr);
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
  DRAKE_DEMAND(M != nullptr && v != nullptr);
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

void ExtractAndAppendVariablesFromExpression(
    const Expression& e, VectorX<Variable>* vars,
    std::unordered_map<Variable::Id, int>* map_var_to_index) {
  DRAKE_DEMAND(static_cast<int>(map_var_to_index->size()) == vars->size());
  for (const Variable& var : e.GetVariables()) {
    if (map_var_to_index->find(var.get_id()) == map_var_to_index->end()) {
      map_var_to_index->emplace(var.get_id(), vars->size());
      const int vars_size = vars->size();
      vars->conservativeResize(vars_size + 1, Eigen::NoChange);
      (*vars)(vars_size) = var;
    }
  }
}

std::pair<VectorX<Variable>, std::unordered_map<Variable::Id, int>>
ExtractVariablesFromExpression(const Expression& e) {
  int var_count = 0;
  const symbolic::Variables var_set = e.GetVariables();
  VectorX<Variable> vars(var_set.size());
  std::unordered_map<Variable::Id, int> map_var_to_index{};
  map_var_to_index.reserve(var_set.size());
  for (const Variable& var : var_set) {
    map_var_to_index.emplace(var.get_id(), var_count);
    vars(var_count++) = var;
  }
  return make_pair(vars, map_var_to_index);
}

void DecomposeQuadraticPolynomial(
    const symbolic::Polynomial& poly,
    const std::unordered_map<Variable::Id, int>& map_var_to_index,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c) {
  const int num_variables = map_var_to_index.size();
  DRAKE_DEMAND(Q->rows() == num_variables);
  DRAKE_DEMAND(Q->cols() == num_variables);
  DRAKE_DEMAND(b->rows() == num_variables);
  Q->setZero();
  b->setZero();
  *c = 0;
  for (const auto& p : poly.monomial_to_coefficient_map()) {
    DRAKE_ASSERT(is_constant(p.second));
    DRAKE_DEMAND(!is_zero(p.second));
    const double coefficient = get_constant_value(p.second);
    const symbolic::Monomial& p_monomial = p.first;
    if (p_monomial.total_degree() > 2) {
      ostringstream oss;
      oss << p.first
          << " has order higher than 2 and it cannot be handled by "
             "DecomposeQuadraticPolynomial."
          << std::endl;
      throw runtime_error(oss.str());
    }
    const auto& monomial_powers = p_monomial.get_powers();
    if (monomial_powers.size() == 2) {
      // cross terms.
      auto it = monomial_powers.begin();
      const int x1_index = map_var_to_index.at(it->first.get_id());
      DRAKE_DEMAND(it->second == 1);
      ++it;
      const int x2_index = map_var_to_index.at(it->first.get_id());
      DRAKE_DEMAND(it->second == 1);
      (*Q)(x1_index, x2_index) += coefficient;
      (*Q)(x2_index, x1_index) = (*Q)(x1_index, x2_index);
    } else if (monomial_powers.size() == 1) {
      // Two cases
      // 1. quadratic term a*x^2
      // 2. linear term b*x
      auto it = monomial_powers.begin();
      DRAKE_DEMAND(it->second == 2 || it->second == 1);
      const int x_index = map_var_to_index.at(it->first.get_id());
      if (it->second == 2) {
        // quadratic term a * x^2
        (*Q)(x_index, x_index) += 2 * coefficient;
      } else if (it->second == 1) {
        // linear term b * x.
        (*b)(x_index) += coefficient;
      }
    } else {
      // constant term.
      *c += coefficient;
    }
  }
}

void DecomposeAffineExpressions(const Eigen::Ref<const VectorX<Expression>>& v,
                                Eigen::MatrixXd* A, Eigen::VectorXd* b,
                                VectorX<Variable>* vars) {
  // 0. Setup map_var_to_index and var_vec.
  std::unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.size(); ++i) {
    ExtractAndAppendVariablesFromExpression(v(i), vars, &map_var_to_index);
  }

  // 1. Construct decompose v as
  // v = A * vars + b
  *A = Eigen::MatrixXd::Zero(v.rows(), vars->rows());
  *b = Eigen::VectorXd::Zero(v.rows());
  for (int i{0}; i < v.size(); ++i) {
    const Expression& e_i{v(i)};
    DecomposeAffineExpression(e_i, map_var_to_index, A->row(i), b->data() + i);
  }
}
}  // namespace symbolic
}  // namespace drake
