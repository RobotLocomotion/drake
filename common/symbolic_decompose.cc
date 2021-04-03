#include "drake/common/symbolic_decompose.h"

#include <map>
#include <stdexcept>
#include <string>
#include <tuple>

#include <fmt/ostream.h>

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

namespace {

typedef std::tuple<VectorX<Expression>, VectorX<Expression>, Expression>
    LumpedFactorization;

// Visitor class to implement DecomposeLumpedParameters.
class DecomposeLumpedParametersVisitor {
 public:
  LumpedFactorization Decompose(const Expression& e,
                                const Variables& parameters) const {
    // Note that it calls `Expression::Expand()` here.
    return Visit(e.Expand(), parameters);
  }

 private:
  LumpedFactorization Visit(const Expression& e,
                            const Variables& parameters) const {
    return VisitExpression<LumpedFactorization>(this, e, parameters);
  }

  LumpedFactorization VisitVariable(const Expression& e,
                                    const Variables& parameters) const {
    const Variable& var{get_variable(e)};
    if (parameters.include(var)) {
      // W = [1], alpha = [e], w0 = [0]
      return LumpedFactorization{Vector1<Expression>{1}, Vector1<Expression>{e},
                                 0};
    } else {
      // W = [], alpha = [], w0 = [e]
      return LumpedFactorization{Vector0<Expression>{}, Vector0<Expression>{},
                                 e};
    }
  }

  LumpedFactorization VisitConstant(const Expression& e,
                                    const Variables&) const {
    return LumpedFactorization{Vector0<Expression>{}, Vector0<Expression>{}, e};
  }

  LumpedFactorization VisitAddition(const Expression& e,
                                    const Variables& parameters) const {
    // Temporary storage to hold the elements of w(n) (as a key) and the
    // elements of α(parameters) (as a value) for e.  We use a map to avoid
    // duplicates.
    std::map<Expression, Expression> w_map;

    // e = c₀ + ∑ᵢ (cᵢ * eᵢ)
    //   => [c₁w₁, c₂w₂, ...]*[α₁, α₂, ...] + (c₀ + ∑ᵢ cᵢw0ᵢ)
    // except for matching terms.
    Expression w0 = get_constant_in_addition(e);
    for (const std::pair<const Expression, double>& p :
         get_expr_to_coeff_map_in_addition(e)) {
      const Expression& e_i{p.first};
      const double c_i{p.second};
      const auto [w_i, alpha_i, w0_i] = Visit(e_i, parameters);
      w0 += c_i * w0_i;
      // TODO(russt): generalize this to matching up to a constant factor.
      for (int j = 0; j < w_i.size(); j++) {
        auto it = w_map.emplace(c_i * w_i[j], 0).first;
        it->second += alpha_i[j];
      }
    }
    VectorX<Expression> w(w_map.size());
    VectorX<Expression> alpha(w_map.size());
    int i = 0;
    for (const auto& [key, value] : w_map) {
      w[i] = key;
      alpha[i++] = value;
    }
    return LumpedFactorization{w, alpha, w0};
  }

  // Handle basic multiplication: e = a * b
  LumpedFactorization SimpleMultiplication(const LumpedFactorization& a,
                                           const LumpedFactorization& b) const {
    const auto& [w_a, alpha_a, w0_a] = a;
    const auto& [w_b, alpha_b, w0_b] = b;

    // Avoid adding terms with zero coefficients, otherwise they start to
    // accumulate quickly.
    const bool nonzero_w0a = !is_zero(w0_a);
    const bool nonzero_w0b = !is_zero(w0_b);

    // a*b = (wa*αa + w₀a) (wb*αb + w₀b)
    //     = w₀a*w₀b + ∑ᵢⱼ(waᵢ*wbⱼ * αaᵢ*αbⱼ) + ∑ⱼw₀a*wbⱼ*αbⱼ + ∑ᵢw₀b*waᵢ*αaᵢ
    const auto N = w_a.size() * w_b.size() + (nonzero_w0a ? w_b.size() : 0) +
                   (nonzero_w0b ? w_a.size() : 0);
    VectorX<Expression> w(N);
    VectorX<Expression> alpha(N);
    const Expression w0 = w0_a * w0_b;
    if (w_a.size() * w_b.size() != 0) {
      w.head(w_a.size() * w_b.size()) << w_a * w_b.transpose();
      alpha.head(w_a.size() * w_b.size()) << alpha_a * alpha_b.transpose();
    }
    if (nonzero_w0a) {
      const auto offset = w_a.size() * w_b.size();
      w.segment(offset, w_b.size()) = w0_a * w_b;
      alpha.segment(offset, w_b.size()) = alpha_b;
    }
    if (nonzero_w0b) {
      w.tail(w_a.size()) = w0_b * w_a;
      alpha.tail(w_a.size()) = alpha_a;
    }
    // TODO(russt): Avoid duplicates.
    return LumpedFactorization(w, alpha, w0);
  }

  LumpedFactorization VisitMultiplication(const Expression& e,
                                          const Variables& parameters) const {
    const double c = get_constant_in_multiplication(e);
    LumpedFactorization f({}, {}, {c});

    // e = c * ∏ᵢ pow(baseᵢ, exponentᵢ).
    for (const std::pair<const Expression, Expression>& p :
         get_base_to_exponent_map_in_multiplication(e)) {
      const Expression& base_i{p.first};
      const Expression& exponent_i{p.second};
      const auto [w, alpha, w0] = SimpleMultiplication(
          f, is_one(exponent_i)
                 ? Visit(base_i, parameters)
                 : VisitPow(pow(base_i, exponent_i), parameters));
      // Watch out for aliasing (do I need .eval() in this case?).
      f = LumpedFactorization{w.eval(), alpha.eval(), w0};
    }

    return f;
  }

  LumpedFactorization VisitPow(const Expression& e,
                               const Variables& parameters) const {
    const Expression& exponent{get_second_argument(e)};
    const Variables vars = e.GetVariables();
    if (vars.IsSubsetOf(parameters)) {  // All parameters.
      return LumpedFactorization{
          Vector1<Expression>{1}, Vector1<Expression>{e}, {0}};
    } else if (intersect(vars, parameters).empty()) {  // All non-parameters.
      return LumpedFactorization{{}, {}, e};
    } else if (is_constant(exponent)) {
      // Note(russt): I don't *think* that this code is reachable, since the
      // Expand() called at the beginning of the decomposition will break apart
      // cases like this.  But we can implement this if we ever determine it is
      // needed, e.g., repeated calls to SimpleMultiplication.
      throw runtime_error(
          fmt::format("{} CAN be factored into lumped parameters, but this "
                      "case has not been implemented yet.",
                      e));
    } else {
      throw runtime_error(
          fmt::format("{} cannot be factored into lumped parameters, since it "
                      "depends on both parameters and non-parameter variables "
                      "in a non-multiplicative way.",
                      e));
    }
  }

  LumpedFactorization VisitNonPolynomialTerm(
      const Expression& e, const Variables& parameters) const {
    // Must be either all parameters or all non-parameters.
    const Variables& vars = e.GetVariables();
    if (vars.IsSubsetOf(parameters)) {
      return LumpedFactorization{
          Vector1<Expression>{1}, Vector1<Expression>{e}, {0}};
    } else if (intersect(vars, parameters).empty()) {
      return LumpedFactorization{{}, {}, e};
    } else {
      throw runtime_error(
          fmt::format("{} cannot be factored into lumped parameters, since it "
                      "depends on both parameters and non-parameter variables.",
                      e));
    }
  }

  LumpedFactorization VisitDivision(const Expression& e,
                                    const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }

  LumpedFactorization VisitAbs(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitLog(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitExp(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitSqrt(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitSin(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitCos(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitTan(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitAsin(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitAcos(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitAtan(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitAtan2(const Expression& e,
                                 const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitSinh(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitCosh(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitTanh(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitMin(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitMax(const Expression& e,
                               const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitCeil(const Expression& e,
                                const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitFloor(const Expression& e,
                                 const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitIfThenElse(const Expression& e,
                                      const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }
  LumpedFactorization VisitUninterpretedFunction(
      const Expression& e, const Variables& parameters) const {
    return VisitNonPolynomialTerm(e, parameters);
  }

  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend LumpedFactorization drake::symbolic::VisitExpression<
      LumpedFactorization>(const DecomposeLumpedParametersVisitor*,
                           const Expression&, const Variables&);
};

}  // namespace

std::tuple<MatrixX<Expression>, VectorX<Expression>, VectorX<Expression>>
DecomposeLumpedParameters(
    const Eigen::Ref<const VectorX<Expression>>& f,
    const Eigen::Ref<const VectorX<Variable>>& parameters) {
  const DecomposeLumpedParametersVisitor visitor{};

  // Compute Wα (avoiding duplicate α) by filling a map from alpha to the
  // corresponding column of W.
  std::map<Expression, VectorX<Expression>> alpha_map;

  VectorX<Expression> w0(f.size());
  for (int i = 0; i < f.size(); i++) {
    auto const[w, alpha, this_w0] =
        visitor.Decompose(f[i], Variables(parameters));
    w0[i] = this_w0;
    for (int j = 0; j < alpha.size(); j++) {
      auto it = alpha_map.emplace(alpha[j], VectorX<Expression>::Zero(f.size()))
                    .first;
      (it->second)[i] += w[j];  // add to element i of column j.
    }
  }
  MatrixX<Expression> W = MatrixX<Expression>::Zero(f.size(), alpha_map.size());
  VectorX<Expression> alpha(alpha_map.size());
  int j = 0;
  for (const auto& [key, value] : alpha_map) {
    alpha[j] = key;
    W.col(j++) = value;
  }
  return {W, alpha, w0};
}

}  // namespace symbolic
}  // namespace drake
