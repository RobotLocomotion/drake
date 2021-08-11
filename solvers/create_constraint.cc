#include "drake/solvers/create_constraint.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "drake/common/symbolic.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/math/quadratic_form.h"

namespace drake {
namespace solvers {
namespace internal {

using std::find;
using std::isfinite;
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
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

Binding<Constraint> ParseConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  DRAKE_ASSERT(v.rows() == lb.rows() && v.rows() == ub.rows());

  if (!IsAffine(v)) {
    auto constraint = make_shared<ExpressionConstraint>(v, lb, ub);
    return CreateBinding(constraint, constraint->vars());
  }  // else, continue on to linear-specific version below.

  if ((ub - lb).isZero()) {
    return ParseLinearEqualityConstraint(v, lb);
  }

  // Setup map_var_to_index and var_vec.
  // such that map_var_to_index[var(i)] = i
  unordered_map<Variable::Id, int> map_var_to_index;
  VectorXDecisionVariable vars(0);
  for (int i = 0; i < v.size(); ++i) {
    symbolic::ExtractAndAppendVariablesFromExpression(v(i), &vars,
                                                      &map_var_to_index);
  }

  // Construct A, new_lb, new_ub. map_var_to_index is used here.
  Eigen::MatrixXd A{Eigen::MatrixXd::Zero(v.size(), vars.size())};
  Eigen::VectorXd new_lb{v.size()};
  Eigen::VectorXd new_ub{v.size()};
  // We will determine if lb <= v <= ub is a bounding box constraint, namely
  // x_lb <= x <= x_ub.
  bool is_v_bounding_box = true;
  for (int i = 0; i < v.size(); ++i) {
    double constant_term = 0;
    int num_vi_variables = symbolic::DecomposeAffineExpression(
        v(i), map_var_to_index, A.row(i), &constant_term);
    if (num_vi_variables == 0 &&
        !(lb(i) <= constant_term && constant_term <= ub(i))) {
      // Unsatisfiable constraint with no variables, such as 1 <= 0 <= 2
      throw std::runtime_error(
          fmt::format("Constraint {} <= {} <= {} is unsatisfiable but called "
                      "with ParseConstraint.",
                      lb(i), v(i).to_string(), ub(i)));

    } else {
      new_lb(i) = lb(i) - constant_term;
      new_ub(i) = ub(i) - constant_term;
      DRAKE_DEMAND(!std::isnan(new_lb(i)));
      DRAKE_DEMAND(!std::isnan(new_ub(i)));
      if (num_vi_variables != 1) {
        is_v_bounding_box = false;
      }
    }
  }
  if (is_v_bounding_box) {
    // If every lb(i) <= v(i) <= ub(i) is a bounding box constraint, then
    // formulate a bounding box constraint x_lb <= x <= x_ub
    VectorXDecisionVariable bounding_box_x(v.size());
    for (int i = 0; i < v.size(); ++i) {
      // v(i) is in the form of c * x
      double x_coeff = 0;
      for (const auto& x : v(i).GetVariables()) {
        const double coeff = A(i, map_var_to_index[x.get_id()]);
        if (coeff != 0) {
          x_coeff += coeff;
          bounding_box_x(i) = x;
        }
      }
      if (x_coeff > 0) {
        new_lb(i) /= x_coeff;
        new_ub(i) /= x_coeff;
      } else {
        const double lb_i = new_lb(i);
        new_lb(i) = new_ub(i) / x_coeff;
        new_ub(i) = lb_i / x_coeff;
      }
      DRAKE_DEMAND(!std::isnan(new_lb(i)));
      DRAKE_DEMAND(!std::isnan(new_ub(i)));
    }
    return CreateBinding(make_shared<BoundingBoxConstraint>(new_lb, new_ub),
                         bounding_box_x);
  }

  return CreateBinding(make_shared<LinearConstraint>(A, new_lb, new_ub), vars);
}

std::unique_ptr<Binding<Constraint>> MaybeParseLinearConstraint(
    const symbolic::Expression& e, double lb, double ub) {
  if (!e.is_polynomial()) {
    return std::unique_ptr<Binding<Constraint>>{nullptr};
  }
  const Polynomial p{e};
  if (p.TotalDegree() > 1) {
    return std::unique_ptr<Binding<Constraint>>{nullptr};
  }
  // If p only has one indeterminates, then we can always return a bounding box
  // constraint.
  if (p.indeterminates().size() == 1) {
    // We decompose the polynomial `p` into `constant_term + coeff * var`.
    double coeff = 0;
    double constant_term = 0;
    for (const auto& term : p.monomial_to_coefficient_map()) {
      if (term.first.total_degree() == 0) {
        constant_term += get_constant_value(term.second);
      } else {
        coeff += get_constant_value(term.second);
      }
    }
    // coeff should not be 0. The symbolic polynomial should be able to detect
    // when the coefficient is 0, and remove it from
    // monomial_to_coefficient_map.
    DRAKE_DEMAND(coeff != 0);
    double var_lower{}, var_upper{};
    if (coeff > 0) {
      var_lower = (lb - constant_term) / coeff;
      var_upper = (ub - constant_term) / coeff;
    } else {
      var_lower = (ub - constant_term) / coeff;
      var_upper = (lb - constant_term) / coeff;
    }
    return std::make_unique<Binding<Constraint>>(
        std::make_shared<BoundingBoxConstraint>(Vector1d(var_lower),
                                                Vector1d(var_upper)),
        Vector1<symbolic::Variable>(*(p.indeterminates().begin())));
  }
  VectorX<symbolic::Variable> bound_variables(p.indeterminates().size());
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  int index = 0;
  for (const auto& var : p.indeterminates()) {
    bound_variables(index) = var;
    map_var_to_index.emplace(var.get_id(), index++);
  }
  Eigen::RowVectorXd a(p.indeterminates().size());
  a.setZero();
  double lower = lb;
  double upper = ub;
  for (const auto& term : p.monomial_to_coefficient_map()) {
    if (term.first.total_degree() == 0) {
      const double coeff = get_constant_value(term.second);
      lower -= coeff;
      upper -= coeff;
    } else {
      const int var_index =
          map_var_to_index.at(term.first.GetVariables().begin()->get_id());
      a(var_index) = get_constant_value(term.second);
    }
  }
  if (lower == upper) {
    return std::make_unique<Binding<Constraint>>(
        std::make_shared<LinearEqualityConstraint>(a, Vector1d(lower)),
        bound_variables);
  } else {
    return std::make_unique<Binding<Constraint>>(
        std::make_shared<LinearConstraint>(a, Vector1d(lower), Vector1d(upper)),
        bound_variables);
  }
}

namespace {
// Given two symbolic expressions, e1 and e2, finds an equi-satisfiable
// constraint `e <= c` for `e1 <= e2`. First, it decomposes e1 and e2 into `e1 =
// c1 + e1'` and `e2 = c2 + e2'`. Then it does the following case analysis.
//
// Case 1: If c1 or c2 are finite, we use the following derivations:
//
//              e1 <= e2
//   ->   c1 + e1' <= c2 + e2'
//   ->  e1' - e2' <= c2 - c1.
//
// and set e := e1' - e2' and c := c2 - c1.
//
// Case 2: If both c1 and c2 are infinite. We use the following table
//
//        c1    c2
//     --------------------------
//        +∞ <= +∞     Trivially holds.
//        +∞ <= -∞     Infeasible.
//        -∞ <= +∞     Trivially holds.
//        -∞ <= -∞     Trivially holds.
//
//  and throw an exception for all the cases.
//
// Note that c1 (resp. c2) can be infinite only if e1 (resp. e2) is zero.
// Otherwise, it throws an exception. To understand this side-condition,
// consider the following example:
//
//     e1 = 0
//     e2 = x + ∞
//
//     e1 <= e2   :=    0 <= x + ∞    -- (1)
//
// Without the side-condition, we might derive the following (wrong)
// equi-satisfiable constraint:
//
//     -x <= ∞                        -- (2)
//
// This is problematic because x ↦ -∞ is a satisfying constraint of
// (2) but it's not for (1) since we have:
//
//     0 <= -∞ + ∞
//     0 <= nan
//     False.
//
void FindBound(const Expression& e1, const Expression& e2, Expression* const e,
               double* const c) {
  DRAKE_ASSERT(e != nullptr);
  DRAKE_ASSERT(c != nullptr);
  double c1 = 0;
  double c2 = 0;
  const Expression e1_expanded{e1.Expand()};
  if (is_constant(e1_expanded)) {
    c1 = get_constant_value(e1_expanded);
  } else if (is_addition(e1_expanded)) {
    c1 = get_constant_in_addition(e1_expanded);
    if (!isfinite(c1)) {
      ostringstream oss;
      oss << "FindBound() cannot handle the constraint: " << e1 << " <= " << e2
          << " because " << e1
          << " has infinity in the constant term after expansion.";
      throw runtime_error{oss.str()};
    }
    *e = Expression::Zero();
    for (const auto& p : get_expr_to_coeff_map_in_addition(e1_expanded)) {
      *e += p.first * p.second;
    }
  } else {
    *e = e1_expanded;
  }
  const Expression e2_expanded{e2.Expand()};
  if (is_constant(e2_expanded)) {
    c2 = get_constant_value(e2_expanded);
  } else if (is_addition(e2_expanded)) {
    c2 = get_constant_in_addition(e2_expanded);
    if (!isfinite(c2)) {
      ostringstream oss;
      oss << "FindBound() cannot handle the constraint: " << e1 << " <= " << e2
          << " because " << e2
          << " has infinity in the constant term after expansion.";
      throw runtime_error{oss.str()};
    }
    for (const auto& p : get_expr_to_coeff_map_in_addition(e2_expanded)) {
      *e -= p.first * p.second;
    }
  } else {
    *e -= e2_expanded;
  }
  if (isfinite(c1) || isfinite(c2)) {
    *c = c2 - c1;
    return;
  }
  // Handle special cases where both of `c1` and `c2` are infinite.
  //    c1    c2
  // --------------------------
  //    +∞ <= +∞     Trivially holds.
  //    +∞ <= -∞     Infeasible.
  //    -∞ <= +∞     Trivially holds.
  //    -∞ <= -∞     Trivially holds.
  ostringstream oss;
  if (c1 == numeric_limits<double>::infinity() &&
      c2 == -numeric_limits<double>::infinity()) {
    oss << "FindBound() detects an infeasible constraint: " << e1
        << " <= " << e2 << ".";
    throw runtime_error{oss.str()};
  } else {
    oss << "FindBound() detects a trivial constraint: " << e1 << " <= " << e2
        << ".";
    throw runtime_error{oss.str()};
  }
}
}  // namespace

Binding<Constraint> ParseConstraint(const set<Formula>& formulas) {
  const auto n = formulas.size();

  // Decomposes a set of formulas into a 1D-vector of expressions, `v`, and two
  // 1D-vector of double `lb` and `ub`.
  VectorX<Expression> v{n};
  Eigen::VectorXd lb{n};
  Eigen::VectorXd ub{n};
  int i{0};  // index variable used in the loop
  // After the following loop, we call `ParseLinearEqualityConstraint`
  // if `are_all_formulas_equal` is still true. Otherwise, we call
  // `ParseLinearConstraint`.  on the value of this Boolean flag.
  bool are_all_formulas_equal{true};
  bool is_linear{true};
  for (const Formula& f : formulas) {
    if (is_equal_to(f)) {
      // f := (lhs == rhs)
      //      (lhs - rhs == 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = 0.0;
      ub(i) = 0.0;
    } else if (is_less_than_or_equal_to(f)) {
      // f := (lhs <= rhs)
      const Expression& lhs = get_lhs_expression(f);
      const Expression& rhs = get_rhs_expression(f);
      lb(i) = -numeric_limits<double>::infinity();
      FindBound(lhs, rhs, &v(i), &ub(i));
      are_all_formulas_equal = false;
    } else if (is_greater_than_or_equal_to(f)) {
      // f := (lhs >= rhs)
      const Expression& lhs = get_lhs_expression(f);
      const Expression& rhs = get_rhs_expression(f);
      lb(i) = -numeric_limits<double>::infinity();
      FindBound(rhs, lhs, &v(i), &ub(i));
      are_all_formulas_equal = false;
    } else {
      ostringstream oss;
      oss << "ParseConstraint(const set<Formula>& "
          << "formulas) is called while its argument 'formulas' includes "
          << "a formula " << f
          << " which is not a relational formula using one of {==, <=, >=} "
          << "operators.";
      throw runtime_error(oss.str());
    }

    // Check that elements are linear.
    if (is_linear) {
      if (!v(i).is_polynomial()) {
        is_linear = false;
      } else {
        const Polynomial p{v(i)};
        if (p.TotalDegree() > 1) {
          is_linear = false;
        }
      }
    }
    ++i;
  }
  if (are_all_formulas_equal && is_linear) {
    return ParseLinearEqualityConstraint(v, lb);
  } else {
    return ParseConstraint(v, lb, ub);
  }
}

Binding<Constraint> ParseConstraint(const Formula& f) {
  if (is_equal_to(f)) {
    // e1 == e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseConstraint(e1 - e2, 0.0, 0.0);
  } else if (is_greater_than_or_equal_to(f)) {
    // e1 >= e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    Expression e;
    double ub = 0.0;
    FindBound(e2, e1, &e, &ub);
    return ParseConstraint(e, -numeric_limits<double>::infinity(), ub);
  } else if (is_less_than_or_equal_to(f)) {
    // e1 <= e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    Expression e;
    double ub = 0.0;
    FindBound(e1, e2, &e, &ub);
    return ParseConstraint(e, -numeric_limits<double>::infinity(), ub);
  }
  if (is_conjunction(f)) {
    return ParseConstraint(get_operands(f));
  }
  ostringstream oss;
  oss << "ParseConstraint is called with a formula " << f
      << " which is neither a relational formula using one of {==, <=, >=} "
         "operators nor a conjunction of those relational formulas.";
  throw runtime_error(oss.str());
}

Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const set<Formula>& formulas) {
  const auto n = formulas.size();
  // Decomposes a set of formulas, `{e₁₁ == e₁₂, ..., eₙ₁ == eₙ₂}`
  // into a 1D-vector of expressions, `v = [e₁₁ - e₁₂, ..., eₙ₁ - eₙ₂]`.
  VectorX<symbolic::Expression> v{n};
  int i{0};  // index variable used in the loop
  for (const symbolic::Formula& f : formulas) {
    if (is_equal_to(f)) {
      // f := (lhs == rhs)
      //      (lhs - rhs == 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
    } else {
      ostringstream oss;
      oss << "ParseLinearEqualityConstraint(const "
          << "set<Formula>& formulas) is called while its argument 'formulas' "
          << "includes a non-equality formula " << f << ".";
      throw runtime_error(oss.str());
    }
    ++i;
  }
  return ParseLinearEqualityConstraint(v, Eigen::VectorXd::Zero(n));
}

Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const Formula& f) {
  if (is_equal_to(f)) {
    // e1 == e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearEqualityConstraint(e1 - e2, 0.0);
  }
  if (is_conjunction(f)) {
    return ParseLinearEqualityConstraint(get_operands(f));
  }
  ostringstream oss;
  oss << "ParseLinearConstraint is called with a formula " << f
      << " which is neither an equality formula nor a conjunction of equality "
         "formulas.";
  throw runtime_error(oss.str());
}

Binding<LinearEqualityConstraint> DoParseLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  DRAKE_DEMAND(v.rows() == b.rows());
  VectorXDecisionVariable vars(0);
  unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.rows(); ++i) {
    symbolic::ExtractAndAppendVariablesFromExpression(v(i), &vars,
                                                      &map_var_to_index);
  }
  // TODO(hongkai.dai): use sparse matrix.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(v.rows(), vars.rows());
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(v.rows());
  for (int i = 0; i < v.rows(); ++i) {
    double constant_term(0);
    symbolic::DecomposeAffineExpression(v(i), map_var_to_index, A.row(i),
                                        &constant_term);
    beq(i) = b(i) - constant_term;
  }
  return CreateBinding(make_shared<LinearEqualityConstraint>(A, beq), vars);
}

Binding<QuadraticConstraint> ParseQuadraticConstraint(
    const symbolic::Expression& e, double lower_bound, double upper_bound) {
  // First build an Eigen vector that contains all the bound variables.
  auto p = symbolic::ExtractVariablesFromExpression(e);
  const auto& vars_vec = p.first;
  const auto& map_var_to_index = p.second;

  // Now decompose the expression into coefficients and monomials.
  const symbolic::Polynomial poly{e};

  Eigen::MatrixXd Q(vars_vec.size(), vars_vec.size());
  Eigen::VectorXd b(vars_vec.size());
  double constant_term;
  // Decompose the polynomial as 0.5xᵀQx + bᵀx + k.
  symbolic::DecomposeQuadraticPolynomial(poly, map_var_to_index, &Q, &b,
                                         &constant_term);
  // The constraint to be imposed is
  // lb - k ≤ 0.5 xᵀQx + bᵀx ≤ ub - k
  return CreateBinding(
      make_shared<QuadraticConstraint>(Q, b, lower_bound - constant_term,
                                       upper_bound - constant_term),
      vars_vec);
}

shared_ptr<Constraint> MakePolynomialConstraint(
    const VectorXPoly& polynomials,
    const vector<Polynomiald::VarType>& poly_vars, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub) {
  // Polynomials that are actually affine (a sum of linear terms + a
  // constant) can be special-cased.  Other polynomials are treated as
  // generic for now.
  // TODO(ggould-tri) There may be other such special easy cases.
  bool all_affine = true;
  for (int i = 0; i < polynomials.rows(); i++) {
    if (!polynomials[i].IsAffine()) {
      all_affine = false;
      break;
    }
  }
  if (all_affine) {
    Eigen::MatrixXd linear_constraint_matrix =
        Eigen::MatrixXd::Zero(polynomials.rows(), poly_vars.size());
    Eigen::VectorXd linear_constraint_lb = lb;
    Eigen::VectorXd linear_constraint_ub = ub;
    for (int poly_num = 0; poly_num < polynomials.rows(); poly_num++) {
      for (const auto& monomial : polynomials[poly_num].GetMonomials()) {
        if (monomial.terms.size() == 0) {
          linear_constraint_lb[poly_num] -= monomial.coefficient;
          linear_constraint_ub[poly_num] -= monomial.coefficient;
        } else {
          DRAKE_DEMAND(monomial.terms.size() == 1);  // Because isAffine().
          const Polynomiald::VarType term_var = monomial.terms[0].var;
          int var_num = (find(poly_vars.begin(), poly_vars.end(), term_var) -
                         poly_vars.begin());
          DRAKE_ASSERT(var_num < static_cast<int>(poly_vars.size()));
          linear_constraint_matrix(poly_num, var_num) = monomial.coefficient;
        }
      }
    }
    if (ub == lb) {
      return make_shared<LinearEqualityConstraint>(linear_constraint_matrix,
                                                   linear_constraint_ub);
    } else {
      return make_shared<LinearConstraint>(
          linear_constraint_matrix, linear_constraint_lb, linear_constraint_ub);
    }
  } else {
    return make_shared<PolynomialConstraint>(polynomials, poly_vars, lb, ub);
  }
}

Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    LorentzConeConstraint::EvalType eval_type) {
  DRAKE_DEMAND(v.rows() >= 2);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  symbolic::DecomposeAffineExpressions(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  return CreateBinding(make_shared<LorentzConeConstraint>(A, b, eval_type),
                       vars);
}

Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const Expression& linear_expr, const Expression& quadratic_expr, double tol,
    LorentzConeConstraint::EvalType eval_type) {
  const auto& quadratic_p =
      symbolic::ExtractVariablesFromExpression(quadratic_expr);
  const auto& quadratic_vars = quadratic_p.first;
  const auto& quadratic_var_to_index_map = quadratic_p.second;
  const symbolic::Polynomial poly{quadratic_expr};
  Eigen::MatrixXd Q(quadratic_vars.size(), quadratic_vars.size());
  Eigen::VectorXd b(quadratic_vars.size());
  double a;
  symbolic::DecomposeQuadraticPolynomial(poly, quadratic_var_to_index_map, &Q,
                                         &b, &a);
  // The constraint that the linear expression v1 satisfying
  // v1 >= sqrt(0.5 * x' * Q * x + b' * x + a), is equivalent to the vector
  // [z; y] being within a Lorentz cone, where
  // z = v1
  // y = C * x + d
  // such that yᵀy = 0.5xᵀQx + bᵀx + a

  VectorX<Expression> expr{};
  Eigen::MatrixXd C;
  Eigen::VectorXd d;
  std::tie(C, d) = math::DecomposePositiveQuadraticForm(0.5 * Q, b, a, tol);
  expr.resize(1 + C.rows());
  // expr(0) is z
  expr(0) = linear_expr;
  // expr.segment(1, C.rows()) = y
  expr.segment(1, C.rows()) = C * quadratic_vars + d;
  return ParseLorentzConeConstraint(expr, eval_type);
}

Binding<RotatedLorentzConeConstraint> ParseRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 3);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  symbolic::DecomposeAffineExpressions(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  return CreateBinding(std::make_shared<RotatedLorentzConeConstraint>(A, b),
                       vars);
}

Binding<RotatedLorentzConeConstraint> ParseRotatedLorentzConeConstraint(
    const symbolic::Expression& linear_expr1,
    const symbolic::Expression& linear_expr2,
    const symbolic::Expression& quadratic_expr, double tol) {
  const auto& quadratic_p =
      symbolic::ExtractVariablesFromExpression(quadratic_expr);
  const auto& quadratic_vars = quadratic_p.first;
  const auto& quadratic_var_to_index_map = quadratic_p.second;
  const symbolic::Polynomial poly{quadratic_expr};
  Eigen::MatrixXd Q(quadratic_vars.size(), quadratic_vars.size());
  Eigen::VectorXd b(quadratic_vars.size());
  double a;
  symbolic::DecomposeQuadraticPolynomial(poly, quadratic_var_to_index_map, &Q,
                                         &b, &a);

  Eigen::MatrixXd C;
  Eigen::VectorXd d;
  std::tie(C, d) = math::DecomposePositiveQuadraticForm(0.5 * Q, b, a, tol);
  VectorX<symbolic::Expression> expr(2 + C.rows());
  expr(0) = linear_expr1;
  expr(1) = linear_expr2;
  expr.tail(C.rows()) = C * quadratic_vars + d;
  return ParseRotatedLorentzConeConstraint(expr);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
