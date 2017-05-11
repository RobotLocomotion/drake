#include "drake/solvers/create_constraint.h"

#include <algorithm>
#include <sstream>

#include "drake/solvers/symbolic_extraction.h"

namespace drake {
namespace solvers {
namespace internal {

using std::find;
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

using internal::DecomposeLinearExpression;
using internal::DecomposeQuadraticExpressionWithMonomialToCoeffMap;
using internal::ExtractAndAppendVariablesFromExpression;
using internal::ExtractVariablesFromExpression;
using internal::SymbolicError;

Binding<LinearConstraint> ParseLinearConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  DRAKE_ASSERT(v.rows() == lb.rows() && v.rows() == ub.rows());

  // Setup map_var_to_index and var_vec.
  // such that map_var_to_index[var(i)] = i
  unordered_map<Variable::Id, int> map_var_to_index;
  VectorXDecisionVariable vars(0);
  for (int i = 0; i < v.size(); ++i) {
    ExtractAndAppendVariablesFromExpression(v(i), &vars, &map_var_to_index);
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
    int num_vi_variables = DecomposeLinearExpression(v(i), map_var_to_index,
                                                     A.row(i), &constant_term);
    if (num_vi_variables == 0 &&
        !(lb(i) <= constant_term && constant_term <= ub(i))) {
      // Unsatisfiable constraint with no variables, such as 1 <= 0 <= 2
      throw SymbolicError(v(i), lb(i), ub(i),
                          "unsatisfiable but called with"
                          " ParseLinearConstraint");

    } else {
      new_lb(i) = lb(i) - constant_term;
      new_ub(i) = ub(i) - constant_term;
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
    }
    return CreateBinding(make_shared<BoundingBoxConstraint>(new_lb, new_ub),
                         bounding_box_x);
  } else {
    return CreateBinding(make_shared<LinearConstraint>(A, new_lb, new_ub),
                         vars);
  }
}

Binding<LinearConstraint> ParseLinearConstraint(const set<Formula>& formulas) {
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
  for (const Formula& f : formulas) {
    if (is_equal_to(f)) {
      // f := (lhs == rhs)
      //      (lhs - rhs == 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = 0.0;
      ub(i) = 0.0;
    } else if (is_less_than_or_equal_to(f)) {
      // f := (lhs <= rhs)
      //      (-∞ <= lhs - rhs <= 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = -numeric_limits<double>::infinity();
      ub(i) = 0.0;
      are_all_formulas_equal = false;
    } else if (is_greater_than_or_equal_to(f)) {
      // f := (lhs >= rhs)
      //      (∞ >= lhs - rhs >= 0)
      v(i) = get_lhs_expression(f) - get_rhs_expression(f);
      lb(i) = 0.0;
      ub(i) = numeric_limits<double>::infinity();
      are_all_formulas_equal = false;
    } else {
      ostringstream oss;
      oss << "ParseLinearConstraint(const set<Formula>& "
          << "formulas) is called while its argument 'formulas' includes "
          << "a formula " << f
          << " which is not a relational formula using one of {==, <=, >=} "
          << "operators.";
      throw runtime_error(oss.str());
    }
    ++i;
  }
  if (are_all_formulas_equal) {
    return ParseLinearEqualityConstraint(v, lb);
  } else {
    return ParseLinearConstraint(v, lb, ub);
  }
}

Binding<LinearConstraint> ParseLinearConstraint(const Formula& f) {
  if (is_equal_to(f)) {
    // e1 == e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearEqualityConstraint(e1 - e2, 0.0);
  } else if (is_greater_than_or_equal_to(f)) {
    // e1 >= e2  ->  e1 - e2 >= 0  ->  0 <= e1 - e2 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearConstraint(e1 - e2, 0.0,
                                 numeric_limits<double>::infinity());
  } else if (is_less_than_or_equal_to(f)) {
    // e1 <= e2  ->  0 <= e2 - e1  ->  0 <= e2 - e1 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return ParseLinearConstraint(e2 - e1, 0.0,
                                 numeric_limits<double>::infinity());
  }
  if (is_conjunction(f)) {
    return ParseLinearConstraint(get_operands(f));
  }
  ostringstream oss;
  oss << "ParseLinearConstraint is called with a formula " << f
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
  oss << "ParseLinearConstraint is called with a formula "
      << f
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
    ExtractAndAppendVariablesFromExpression(v(i), &vars, &map_var_to_index);
  }
  // TODO(hongkai.dai): use sparse matrix.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(v.rows(), vars.rows());
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(v.rows());
  for (int i = 0; i < v.rows(); ++i) {
    double constant_term(0);
    DecomposeLinearExpression(v(i), map_var_to_index, A.row(i), &constant_term);
    beq(i) = b(i) - constant_term;
  }
  return CreateBinding(make_shared<LinearEqualityConstraint>(A, beq), vars);
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
        } else if (monomial.terms.size() == 1) {
          const Polynomiald::VarType term_var = monomial.terms[0].var;
          int var_num = (find(poly_vars.begin(), poly_vars.end(), term_var) -
                         poly_vars.begin());
          DRAKE_ASSERT(var_num < static_cast<int>(poly_vars.size()));
          linear_constraint_matrix(poly_num, var_num) = monomial.coefficient;
        } else {
          DRAKE_ABORT();  // Can't happen (unless isAffine() lied to us).
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
    const Eigen::Ref<const VectorX<Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 2);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  DecomposeLinearExpression(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  return CreateBinding(make_shared<LorentzConeConstraint>(A, b), vars);
}

Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const Expression& linear_expr, const Expression& quadratic_expr) {
  const auto& quadratic_p = ExtractVariablesFromExpression(quadratic_expr);
  const auto& quadratic_vars = quadratic_p.first;
  const auto& quadratic_var_to_index_map = quadratic_p.second;
  const auto& monomial_to_coeff_map = symbolic::DecomposePolynomialIntoMonomial(
      quadratic_expr, quadratic_expr.GetVariables());
  Eigen::MatrixXd Q(quadratic_vars.size(), quadratic_vars.size());
  Eigen::VectorXd b(quadratic_vars.size());
  double a;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, quadratic_var_to_index_map, quadratic_vars.size(),
      &Q, &b, &a);
  // The constraint that the linear expression v1 satisfying
  // v1 >= sqrt(0.5 * x' * Q * x + b' * x + a), is equivalent to the vector
  // [z; y] being within a Lorentz cone, where
  // z = v1
  // y = [1/sqrt(2) * (R * x + R⁻ᵀb); sqrt(a - 0.5 * bᵀ * Q⁻¹ * a)]
  // R is the matrix satisfying Rᵀ * R = Q

  VectorX<Expression> expr{};

  double constant;  // constant is a - 0.5 * bᵀ * Q⁻¹ * a
  // If Q is strictly positive definite, then use LLT
  Eigen::LLT<Eigen::MatrixXd> llt_Q(Q.selfadjointView<Eigen::Upper>());
  if (llt_Q.info() == Eigen::Success) {
    Eigen::MatrixXd R = llt_Q.matrixU();
    expr.resize(2 + R.rows());
    expr(0) = linear_expr;
    expr.segment(1, R.rows()) =
        1.0 / std::sqrt(2) * (R * quadratic_vars + llt_Q.matrixL().solve(b));
    constant = a - 0.5 * b.dot(llt_Q.solve(b));
  } else {
    // Q is not strictly positive definite.
    // First check if Q is zero.
    const bool is_Q_zero = (Q.array() == 0).all();

    if (is_Q_zero) {
      // Now check if the linear term b is zero. If both Q and b are zero, then
      // add the linear constraint linear_expr >= sqrt(a); otherwise throw a
      // runtime error.
      const bool is_b_zero = (b.array() == 0).all();
      if (!is_b_zero) {
        ostringstream oss;
        oss << "Expression " << quadratic_expr
            << " is not quadratic, cannot call ParseLorentzConeConstraint.\n";
        throw runtime_error(oss.str());
      } else {
        if (a < 0) {
          ostringstream oss;
          oss << "Expression " << quadratic_expr
              << " is negative, cannot call ParseLorentzConeConstraint.\n";
          throw runtime_error(oss.str());
        }
        Vector2<Expression> expr_constant_quadratic(linear_expr, std::sqrt(a));
        return ParseLorentzConeConstraint(expr_constant_quadratic);
      }
    }
    // Q is not strictly positive, nor is it zero. Use LDLT to decompose Q
    // into R * Rᵀ.
    // Question: is there a better way to compute R * x and R⁻ᵀb? The following
    // code is really ugly.
    Eigen::LDLT<Eigen::MatrixXd> ldlt_Q(Q.selfadjointView<Eigen::Upper>());
    if (ldlt_Q.info() != Eigen::Success || !ldlt_Q.isPositive()) {
      ostringstream oss;
      oss << "Expression" << quadratic_expr
          << " does not have a positive semidefinite Hessian. Cannot be called "
             "with ParseLorentzConeConstraint.\n";
      throw runtime_error(oss.str());
    }
    Eigen::MatrixXd R1 = ldlt_Q.matrixU();
    for (int i = 0; i < R1.rows(); ++i) {
      for (int j = 0; j < i; ++j) {
        R1(i, j) = 0;
      }
      const double d_sqrt = std::sqrt(ldlt_Q.vectorD()(i));
      for (int j = i; j < R1.cols(); ++j) {
        R1(i, j) *= d_sqrt;
      }
    }
    Eigen::MatrixXd R = R1 * ldlt_Q.transpositionsP();

    expr.resize(2 + R1.rows());
    expr(0) = linear_expr;
    // expr.segment(1, R1.rows()) = 1/sqrt(2) * (R * x + R⁻ᵀb)
    expr.segment(1, R1.rows()) =
        1.0 / std::sqrt(2) *
        (R * quadratic_vars + R.transpose().fullPivHouseholderQr().solve(b));
    constant = a - 0.5 * b.dot(ldlt_Q.solve(b));
  }
  if (constant < 0) {
    ostringstream oss;
    oss << "Expression " << quadratic_expr
        << " is not guaranteed to be non-negative, cannot call it with "
           "ParseLorentzConeConstraint.\n";
    throw runtime_error(oss.str());
  }
  expr(expr.rows() - 1) = std::sqrt(constant);
  return ParseLorentzConeConstraint(expr);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
