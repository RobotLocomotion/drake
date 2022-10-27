#include "drake/common/symbolic/trigonometric_polynomial.h"

#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#define DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER
#include "drake/common/symbolic/expression/expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER

#include <fmt/format.h>

namespace drake {
namespace symbolic {

namespace {

enum TrigStatus {
  kNotSinCos,
  kInsideSin,
  kInsideCos,
};

// Visitor class for sin/cos substitution.
class SinCosVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SinCosVisitor)

  explicit SinCosVisitor(SinCosSubstitution s)
      : subs_{std::move(s)} {};

  [[nodiscard]] Expression Substitute(
      const Expression& e,
      std::optional<bool> needs_substitution = std::nullopt) const {
    if (!needs_substitution.has_value()) {
      needs_substitution = false;
      Variables vars = e.GetVariables();
      for (const auto& sc : subs_) {
        if (vars.find(sc.first) != vars.end()) {
          needs_substitution = true;
        }
      }
    }
    if (*needs_substitution) {
      return VisitExpression<Expression, const SinCosVisitor, TrigStatus>(
          this, e, kNotSinCos);
    } else {
      return e;
    }
  }

  [[nodiscard]] Expression VisitVariable(const Expression& e,
                                         TrigStatus status) const {
    auto iter = subs_.find(get_variable(e));
    if (iter == subs_.end()) {
      return e;
    } else if (status == kInsideSin) {
      return iter->second.type == SinCosSubstitutionType::kAngle
                 ? iter->second.s
                 : 2 * iter->second.s * iter->second.c;
    } else if (status == kInsideCos) {
      switch (iter->second.type) {
        case SinCosSubstitutionType::kAngle:
          return iter->second.c;
        case SinCosSubstitutionType::kHalfAnglePreferSin:
          return 1 - 2 * iter->second.s * iter->second.s;
        case SinCosSubstitutionType::kHalfAnglePreferCos:
          return 2 * iter->second.c * iter->second.c - 1;
      }
    }
    return e;
  }

  [[nodiscard]] Expression VisitConstant(const Expression& e,
                                         TrigStatus status) const {
    if (status == kInsideSin) {
      return sin(e);
    } else if (status == kInsideCos) {
      return cos(e);
    }
    return e;
  }

  [[nodiscard]] Expression VisitAddition(const Expression& e,
                                         TrigStatus status) const {
    const double c{get_constant_in_addition(e)};
    std::map<Expression, double> expr_to_coeff_map{
        get_expr_to_coeff_map_in_addition(e)};
    if (status == kInsideSin) {
      if (c != 0.0) {
        return sin(c) * Substitute(cos(e - c), true) +
               cos(c) * Substitute(sin(e - c), true);
      }
      // Recursively substitute sin(x + y) where x is the first term, and y is
      // all the rest.
      auto x = expr_to_coeff_map.extract(expr_to_coeff_map.begin());
      Expression y = ExpressionAddFactory(0, expr_to_coeff_map).GetExpression();
      return Substitute(sin(x.key() * x.mapped())) * Substitute(cos(y)) +
             Substitute(cos(x.key() * x.mapped())) * Substitute(sin(y));
    } else if (status == kInsideCos) {
      if (c != 0.0) {
        return cos(c) * Substitute(cos(e - c), true) -
               sin(c) * Substitute(sin(e - c), true);
      }
      // Recursively substitute cos(x + y) where x is the first term, and y is
      // all the rest.
      auto x = expr_to_coeff_map.extract(expr_to_coeff_map.begin());
      Expression y = ExpressionAddFactory(0, expr_to_coeff_map).GetExpression();
      return Substitute(cos(x.key() * x.mapped())) * Substitute(cos(y)) -
             Substitute(sin(x.key() * x.mapped())) * Substitute(sin(y));
    }
    Expression v{c};
    for (const auto& [term, coeff] : expr_to_coeff_map) {
      v += coeff * Substitute(term);
    }
    return v;
  }

  [[nodiscard]] Expression VisitMultiplication(const Expression& e,
                                               TrigStatus status) const {
    const double c{get_constant_in_multiplication(e)};
    const std::map<Expression, Expression>& base_to_exponent_map{
        get_base_to_exponent_map_in_multiplication(e)};
    if (status == kInsideSin) {
      std::string msg = fmt::format(
          "Got sin({}), but we only support sin(c*x) where c is an integer (c "
          "= +/- 0.5 is also supported if performing a half-angle "
          "substitution), and x is a variable to be substituted.",
          e.to_string());
      if (base_to_exponent_map.size() != 1 ||
          base_to_exponent_map.begin()->second != 1.0) {
        throw std::runtime_error(msg);
      }
      Expression x = base_to_exponent_map.begin()->first;
      if (!is_variable(x)) {
        throw std::runtime_error(msg);
      }
      auto iter = subs_.find(get_variable(x));
      if (iter == subs_.end()) {
        throw std::runtime_error(msg);
      }
      if (iter->second.type != SinCosSubstitutionType::kAngle) {
        // Handle special case of sin(±0.5*x).
        if (c == 0.5) {
          return iter->second.s;
        } else if (c == -0.5) {
          return -iter->second.s;
        }
        // TODO(russt): Handle the cases where c is an odd multiple of 0.5.
      }
      double c_int;
      if (modf(c, &c_int) != 0.0) {
        throw std::runtime_error(msg);
      }
      if (c_int == 0.0) {
        return 0.0;  // Don't expect to get here, though.
      } else if (c_int == 1.0) {
        return Substitute(sin(x), true);
      } else if (c_int > 1.0) {
        // TODO(russt): Use
        // https://mathworld.wolfram.com/Multiple-AngleFormulas.html to avoid
        // this recursive computation.
        return Substitute(
            sin(x) * cos((c_int - 1) * x) + cos(x) * sin((c_int - 1) * x),
            true);
      } else if (c_int < 0.0) {
        return Substitute(-sin(-c_int * x), true);
      }
      throw std::runtime_error("Got unexpected 0 < c_int < 1");
    } else if (status == kInsideCos) {
      std::string msg = fmt::format(
          "Got cos({}), but we only support cos(c*x) where c is an integer (c "
          "= +/- 0.5 is also supported if performing a half-angle "
          "substitution), and x is a variable to be substituted.",
          e.to_string());
      if (base_to_exponent_map.size() != 1 ||
          base_to_exponent_map.begin()->second != 1.0) {
        throw std::runtime_error(msg);
      }
      Expression x = base_to_exponent_map.begin()->first;
      if (!is_variable(x)) {
        throw std::runtime_error(msg);
      }
      auto iter = subs_.find(get_variable(x));
      if (iter == subs_.end()) {
          throw std::runtime_error(msg);
      }
      if (iter->second.type != SinCosSubstitutionType::kAngle) {
        // Handle special case of cos(±0.5*x).
        if (c == 0.5 || c == -0.5) {
          return iter->second.c;
        }
        // TODO(russt): Handle the cases where c is an odd multiple of 0.5.
      }
      double c_int;
      if (modf(c, &c_int) != 0.0) {
        throw std::runtime_error(msg);
      }
      if (c_int == 0.0) {
        return 1.0;  // Don't expect to get here, though.
      } else if (c_int == 1.0) {
        return Substitute(cos(x), true);
      } else if (c_int > 1.0) {
        // TODO(russt): Use
        // https://mathworld.wolfram.com/Multiple-AngleFormulas.html to avoid
        // this recursive computation.
        return Substitute(
            cos(x) * cos((c_int - 1) * x) - sin(x) * sin((c_int - 1) * x),
            true);
      } else if (c_int < 0.0) {
        return Substitute(cos(-c_int * x), true);
      }
      throw std::runtime_error("Got unexpected 0 < c_int < 1");
    }
    Expression v{c};
    for (const auto& [base, exponent] : base_to_exponent_map) {
      v *= pow(Substitute(base), Substitute(exponent));
    }
    return v;
  }

  // Helper method to handle unary cases.
  [[nodiscard]] Expression VisitUnary(
      const std::function<Expression(const Expression&)>& pred,
      const Expression& e) const {
    // Getting here implies that get_argument(e) needs substitution.
    return pred(Substitute(get_argument(e), true));
  }

  // Helper method to handle binary cases.
  [[nodiscard]] Expression VisitBinary(
      const std::function<Expression(const Expression&, const Expression&)>&
          pred,
      const Expression& e) const {
    return pred(Substitute(get_first_argument(e)),
                Substitute(get_second_argument(e)));
  }

  [[nodiscard]] Expression VisitPow(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return pow(Substitute(get_first_argument(e)),
               Substitute(get_second_argument(e)));
  }

  [[nodiscard]] Expression VisitDivision(const Expression& e,
                                         TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return Substitute(get_first_argument(e)) /
           Substitute(get_second_argument(e));
  }

  [[nodiscard]] Expression VisitAbs(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(abs, e);
  }

  [[nodiscard]] Expression VisitLog(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(log, e);
  }

  [[nodiscard]] Expression VisitExp(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(exp, e);
  }

  [[nodiscard]] Expression VisitSqrt(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(sqrt, e);
  }

  [[nodiscard]] Expression VisitSin(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);  // Do not support nested sin/cos.

    return VisitExpression<Expression, const SinCosVisitor, TrigStatus>(
        this, get_argument(e), kInsideSin);
  }

  [[nodiscard]] Expression VisitCos(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);  // Do not support nested sin/cos.

    return VisitExpression<Expression, const SinCosVisitor, TrigStatus>(
        this, get_argument(e), kInsideCos);
  }

  [[nodiscard]] Expression VisitTan(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);  // Do not support nested sin/cos.
    return Substitute(sin(get_argument(e))) / Substitute(cos(get_argument(e)));
  }

  [[nodiscard]] Expression VisitAsin(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(asin, e);
  }

  [[nodiscard]] Expression VisitAcos(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(acos, e);
  }

  [[nodiscard]] Expression VisitAtan(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(atan, e);
  }

  [[nodiscard]] Expression VisitAtan2(const Expression& e,
                                      TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitBinary(atan2, e);
  }

  [[nodiscard]] Expression VisitSinh(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(sinh, e);
  }

  [[nodiscard]] Expression VisitCosh(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(cosh, e);
  }

  [[nodiscard]] Expression VisitTanh(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(tanh, e);
  }

  [[nodiscard]] Expression VisitMin(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitBinary(min, e);
  }

  [[nodiscard]] Expression VisitMax(const Expression& e,
                                    TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitBinary(max, e);
  }

  [[nodiscard]] Expression VisitCeil(const Expression& e,
                                     TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(ceil, e);
  }

  [[nodiscard]] Expression VisitFloor(const Expression& e,
                                      TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    return VisitUnary(floor, e);
  }

  [[nodiscard]] Expression VisitIfThenElse(const Expression& e,
                                           TrigStatus status) const {
    DRAKE_THROW_UNLESS(status == kNotSinCos);
    // We don't support Formulas yet, so need to throw if the Formula requires
    // any substitutions.
    Variables vars = get_conditional_formula(e).GetFreeVariables();
    for (const auto& sc : subs_) {
      if (vars.find(sc.first) != vars.end()) {
        throw std::runtime_error(
            "Substituting sin/cos into formulas is not supported yet");
      }
    }

    // TODO(russt): substitute conditional once we visit formulas.
    return if_then_else(get_conditional_formula(e),
                        Substitute(get_then_expression(e)),
                        Substitute(get_else_expression(e)));
  }

  [[nodiscard]] Expression VisitUninterpretedFunction(const Expression&,
                                                      TrigStatus) const {
    throw std::runtime_error(
        "SubstituteSinCos does not support uninterpreted functions.");
  }

 private:
  SinCosSubstitution subs_;
};

}  // namespace

Expression Substitute(const Expression& e, const SinCosSubstitution& subs) {
  return SinCosVisitor(subs).Substitute(e);
}

namespace internal {
symbolic::RationalFunction SubstituteStereographicProjectionImpl(
    const symbolic::Polynomial& e_poly, const std::vector<SinCos>& sin_cos,
    const VectorX<symbolic::Variable>& t, const symbolic::Variables& t_set,
    const VectorX<symbolic::Polynomial>& one_plus_t_angles_squared,
    const VectorX<symbolic::Polynomial>& two_t_angles,
    const VectorX<symbolic::Polynomial>& one_minus_t_angles_squared) {
  DRAKE_DEMAND(static_cast<int>(sin_cos.size()) == t.rows());
  DRAKE_DEMAND(one_plus_t_angles_squared.size() == t.rows());
  DRAKE_DEMAND(two_t_angles.size() == t.rows());
  DRAKE_DEMAND(one_minus_t_angles_squared.size() == t.rows());

  symbolic::Variables sin_cos_vars;
  for (const auto& sc : sin_cos) {
    sin_cos_vars.insert(sc.s);
    sin_cos_vars.insert(sc.c);
  }
  // We also group the variables in e_poly.indeterminates() that are not in
  // sin_cos_vars to non_sin_cos_vars.
  symbolic::Variables non_sin_cos_vars;
  for (const auto& var : e_poly.indeterminates()) {
    if (sin_cos_vars.find(var) == sin_cos_vars.end()) {
      non_sin_cos_vars.insert(var);
    }
  }
  if (non_sin_cos_vars.empty()) {
    // e_poly.indeterminates() is a subset of sin_cos_vars.
    // First find the angles whose cos or sin appear in the polynomial. This
    // will determine the denominator of the rational function.
    std::set<int> angle_indices;
    for (const auto& pair : e_poly.monomial_to_coefficient_map()) {
      // Also check that this monomial can't contain both cos_vars(i) and
      // sin_vars(i).
      for (int i = 0; i < static_cast<int>(sin_cos.size()); ++i) {
        const auto& sin_var = sin_cos[i].s;
        const auto& cos_var = sin_cos[i].c;
        const int angle_degree =
            pair.first.degree(cos_var) + pair.first.degree(sin_var);
        DRAKE_DEMAND(angle_degree <= 1);
        if (angle_degree == 1) {
          angle_indices.insert(i);
        }
      }
    }
    if (angle_indices.empty()) {
      return symbolic::RationalFunction(
          symbolic::Polynomial(e_poly.ToExpression(), t_set));
    }
    const symbolic::Monomial monomial_one{};
    symbolic::Polynomial denominator{1};
    for (int angle_index : angle_indices) {
      // denominator *= (1 + t(angle_index)^2)
      denominator *= one_plus_t_angles_squared[angle_index];
    }
    symbolic::Polynomial numerator{};

    for (const auto& [monomial, coeff] : e_poly.monomial_to_coefficient_map()) {
      // If the monomial contains cos_vars(i), then replace cos_vars(i) with
      // 1 - t(i) * t(i).
      // If the monomial contains sin_vars(i), then replace sin_vars(i) with
      // 2 * t(i).
      // Otherwise, multiplies with 1 + t(i) * t(i)

      // The coefficient could contain "t_set", (the indeterminates for e are
      // cos_vars and sin_vars). Hence we first need to write the coefficient
      // as a polynomial of indeterminates intersect(t_set, coeff.variables()).
      symbolic::Polynomial numerator_term(
          coeff, symbolic::intersect(t_set, coeff.GetVariables()));
      for (int angle_index : angle_indices) {
        const auto& sin_var = sin_cos[angle_index].s;
        const auto& cos_var = sin_cos[angle_index].c;
        if (monomial.degree(cos_var) > 0) {
          numerator_term *= one_minus_t_angles_squared[angle_index];
        } else if (monomial.degree(sin_var) > 0) {
          numerator_term *= two_t_angles[angle_index];
        } else {
          numerator_term *= one_plus_t_angles_squared[angle_index];
        }
      }
      numerator += numerator_term;
    }

    return symbolic::RationalFunction(numerator, denominator);
  } else {
    // The indeterminates of e_poly contains sin/cos and other indeterminates.
    // I parse e_poly as a polynomial of only sin_cos_vars, and then convert
    // this new polynomial to a rational function; finally I reparse this new
    // rational function with indeterminates t + non_sin_cos_vars.
    const symbolic::Polynomial e_poly_reparse(e_poly.ToExpression(),
                                              sin_cos_vars);
    const symbolic::RationalFunction e_rational_reparse =
        SubstituteStereographicProjectionImpl(
            e_poly_reparse, sin_cos, t, t_set, one_plus_t_angles_squared,
            two_t_angles, one_minus_t_angles_squared);
    // indeterminates is the union of non_sin_cos_vars and t_set.
    symbolic::Variables indeterminates = non_sin_cos_vars;
    for (const auto& var : t_set) {
      indeterminates.insert(var);
    }

    const symbolic::Polynomial numerator(
        e_rational_reparse.numerator().ToExpression(), indeterminates);
    // We know that the indeterminates in e_rational_reparse.denominator() are
    // all in t_set, already a subset of `indeterminates`, so we do not need to
    // rebuild this polynomial with `indeterminates`.
    return symbolic::RationalFunction(numerator,
                                      e_rational_reparse.denominator());
  }
}
}  // namespace internal

symbolic::RationalFunction SubstituteStereographicProjection(
    const symbolic::Polynomial& e_poly, const std::vector<SinCos>& sin_cos,
    const VectorX<symbolic::Variable>& t) {
  const symbolic::Monomial monomial_one{};
  VectorX<symbolic::Polynomial> one_minus_t_square(t.rows());
  VectorX<symbolic::Polynomial> two_t(t.rows());
  VectorX<symbolic::Polynomial> one_plus_t_square(t.rows());
  for (int i = 0; i < t.rows(); ++i) {
    one_minus_t_square[i] = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(t(i), 2), -1}});
    two_t[i] = symbolic::Polynomial({{symbolic::Monomial(t(i), 1), 2}});
    one_plus_t_square[i] = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(t(i), 2), 1}});
  }
  const symbolic::Variables t_set{t};
  return internal::SubstituteStereographicProjectionImpl(
      e_poly, sin_cos, t, t_set, one_plus_t_square, two_t, one_minus_t_square);
}

symbolic::RationalFunction SubstituteStereographicProjection(
    const symbolic::Expression& e,
    const std::unordered_map<symbolic::Variable, symbolic::Variable>& subs) {
  // First substitute all cosθ and sinθ with cos_var and sin_var.
  SinCosSubstitution sin_cos_subs;
  symbolic::Variables sin_cos_vars;
  std::vector<SinCos> sin_cos_vec;
  sin_cos_vec.reserve(subs.size());
  VectorX<symbolic::Variable> t_vars(subs.size());
  int var_count = 0;
  for (const auto& [theta, t] : subs) {
    const SinCos sin_cos(Variable("s_" + theta.get_name()),
                         Variable("c_" + theta.get_name()),
                         SinCosSubstitutionType::kAngle);
    sin_cos_subs.emplace(theta, sin_cos);
    sin_cos_vars.insert(sin_cos.c);
    sin_cos_vars.insert(sin_cos.s);
    sin_cos_vec.push_back(sin_cos);
    t_vars(var_count) = t;
    var_count++;
  }
  const symbolic::Expression e_multilinear = Substitute(e, sin_cos_subs);

  // Now rewrite e_multilinear as a polynomial of sin and cos.
  const symbolic::Polynomial e_poly{e_multilinear, sin_cos_vars};

  return SubstituteStereographicProjection(e_poly, sin_cos_vec, t_vars);
}

}  // namespace symbolic
}  // namespace drake
