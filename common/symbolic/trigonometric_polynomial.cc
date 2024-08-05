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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SinCosVisitor);

  explicit SinCosVisitor(SinCosSubstitution s) : subs_{std::move(s)} {};

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
    const symbolic::Variables& sin_cos_set,
    const VectorX<symbolic::Variable>& t, const symbolic::Variables& t_set,
    const VectorX<symbolic::Polynomial>& one_plus_t_angles_squared,
    const VectorX<symbolic::Polynomial>& two_t_angles,
    const VectorX<symbolic::Polynomial>& one_minus_t_angles_squared) {
  DRAKE_DEMAND(static_cast<int>(sin_cos.size()) == t.rows());
  DRAKE_DEMAND(one_plus_t_angles_squared.size() == t.rows());
  DRAKE_DEMAND(two_t_angles.size() == t.rows());
  DRAKE_DEMAND(one_minus_t_angles_squared.size() == t.rows());

  // We first count the degree of each sin(θᵢ) and cos(θᵢ) variable.
  // sin_cos_degrees[i] is the maximal of monomial.degree(cos(θᵢ)) +
  // monomial.degree(sin(θᵢ)) for every monomial in e_poly.
  std::vector<int> sin_cos_degrees(t.rows(), 0);
  for (const auto& [monomial, unused] : e_poly.monomial_to_coefficient_map()) {
    for (int i = 0; i < t.rows(); ++i) {
      const int n =
          monomial.degree(sin_cos[i].s) + monomial.degree(sin_cos[i].c);
      if (n > sin_cos_degrees[i]) {
        sin_cos_degrees[i] = n;
      }
    }
  }
  // The denominator is Πᵢ (1 + tᵢ²)ᵈⁱ, where dᵢ = sin_cos_degrees[i]
  symbolic::Polynomial denominator{1};
  for (int i = 0; i < t.rows(); ++i) {
    const int d_i = sin_cos_degrees[i];
    if (d_i > 0) {
      denominator *= pow(one_plus_t_angles_squared(i), d_i);
    }
  }

  // Now go through each monomial to compute the numerator.
  symbolic::Polynomial numerator{0};
  for (const auto& [monomial, coeff] : e_poly.monomial_to_coefficient_map()) {
    // We substitute sin(θᵢ) and cos(θᵢ) in each monomial, but keep the rest of
    // the monomial. We store the substitution result in post_subs.

    // First the coefficient is unchanged. But since the coefficient might
    // contain `t` in its variables, we need to re-parse the coefficient with
    // indeterminates in t.
    symbolic::Polynomial post_subs = symbolic::Polynomial(coeff, t_set);
    // Loop through each variable in monomial
    for (const auto& [var, degree] : monomial.get_powers()) {
      if (sin_cos_set.find(var) == sin_cos_set.end()) {
        // This is not a sin or cos variable. Keep this variable after
        // substitution.
        post_subs *= symbolic::Monomial(var, degree);
      }
    }
    // Now I only need to handle each sin and cos variable as the other
    // variables have been included in post_subs already.
    for (int i = 0; i < t.rows(); ++i) {
      // If we have pow(sin(θᵢ), d₁)*pow(cos(θᵢ), d₂), we want to substitute it
      // as pow(2tᵢ, d₁)*pow(1−tᵢ², d₂) / pow(1+tᵢ², d₁+d₂). The denominator
      // already has the term pow(1+tᵢ², sin_cos_degrees[i]), hence we want the
      // term pow(2tᵢ, d₁)*pow(1−tᵢ², d₂) * pow(1+tᵢ², sin_cos_degrees[i]-d₁-d₂)
      // in the numerator.
      const int d1 = monomial.degree(sin_cos[i].s);
      const int d2 = monomial.degree(sin_cos[i].c);
      if (d1 > 0) {
        post_subs *= pow(two_t_angles(i), d1);
      }
      if (d2 > 0) {
        post_subs *= pow(one_minus_t_angles_squared(i), d2);
      }
      if (sin_cos_degrees[i] - d1 - d2 > 0) {
        post_subs *=
            pow(one_plus_t_angles_squared(i), sin_cos_degrees[i] - d1 - d2);
      }
    }
    numerator += post_subs;
  }
  return symbolic::RationalFunction(numerator, denominator);
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
  symbolic::Variables sin_cos_set;
  for (const auto& sc : sin_cos) {
    sin_cos_set.insert(sc.s);
    sin_cos_set.insert(sc.c);
  }
  const symbolic::Variables t_set{t};
  return internal::SubstituteStereographicProjectionImpl(
      e_poly, sin_cos, sin_cos_set, t, t_set, one_plus_t_square, two_t,
      one_minus_t_square);
}

}  // namespace symbolic
}  // namespace drake
