#include "drake/common/symbolic/trigonometric_polynomial.h"

#include <map>
#include <optional>
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

}  // namespace symbolic
}  // namespace drake
