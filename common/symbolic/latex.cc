#include "drake/common/symbolic/latex.h"

#include <optional>
#include <sstream>
#include <stdexcept>

namespace drake {
namespace symbolic {

using std::optional;
using std::ostringstream;
using std::runtime_error;
using std::string;
using std::to_string;

namespace {

// If `value` is an integer multiple of `famous_constant_value`, returns the
// latex for the multiplied constant `{int_coeff}{famous_constant_latex}`.
std::optional<string> multiple_of_famous_constant(
    double value, double famous_constant_value, string famous_constant_latex) {
  const double epsilon = 1e-14;
  if (std::abs(value) < epsilon) {  // Handle zero.
    return std::nullopt;
  }
  const double coeff = std::round(value / famous_constant_value);
  const double difference = coeff * famous_constant_value - value;
  if (std::abs(difference) < epsilon) {
    if (coeff == 1.0) {
      return famous_constant_latex;
    } else if (coeff == -1.0) {
      return "-" + famous_constant_latex;
    }
    return ToLatex(coeff, 0) + famous_constant_latex;
  }
  return std::nullopt;
}

// Visitor class for code generation.
class LatexVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LatexVisitor);

  explicit LatexVisitor(int precision) : precision_{precision} {};

  // Generates latex expression for the expression @p e.
  [[nodiscard]] std::string Latex(const Expression& e) const {
    return VisitExpression<string>(this, e);
  }
  [[nodiscard]] std::string Latex(const Formula& f) const {
    return VisitFormula(f, true);
  }

 private:
  [[nodiscard]] std::string VisitVariable(const Expression& e) const {
    std::string s = get_variable(e).to_string();
    // Use subscripts for variable indices.
    // x(a) => x_{a}, x(a,b) => x_{a,b}, etc.
    const auto start_paren = s.find_first_of('(');
    const auto end_paren = s.find_last_of(')');
    if (start_paren != std::string::npos && end_paren != std::string::npos &&
        end_paren > start_paren) {
      s.replace(end_paren, 1, "}");
      s.replace(start_paren, 1, "_{");
    }
    return s;
  }

  [[nodiscard]] std::string VisitConstant(const Expression& e) const {
    return ToLatex(get_constant_value(e), precision_);
  }

  [[nodiscard]] std::string VisitAddition(const Expression& e) const {
    const double c{get_constant_in_addition(e)};
    const auto& expr_to_coeff_map{get_expr_to_coeff_map_in_addition(e)};
    ostringstream oss;
    bool print_plus{false};
    oss << "(";
    if (c != 0.0) {
      oss << ToLatex(c, precision_);
      print_plus = true;
    }
    for (const auto& [term, coeff] : expr_to_coeff_map) {
      if (coeff > 0.0) {
        if (print_plus) {
          oss << " + ";
        }
        // Do not print "1 * t"
        if (coeff != 1.0) {
          oss << ToLatex(coeff, precision_);
        }
      } else {
        // Instead of printing "+ (- E)", just print "- E".
        oss << " - ";
        if (coeff != -1.0) {
          oss << ToLatex((-coeff), precision_);
        }
      }
      oss << Latex(term);
      print_plus = true;
    }
    oss << ")";
    return oss.str();
  }

  [[nodiscard]] std::string VisitMultiplication(const Expression& e) const {
    const double c{get_constant_in_multiplication(e)};
    const auto& base_to_exponent_map{
        get_base_to_exponent_map_in_multiplication(e)};
    bool print_space = false;
    ostringstream oss;
    if (c != 1.0) {
      oss << ToLatex(c, precision_);
      print_space = true;
    }
    for (const auto& [e_1, e_2] : base_to_exponent_map) {
      if (print_space) {
        oss << " ";
      }
      if (is_one(e_2)) {
        oss << Latex(e_1);
      } else {
        oss << Latex(e_1) << "^{" << Latex(e_2) << "}";
      }
      print_space = true;
    }
    return oss.str();
  }

  // Helper method to handle unary cases.
  [[nodiscard]] string VisitUnary(const string& f, const Expression& e) const {
    return "\\" + f + "{" + Latex(get_argument(e)) + "}";
  }

  // Helper method to handle binary cases.
  [[nodiscard]] string VisitBinary(const string& f, const Expression& e) const {
    return "\\" + f + "\\{" + Latex(get_first_argument(e)) + ", " +
           Latex(get_second_argument(e)) + "\\}";
  }

  [[nodiscard]] string VisitPow(const Expression& e) const {
    return Latex(get_first_argument(e)) + "^{" + Latex(get_second_argument(e)) +
           "}";
  }

  [[nodiscard]] string VisitDivision(const Expression& e) const {
    return "\\frac{" + Latex(get_first_argument(e)) + "}{" +
           Latex(get_second_argument(e)) + "}";
  }

  [[nodiscard]] string VisitAbs(const Expression& e) const {
    return "|" + Latex(get_argument(e)) + "|";
  }

  [[nodiscard]] string VisitLog(const Expression& e) const {
    return VisitUnary("log", e);
  }

  [[nodiscard]] string VisitExp(const Expression& e) const {
    return "e^{" + Latex(get_argument(e)) + "}";
  }

  [[nodiscard]] string VisitSqrt(const Expression& e) const {
    return VisitUnary("sqrt", e);
  }

  [[nodiscard]] string VisitSin(const Expression& e) const {
    return VisitUnary("sin", e);
  }

  [[nodiscard]] string VisitCos(const Expression& e) const {
    return VisitUnary("cos", e);
  }

  [[nodiscard]] string VisitTan(const Expression& e) const {
    return VisitUnary("tan", e);
  }

  [[nodiscard]] string VisitAsin(const Expression& e) const {
    return VisitUnary("asin", e);
  }

  [[nodiscard]] string VisitAcos(const Expression& e) const {
    return VisitUnary("acos", e);
  }

  [[nodiscard]] string VisitAtan(const Expression& e) const {
    return VisitUnary("atan", e);
  }

  [[nodiscard]] string VisitAtan2(const Expression& e) const {
    return "\\atan{\\frac{" + Latex(get_first_argument(e)) + "}{" +
           Latex(get_second_argument(e)) + "}}";
  }

  [[nodiscard]] string VisitSinh(const Expression& e) const {
    return VisitUnary("sinh", e);
  }

  [[nodiscard]] string VisitCosh(const Expression& e) const {
    return VisitUnary("cosh", e);
  }

  [[nodiscard]] string VisitTanh(const Expression& e) const {
    return VisitUnary("tanh", e);
  }

  [[nodiscard]] string VisitMin(const Expression& e) const {
    return VisitBinary("min", e);
  }

  [[nodiscard]] string VisitMax(const Expression& e) const {
    return VisitBinary("max", e);
  }

  [[nodiscard]] string VisitCeil(const Expression& e) const {
    return "\\lceil " + Latex(get_argument(e)) + " \\rceil";
  }

  [[nodiscard]] string VisitFloor(const Expression& e) const {
    return "\\lfloor " + Latex(get_argument(e)) + " \\rfloor";
  }

  [[nodiscard]] string VisitIfThenElse(const Expression& e) const {
    std::ostringstream oss;
    oss << "\\begin{cases} ";
    oss << Latex(get_then_expression(e)) << " & \\text{if } ";
    oss << Latex(get_conditional_formula(e)) << ", \\\\ ";
    oss << Latex(get_else_expression(e)) << " & \\text{otherwise}.";
    oss << "\\end{cases}";
    return oss.str();
  }

  [[nodiscard]] string VisitUninterpretedFunction(const Expression&) const {
    throw runtime_error("ToLatex does not support uninterpreted functions.");
  }

  // The parameter `polarity` is to indicate whether it processes `f` (if
  // `polarity` is true) or `¬f` (if `polarity` is false).
  [[nodiscard]] std::string VisitFormula(const Formula& f,
                                         const bool polarity = true) const {
    return symbolic::VisitFormula<std::string>(this, f, polarity);
  }
  [[nodiscard]] std::string VisitFalse(const Formula&,
                                       const bool polarity) const {
    return polarity ? "\\text{false}" : "\\text{true}";
  }
  [[nodiscard]] std::string VisitTrue(const Formula&,
                                      const bool polarity) const {
    return polarity ? "\\text{true}" : "\\text{false}";
  }
  [[nodiscard]] std::string VisitVariable(const Formula& f,
                                          const bool polarity) const {
    return (polarity ? "" : "\\neg") + get_variable(f).to_string();
  }
  [[nodiscard]] std::string VisitEqualTo(const Formula& f,
                                         const bool polarity) const {
    return Latex(get_lhs_expression(f)) + (polarity ? " = " : " \\neq ") +
           Latex(get_rhs_expression(f));
  }
  [[nodiscard]] std::string VisitNotEqualTo(const Formula& f,
                                            const bool polarity) const {
    return Latex(get_lhs_expression(f)) + (polarity ? " \\neq " : " = ") +
           Latex(get_rhs_expression(f));
  }
  [[nodiscard]] std::string VisitGreaterThan(const Formula& f,
                                             const bool polarity) const {
    return Latex(get_lhs_expression(f)) + (polarity ? " > " : " \\le ") +
           Latex(get_rhs_expression(f));
  }
  [[nodiscard]] std::string VisitGreaterThanOrEqualTo(
      const Formula& f, const bool polarity) const {
    return Latex(get_lhs_expression(f)) + (polarity ? " \\ge " : " < ") +
           Latex(get_rhs_expression(f));
  }
  [[nodiscard]] std::string VisitLessThan(const Formula& f,
                                          const bool polarity) const {
    return Latex(get_lhs_expression(f)) + (polarity ? " < " : " \\ge ") +
           Latex(get_rhs_expression(f));
  }
  [[nodiscard]] std::string VisitLessThanOrEqualTo(const Formula& f,
                                                   const bool polarity) const {
    return Latex(get_lhs_expression(f)) + (polarity ? " \\le " : " > ") +
           Latex(get_rhs_expression(f));
  }
  [[nodiscard]] std::string VisitConjunction(const Formula& f,
                                             const bool polarity) const {
    ostringstream oss;
    bool print_symbol = false;
    for (const auto& o : get_operands(f)) {
      if (print_symbol) {
        oss << (polarity ? " \\land " : " \\lor ");
      }
      oss << VisitFormula(o, polarity);
      print_symbol = true;
    }
    return oss.str();
  }
  [[nodiscard]] std::string VisitDisjunction(const Formula& f,
                                             const bool polarity) const {
    ostringstream oss;
    bool print_symbol = false;
    for (const auto& o : get_operands(f)) {
      if (print_symbol) {
        oss << (polarity ? " \\lor " : " \\land ");
      }
      oss << VisitFormula(o, polarity);
      print_symbol = true;
    }
    return oss.str();
  }

  [[nodiscard]] std::string VisitNegation(const Formula& f,
                                          const bool polarity) const {
    return VisitFormula(get_operand(f), !polarity);
  }

  [[nodiscard]] std::string VisitForall(const Formula& f,
                                        const bool polarity) const {
    // TODO(russt): The polarity==False case can be further reduced into
    // ∃v₁...vₙ. (¬f). However, we do not have a representation
    // FormulaExists(∃) yet. Revisit this when we add FormulaExists.
    ostringstream oss;
    if (!polarity) {
      oss << "\\neg ";
    }
    oss << "\\forall " << VisitVariables(get_quantified_variables(f)) << ": "
        << get_quantified_formula(f);
    return oss.str();
  }

  [[nodiscard]] std::string VisitIsnan(const Formula& f,
                                       const bool polarity) const {
    ostringstream oss;
    if (!polarity) {
      oss << "\\neg ";
    }
    oss << "\\text{isnan}(" << Latex(get_unary_expression(f)) << ")";
    return oss.str();
  }

  [[nodiscard]] std::string VisitPositiveSemidefinite(
      const Formula& f, const bool polarity) const {
    DRAKE_ASSERT(polarity);  // "Not PSD" could be an arbitrary matrix.
    return ToLatex(get_matrix_in_positive_semidefinite(f), precision_) +
           " \\succeq 0";
  }

  // Note: This method is not called directly from VisitExpression; but can be
  // used by other methods in this class.
  [[nodiscard]] std::string VisitVariables(const Variables& vars) const {
    ostringstream oss;
    bool delimiter = false;
    for (const auto& v : vars) {
      if (delimiter) {
        oss << ", ";
      }
      oss << VisitVariable(v);
      delimiter = true;
    }
    return oss.str();
  }

  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend std::string VisitExpression<std::string>(const LatexVisitor*,
                                                  const Expression&);

  // Makes VisitFormula a friend of this class so that it can use private
  // methods.
  friend std::string symbolic::VisitFormula<std::string>(const LatexVisitor*,
                                                         const Formula&,
                                                         const bool&);

  int precision_;
};

}  // namespace

string ToLatex(const Expression& e, int precision) {
  return LatexVisitor{precision}.Latex(e);
}

string ToLatex(const Formula& f, int precision) {
  return LatexVisitor{precision}.Latex(f);
}

string ToLatex(double val, int precision) {
  if (std::isnan(val)) {
    return "\\text{NaN}";
  }
  if (std::isinf(val)) {
    return val < 0 ? "-\\infty" : "\\infty";
  }
  if (optional<string> result =
          multiple_of_famous_constant(val, M_PI, "\\pi")) {
    return *result;
  }
  if (optional<string> result = multiple_of_famous_constant(val, M_E, "e")) {
    return *result;
  }
  double intpart;
  if (std::modf(val, &intpart) == 0.0) {
    // Then it's an integer. Note that we can't static_cast<int>() because it
    // might not be a *small* integer.
    return fmt::format("{:.0f}", val);
  }
  std::ostringstream oss;
  oss.precision(precision);
  oss << std::fixed << val;
  return oss.str();
}

}  // namespace symbolic
}  // namespace drake
