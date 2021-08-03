#include "drake/solvers/ibex_converter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace drake {
namespace solvers {
namespace internal {

using std::numeric_limits;
using std::ostringstream;
using std::runtime_error;
using std::vector;

using ibex::ExprCtr;
using ibex::ExprNode;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

namespace {

// This function checks if there exists a 32-bit integer `n` such that `n == v`.
bool is_integer(double v) {
  // v should be in [int_min, int_max].
  if (!((numeric_limits<int>::lowest() <= v) &&
        (v <= numeric_limits<int>::max()))) {
    return false;
  }
  double intpart{};  // dummy variable.
  return std::modf(v, &intpart) == 0.0;
}

}  // namespace

IbexConverter::IbexConverter(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& variables)
    : zero_{&ibex::ExprConstant::new_scalar(0.0)} {
  // Sets up var_array_ and symbolic_var_to_ibex_var_.
  for (int i = 0; i < variables.size(); ++i) {
    const Variable& var{variables[i]};
    // This variable is new to this converter, we need to
    // create a corresponding IBEX symbol.
    const ibex::ExprSymbol* v{
        &ibex::ExprSymbol::new_(var.get_name().c_str(), ibex::Dim::scalar())};
    // Update the map, Variable → ibex::ExprSymbol*.
    symbolic_var_to_ibex_var_.emplace(var.get_id(), v);
    // Update the ibex::Array<const ibex::ExprSymbol>.
    var_array_.add(*v);
  }
}

IbexConverter::~IbexConverter() {
  for (const auto& p : symbolic_var_to_ibex_var_) {
    delete p.second;
  }
}

UniquePtrToExprCtr IbexConverter::Convert(const Formula& f) {
  return UniquePtrToExprCtr{Visit(f, true)};
}

UniquePtrToExprNode IbexConverter::Convert(const Expression& e) {
  return UniquePtrToExprNode{Visit(e)};
}

const ibex::Array<const ibex::ExprSymbol>& IbexConverter::variables() const {
  return var_array_;
}

const ExprNode* IbexConverter::Visit(const Expression& e) {
  return ::drake::symbolic::VisitExpression<const ExprNode*>(this, e);
}

const ExprNode* IbexConverter::VisitVariable(const Expression& e) {
  const Variable& var{get_variable(e)};
  auto const it = symbolic_var_to_ibex_var_.find(var.get_id());
  if (it == symbolic_var_to_ibex_var_.cend()) {
    throw std::runtime_error(fmt::format(
        "Variable {} does not appear in the existing list of variables.",
        var.get_name()));
  }
  return it->second;
}

const ExprNode* IbexConverter::VisitConstant(const Expression& e) {
  return &ibex::ExprConstant::new_scalar(get_constant_value(e));
}

const ExprNode* IbexConverter::VisitAddition(const Expression& e) {
  const double c{get_constant_in_addition(e)};
  const ExprNode* ret{nullptr};
  if (c != 0) {
    ret = &ibex::ExprConstant::new_scalar(c);
  }
  for (const auto& p : get_expr_to_coeff_map_in_addition(e)) {
    const Expression& e_i{p.first};
    const double coeff{p.second};
    if (coeff == 1.0) {
      if (ret) {
        ret = &(*ret + *Visit(e_i));
      } else {
        ret = Visit(e_i);
      }
    } else if (coeff == -1.0) {
      if (ret) {
        ret = &(*ret - *Visit(e_i));
      } else {
        ret = Visit(-e_i);
      }
    } else {
      if (ret) {
        ret = &(*ret + coeff * *Visit(e_i));
      } else {
        ret = &(coeff * *Visit(e_i));
      }
    }
  }
  return ret;
}

const ExprNode* IbexConverter::ProcessPow(const Expression& base,
                                          const Expression& exponent) {
  // Note: IBEX provides the following four function signatures of pow
  // in "ibex_Expr.h" file:
  //
  //   1. pow(EXPR, int)
  //   2. pow(EXPR, double)
  //   3. pow(double, EXPR)
  //   4. pow(EXPR, EXPR)
  //
  // To avoid the loss of precision, we try to avoid calling the last one,
  // "pow(EXPR, EXPR)".
  if (is_constant(exponent)) {
    const double v{get_constant_value(exponent)};
    if (is_integer(v)) {
      // Call pow(EXPR, int).
      return &pow(*Visit(base), static_cast<int>(v));
    }
    if (v == 0.5) {
      // Call sqrt(base).
      return &sqrt(*Visit(base));
    }
    // Call pow(EXPR, double).
    return &pow(*Visit(base), v);
  }
  if (is_constant(base)) {
    // Call pow(double, EXPR).
    const double v{get_constant_value(base)};
    return &pow(v, *Visit(exponent));
  }
  // Call pow(EXPR, EXPR).
  return &pow(*Visit(base), *Visit(exponent));
}

const ExprNode* IbexConverter::VisitMultiplication(const Expression& e) {
  const double c{get_constant_in_multiplication(e)};
  const ExprNode* ret{nullptr};
  if (c != 1.0) {
    ret = &ibex::ExprConstant::new_scalar(c);
  }
  for (const auto& p : get_base_to_exponent_map_in_multiplication(e)) {
    const Expression& base{p.first};
    const Expression& exponent{p.second};
    if (ret) {
      ret = &(*ret * *ProcessPow(base, exponent));
    } else {
      ret = ProcessPow(base, exponent);
    }
  }
  return ret;
}

const ExprNode* IbexConverter::VisitDivision(const Expression& e) {
  return &(*Visit(get_first_argument(e)) / *Visit(get_second_argument(e)));
}

const ExprNode* IbexConverter::VisitLog(const Expression& e) {
  return &log(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitAbs(const Expression& e) {
  return &abs(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitExp(const Expression& e) {
  return &exp(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitSqrt(const Expression& e) {
  return &sqrt(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitPow(const Expression& e) {
  const Expression& base{get_first_argument(e)};
  const Expression& exponent{get_second_argument(e)};
  return ProcessPow(base, exponent);
}

const ExprNode* IbexConverter::VisitSin(const Expression& e) {
  return &sin(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitCos(const Expression& e) {
  return &cos(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitTan(const Expression& e) {
  return &tan(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitAsin(const Expression& e) {
  return &asin(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitAcos(const Expression& e) {
  return &acos(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitAtan(const Expression& e) {
  return &atan(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitAtan2(const Expression& e) {
  return &atan2(*Visit(get_first_argument(e)), *Visit(get_second_argument(e)));
}

const ExprNode* IbexConverter::VisitSinh(const Expression& e) {
  return &sinh(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitCosh(const Expression& e) {
  return &cosh(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitTanh(const Expression& e) {
  return &tanh(*Visit(get_argument(e)));
}

const ExprNode* IbexConverter::VisitMin(const Expression& e) {
  return &min(*Visit(get_first_argument(e)), *Visit(get_second_argument(e)));
}

const ExprNode* IbexConverter::VisitMax(const Expression& e) {
  return &max(*Visit(get_first_argument(e)), *Visit(get_second_argument(e)));
}

const ExprNode* IbexConverter::VisitCeil(const Expression&) {
  throw std::runtime_error(
      "IbexConverter: Ceil expression is not representable in IBEX.");
}

const ExprNode* IbexConverter::VisitFloor(const Expression&) {
  throw std::runtime_error(
      "IbexConverter: Floor expression is not representable in IBEX.");
}

const ExprNode* IbexConverter::VisitIfThenElse(const Expression&) {
  throw std::runtime_error(
      "IbexConverter: If-then-else expression is not representable in IBEX.");
}

const ExprNode* IbexConverter::VisitUninterpretedFunction(const Expression&) {
  throw std::runtime_error(
      "IbexConverter: Uninterpreted function is not representable in IBEX.");
}

// Visits @p e and converts it into ibex::ExprNode.
const ExprCtr* IbexConverter::Visit(const Formula& f, const bool polarity) {
  return ::drake::symbolic::VisitFormula<const ExprCtr*>(this, f, polarity);
}

const ExprCtr* IbexConverter::VisitFalse(const Formula&, const bool) {
  throw std::runtime_error(
      "IbexConverter: False is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitTrue(const Formula&, const bool) {
  throw std::runtime_error("IbexConverter: True is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitVariable(const Formula&, const bool) {
  throw std::runtime_error(
      "IbexConverter: Boolean variable is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitEqualTo(const Formula& f,
                                           const bool polarity) {
  if (polarity) {
    // Note that IBEX provides `ExprNode::operator=(rhs)` which returns an
    // `ExprCtr` representing `this == rhs`.
    //
    // See
    // https://github.com/ibex-team/ibex-lib/blob/7a1e759b7e5993a10cadc8ead57de20d9eb80089/src/symbolic/ibex_Expr.h#L150-L151.
    return &(*Visit(get_lhs_expression(f) - get_rhs_expression(f)) = *zero_);
  }
  throw std::runtime_error(
      "IbexConverter: Non-equlaity (!=) is not representable in IBEX. Note that"
      " considering numerical errors, 'lhs != rhs' holds vacuously.");
}

const ExprCtr* IbexConverter::VisitNotEqualTo(const Formula& f,
                                              const bool polarity) {
  return VisitEqualTo(f, !polarity);
}

const ExprCtr* IbexConverter::VisitGreaterThan(const Formula& f,
                                               const bool polarity) {
  if (polarity) {
    return &(*Visit(get_lhs_expression(f) - get_rhs_expression(f)) > *zero_);
  }
  return &(*Visit(get_lhs_expression(f) - get_rhs_expression(f)) <= *zero_);
}

const ExprCtr* IbexConverter::VisitGreaterThanOrEqualTo(const Formula& f,
                                                        const bool polarity) {
  if (polarity) {
    return &(*Visit(get_lhs_expression(f) - get_rhs_expression(f)) >= *zero_);
  }
  return &(*Visit(get_lhs_expression(f) - get_rhs_expression(f)) < *zero_);
}

const ExprCtr* IbexConverter::VisitLessThan(const Formula& f,
                                            const bool polarity) {
  return VisitGreaterThanOrEqualTo(f, !polarity);
}

const ExprCtr* IbexConverter::VisitLessThanOrEqualTo(const Formula& f,
                                                     const bool polarity) {
  return VisitGreaterThan(f, !polarity);
}

const ExprCtr* IbexConverter::VisitConjunction(const Formula&, const bool) {
  throw std::runtime_error(
      "IbexConverter: Conjunction is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitDisjunction(const Formula&, const bool) {
  throw std::runtime_error(
      "IbexConverter: Disjunction is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitNegation(const Formula& f,
                                            const bool polarity) {
  return Visit(get_operand(f), !polarity);
}

const ExprCtr* IbexConverter::VisitForall(const Formula&, const bool) {
  throw std::runtime_error(
      "IbexConverter: Forall constraint is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitIsnan(const Formula&, const bool) {
  throw std::runtime_error(
      "IbexConverter: Isnan is not representable in IBEX.");
}

const ExprCtr* IbexConverter::VisitPositiveSemidefinite(const Formula&,
                                                        const bool) {
  throw std::runtime_error(
      "IbexConverter: PositiveSemidefinite constraint is not representable in "
      "IBEX.");
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
