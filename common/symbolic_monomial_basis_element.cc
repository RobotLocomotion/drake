// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

namespace drake {
namespace symbolic {

namespace {
// Converts a symbolic expression @p e into an internal representation of
// Monomial class, a mapping from a base (Variable) to its exponent (int). This
// function is called inside of the constructor Monomial(const
// symbolic::Expression&).
std::map<Variable, int> ToMonomialPower(const Expression& e) {
  // TODO(soonho): Re-implement this function by using a Polynomial visitor.
  DRAKE_DEMAND(e.is_polynomial());
  std::map<Variable, int> powers;
  if (is_one(e)) {  // This block is deliberately left empty.
  } else if (is_constant(e)) {
    throw std::runtime_error(
        "A constant not equal to 1, this is not a monomial.");
  } else if (is_variable(e)) {
    powers.emplace(get_variable(e), 1);
  } else if (is_pow(e)) {
    const Expression& base{get_first_argument(e)};
    const Expression& exponent{get_second_argument(e)};
    // The following holds because `e` is polynomial.
    DRAKE_DEMAND(is_constant(exponent));
    // The following static_cast (double -> int) does not lose information
    // because of the precondition `e.is_polynomial()`.
    const int n{static_cast<int>(get_constant_value(exponent))};
    powers = ToMonomialPower(base);
    // pow(base, n) => (∏ᵢ xᵢ)^ⁿ => ∏ᵢ (xᵢ^ⁿ)
    for (auto& p : powers) {
      p.second *= n;
    }
  } else if (is_multiplication(e)) {
    if (!is_one(get_constant_in_multiplication(e))) {
      throw std::runtime_error("The constant in the multiplication is not 1.");
    }
    // e = ∏ᵢ pow(baseᵢ, exponentᵢ).
    for (const auto& p : get_base_to_exponent_map_in_multiplication(e)) {
      for (const auto& q : ToMonomialPower(pow(p.first, p.second))) {
        auto it = powers.find(q.first);
        if (it == powers.end()) {
          powers.emplace(q.first, q.second);
        } else {
          it->second += q.second;
        }
      }
    }
  } else {
    throw std::runtime_error(
        "This expression cannot be converted to a monomial.");
  }
  return powers;
}
// Converts a pair of variables and their integer exponents into an internal
// representation of Monomial class, a mapping from a base (Variable) to its
// exponent (int). This function is called in the constructor taking the same
// types of arguments.
}  // namespace
MonomialBasisElement::MonomialBasisElement(
    const std::map<Variable, int>& var_to_degree_map)
    : PolynomialBasisElement(var_to_degree_map) {}

MonomialBasisElement::MonomialBasisElement(
    const Eigen::Ref<const VectorX<Variable>>& vars,
    const Eigen::Ref<const Eigen::VectorXi>& degrees)
    : PolynomialBasisElement(vars, degrees) {}

MonomialBasisElement::MonomialBasisElement(const Variable& var)
    : MonomialBasisElement({{var, 1}}) {}

MonomialBasisElement::MonomialBasisElement(const Variable& var, int degree)
    : MonomialBasisElement({{var, degree}}) {}

MonomialBasisElement::MonomialBasisElement() : PolynomialBasisElement() {}

MonomialBasisElement::MonomialBasisElement(const Expression& e)
    : MonomialBasisElement(ToMonomialPower(e.Expand())) {}

bool MonomialBasisElement::operator<(const MonomialBasisElement& other) const {
  return this->lexicographical_compare(other);
}

std::pair<double, MonomialBasisElement> MonomialBasisElement::EvaluatePartial(
    const Environment& env) const {
  double coeff{};
  std::map<Variable, int> new_basis_element;
  DoEvaluatePartial(env, &coeff, &new_basis_element);
  return std::make_pair(coeff, MonomialBasisElement(new_basis_element));
}

double MonomialBasisElement::DoEvaluate(double variable_val, int degree) const {
  return std::pow(variable_val, degree);
}

Expression MonomialBasisElement::DoToExpression() const {
  // It builds this base_to_exponent_map and uses ExpressionMulFactory to build
  // a multiplication expression.
  std::map<Expression, Expression> base_to_exponent_map;
  for (const auto& [var, degree] : var_to_degree_map()) {
    base_to_exponent_map.emplace(Expression{var}, degree);
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

std::ostream& operator<<(std::ostream& out, const MonomialBasisElement& m) {
  if (m.var_to_degree_map().empty()) {
    return out << 1;
  }
  auto it = m.var_to_degree_map().begin();
  out << it->first;
  if (it->second > 1) {
    out << "^" << it->second;
  }
  for (++it; it != m.var_to_degree_map().end(); ++it) {
    out << " * ";
    out << it->first;
    if (it->second > 1) {
      out << "^" << it->second;
    }
  }
  return out;
}

MonomialBasisElement& MonomialBasisElement::pow_in_place(const int p) {
  if (p < 0) {
    std::ostringstream oss;
    oss << "MonomialBasisElement::pow(int p) is called with a negative p = "
        << p;
    throw std::runtime_error(oss.str());
  }
  if (p == 0) {
    int* total_degree = get_mutable_total_degree();
    *total_degree = 0;
    get_mutable_var_to_degree_map()->clear();
  } else if (p > 1) {
    for (auto& item : *get_mutable_var_to_degree_map()) {
      int& exponent{item.second};
      exponent *= p;
    }
    int* total_degree = get_mutable_total_degree();
    *total_degree *= p;
  }  // If p == 1, NO OP.
  return *this;
}

std::map<MonomialBasisElement, double> MonomialBasisElement::Differentiate(
    const Variable& var) const {
  std::map<Variable, int> new_var_to_degree_map = var_to_degree_map();
  auto it = new_var_to_degree_map.find(var);
  if (it == new_var_to_degree_map.end()) {
    return {};
  }
  const int degree = it->second;
  it->second--;
  return {{MonomialBasisElement(new_var_to_degree_map), degree}};
}

std::map<MonomialBasisElement, double> MonomialBasisElement::Integrate(
    const Variable& var) const {
  auto new_var_to_degree_map = var_to_degree_map();
  auto it = new_var_to_degree_map.find(var);
  if (it == new_var_to_degree_map.end()) {
    // var is not a variable in var_to_degree_map. Append it to
    // new_var_to_degree_map.
    new_var_to_degree_map.emplace_hint(it, var, 1);
    return {{MonomialBasisElement(new_var_to_degree_map), 1.}};
  }
  const int degree = it->second;
  it->second++;
  return {{MonomialBasisElement(new_var_to_degree_map), 1. / (degree + 1)}};
}

std::map<MonomialBasisElement, double> operator*(
    const MonomialBasisElement& m1, const MonomialBasisElement& m2) {
  std::map<Variable, int> var_to_degree_map_product = m1.var_to_degree_map();
  for (const auto& [var, degree] : m2.var_to_degree_map()) {
    auto it = var_to_degree_map_product.find(var);
    if (it == var_to_degree_map_product.end()) {
      var_to_degree_map_product.emplace(var, degree);
    } else {
      it->second += degree;
    }
  }
  MonomialBasisElement product(var_to_degree_map_product);
  return std::map<MonomialBasisElement, double>{{{product, 1.}}};
}

std::map<MonomialBasisElement, double> pow(MonomialBasisElement m, int p) {
  return std::map<MonomialBasisElement, double>{{{m.pow_in_place(p), 1.}}};
}

}  // namespace symbolic
}  // namespace drake
