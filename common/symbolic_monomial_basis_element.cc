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

namespace {
// Convert a univariate monomial to a weighted sum of Chebyshev polynomials
// For example x³ = 0.25T₃(x) + 0.75T₁(x)
// We return a vector of (degree, coeff) to represent the weighted sum of
// Chebyshev polynomials. For example, 0.25T₃(x) + 0.75T₁(x) is represented
// as [(3, 0.25), (1, 0.75)].
std::vector<std::pair<int, double>> UnivariateMonomialToChebyshevBasis(
    int degree) {
  if (degree == 0) {
    // Return T0(x)
    return std::vector<std::pair<int, double>>{{{0, 1}}};
  }
  // According to equation 3.35 of
  // https://archive.siam.org/books/ot99/OT99SampleChapter.pdf, we know that
  // xⁿ = 2 ¹⁻ⁿ∑ₖ cₖ Tₙ₋₂ₖ(x), k = 0, ..., floor(n/2)
  // where cₖ = 0.5 nchoosek(n, k) if k=n/2, and cₖ = nchoosek(n, k)
  // otherwise
  // Note that the euqation 3.35 of the referenced doc is not entirely correct,
  // specifically the special case (half the coefficient) should be k = n/2
  // instead of k=0.
  const int half_n = degree / 2;
  std::vector<std::pair<int, double>> result(half_n + 1);
  result[0] = std::make_pair(degree, std::pow(2, 1 - degree));
  for (int k = 1; k < half_n + 1; ++k) {
    // Use the relationshipe nchoosek(n, k) = nchoosek(n, k-1) * (n-k+1)/k
    double new_coeff = result[k - 1].second *
                       static_cast<double>(degree - k + 1) /
                       static_cast<double>(k);
    if (2 * k == degree) {
      new_coeff /= 2;
    }
    result[k] = std::make_pair(degree - 2 * k, new_coeff);
  }
  return result;
}

std::map<ChebyshevBasisElement, double> MonomialToChebyshevBasisRecursive(
    std::map<Variable, int> var_to_degree_map) {
  if (var_to_degree_map.empty()) {
    // 1 = T0()
    return {{ChebyshevBasisElement(), 1}};
  }
  auto it = var_to_degree_map.begin();
  const Variable var_first = it->first;
  // If we want to convert xⁿyᵐzˡ to Chebyshev basis, we could first convert xⁿ
  // to Chebyshev basis as xⁿ=∑ᵢcᵢTᵢ(x), and convert yᵐzˡ to Chebyshev basis as
  // yᵐzˡ = ∑ⱼ,ₖdⱼₖTⱼ(y)Tₖ(z), then we multiply them as
  // xⁿyᵐzˡ
  // = xⁿ*(yᵐzˡ)
  // = ∑ᵢcᵢTᵢ(x) * ∑ⱼ,ₖdⱼₖTⱼ(y)Tₖ(z)
  // = ∑ᵢ,ⱼ,ₖcᵢdⱼₖTᵢ(x)Tⱼ(y)Tₖ(z)

  // first_univariate_in_chebyshev contains the degree/coefficient pairs (i, cᵢ)
  // above.
  const std::vector<std::pair<int, double>> first_univariate_in_chebyshev =
      UnivariateMonomialToChebyshevBasis(it->second);

  var_to_degree_map.erase(it);
  // remaining_chebyshevs contains the chebyshev polynomial/coefficient pair
  // (Tⱼ(y)Tₖ(z), dⱼₖ)
  const std::map<ChebyshevBasisElement, double> remaining_chebyshevs =
      MonomialToChebyshevBasisRecursive(var_to_degree_map);
  std::map<ChebyshevBasisElement, double> result;
  for (const auto& [degree_x, coeff_x] : first_univariate_in_chebyshev) {
    for (const auto& [remaining_vars, coeff_remaining_vars] :
         remaining_chebyshevs) {
      std::map<Variable, int> new_chebyshev_var_to_degree_map =
          remaining_vars.var_to_degree_map();
      // Multiply Tᵢ(x) to each term Tⱼ(y)Tₖ(z) to get the new
      // ChebyshevBasisElement.
      new_chebyshev_var_to_degree_map.emplace_hint(
          new_chebyshev_var_to_degree_map.end(), var_first, degree_x);
      // Multiply cᵢ to dⱼₖ to get the new coefficient.
      const double new_coeff = coeff_remaining_vars * coeff_x;
      result.emplace(ChebyshevBasisElement(new_chebyshev_var_to_degree_map),
                     new_coeff);
    }
  }
  return result;
}
}  // namespace

std::map<ChebyshevBasisElement, double> MonomialBasisElement::ToChebyshevBasis()
    const {
  return MonomialToChebyshevBasisRecursive(var_to_degree_map());
}

void MonomialBasisElement::MergeBasisElementInPlace(
    const MonomialBasisElement& other) {
  this->DoMergeBasisElementInPlace(other);
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
