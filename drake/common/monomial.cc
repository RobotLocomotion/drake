#include "drake/common/monomial.h"

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"
#include "drake/common/symbolic_expression_visitor.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::map;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::unordered_map;

namespace internal {
Monomial::Monomial() : total_degree_{0}, powers_{} {}

Monomial::Monomial(const Variable& var, const int exponent)
    : total_degree_{exponent} {
  DRAKE_DEMAND(exponent >= 0);
  if (exponent > 0) {
    powers_.emplace(var.get_id(), exponent);
  }
}

Monomial::Monomial(const map<Variable::Id, int>& powers)
    : total_degree_{TotalDegree(powers)}, powers_{} {
  for (const auto& p : powers) {
    if (p.second > 0) {
      powers_.insert(p);
    } else if (p.second < 0) {
      throw std::runtime_error("The exponent is negative.");
    }
  }
}

// Forward declaration.
map<Variable::Id, int> ToMonomialPower(const Expression& e);

Monomial::Monomial(const Expression& e) : Monomial(ToMonomialPower(e)) {}

size_t Monomial::GetHash() const {
  // To get a hash value for a Monomial, we re-use the hash value for
  // powers_. This is suitable because powers_ is the only independent
  // data-member of Monomial class while another data-member, total_degree_ is
  // determined by a given powers_.
  return hash_value<map<Variable::Id, int>>{}(powers_);
}

bool Monomial::operator==(const Monomial& m) const {
  return powers_ == m.powers_;
}

Expression Monomial::ToExpression(
    const unordered_map<Variable::Id, Variable>& id_to_var_map) const {
  // It builds this base_to_exponent_map and uses ExpressionMulFactory to build
  // a multiplication expression.
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : powers_) {
    const Variable::Id id{p.first};
    const int exponent{p.second};
    const auto it = id_to_var_map.find(id);
    if (it != id_to_var_map.end()) {
      const Variable& var{it->second};
      base_to_exponent_map.emplace(Expression{var}, exponent);
    } else {
      ostringstream oss;
      oss << "Variable whose ID is " << id << " appeared in a monomial "
          << *this << "." << std::endl
          << "However, Monomial::ToExpression method "
             "fails to find the corresponding Variable in a given map.";
      throw runtime_error(oss.str());
    }
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

int Monomial::TotalDegree(const map<Variable::Id, int>& powers) {
  return accumulate(powers.begin(), powers.end(), 0,
                    [](const int degree, const pair<Variable::Id, int>& p) {
                      return degree + p.second;
                    });
}

std::ostream& operator<<(std::ostream& out, const Monomial& m) {
  out << "{";
  for (const auto& power : m.powers_) {
    out << "(" << power.first << ", " << power.second << ") ";
  }
  return out << "}";
}

Monomial operator*(const Monomial& m1, const Monomial& m2) {
  map<Variable::Id, int> powers{m1.get_powers()};
  for (const pair<Variable::Id, int>& p : m2.get_powers()) {
    const Variable::Id var{p.first};
    const int exponent{p.second};
    auto it = powers.find(var);
    if (it == powers.end()) {
      powers.insert(p);
    } else {
      it->second += exponent;
    }
  }
  return Monomial{powers};
}

map<Variable::Id, int> ToMonomialPower(const Expression& e) {
  DRAKE_DEMAND(e.is_polynomial());
  map<Variable::Id, int> powers;
  if (is_one(e)) {  // This block is deliberately left empty.
  } else if (is_constant(e)) {
    throw runtime_error(
        "A constant not equal to to 1, this is not a monomial.");
  } else if (is_variable(e)) {
    powers.emplace(get_variable(e).get_id(), 1);
  } else if (is_pow(e)) {
    const auto& exponent = get_second_argument(e);
    if (!is_constant(exponent)) {
      throw runtime_error("The exponent is not a constant.");
    }
    auto exponent_val = get_constant_value(exponent);
    powers = ToMonomialPower(get_first_argument(e));
    // pow( pow(xᵢ, kᵢ), n) = pow(xᵢ, kᵢ * n)
    for (auto& p : powers) {
      p.second *= exponent_val;
    }
  } else if (is_multiplication(e)) {
    if (!is_one(get_constant_in_multiplication(e))) {
      throw runtime_error("The constant in the multiplication is not 1.");
    }
    for (const auto& p : get_base_to_exponent_map_in_multiplication(e)) {
      map<Variable::Id, int> p_powers = ToMonomialPower(pow(p.first, p.second));
      for (const auto& q : p_powers) {
        auto it = powers.find(q.first);
        if (it == powers.end()) {
          powers.emplace(q.first, q.second);
        } else {
          it->second += q.second;
        }
      }
    }
  } else {
    throw runtime_error("This expression cannot be converted to a monomial.");
  }
  return powers;
}

/* Internally we use internal::Monomial to represent a monomial, rather than
 * using an Expression object.
 */
typedef std::unordered_map<Monomial, Expression, hash_value<Monomial>>
    MonomialToCoefficientMapInternal;

/**
 * Adds a term to the polynomial.
 * Find if the monomial in the new term exists in the polynomial or not. If it
 * does, then increment the corresponding coefficient. Otherwise add a new
 * pair (monomial, coefficient) to the map.
 * @param monomial The monomial in the new term.
 * @param coefficient The coefficient in the new term.
 * @param polynomial The polynomial that the new term is added to.
 */
void AddTermToPolynomial(const Monomial& monomial,
                         const Expression& coefficient,
                         MonomialToCoefficientMapInternal* polynomial) {
  auto it = polynomial->find(monomial);
  if (it == polynomial->end()) {
    polynomial->emplace_hint(it, monomial, coefficient);
  } else {
    it->second += coefficient;
  }
}

/**
 * For a polynomial e = c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ), compute
 * the square of the polynomial.
 * Note that x and kᵢ can both be vectors, for example x = (x₀, x₁),
 * kᵢ = (1, 2), then pow(x, kᵢ) = x₀x₁²
 * @param map maps the monomial in the input polynomial to its coefficient.
 * @return maps the monomial in the output polynomial to its coefficient.
 */
MonomialToCoefficientMapInternal PolynomialSqaure(
    const MonomialToCoefficientMapInternal& map) {
  MonomialToCoefficientMapInternal map_square;
  map_square.reserve(map.size() * (map.size() + 1) / 2);
  for (auto it1 = map.begin(); it1 != map.end(); ++it1) {
    for (auto it2 = it1; it2 != map.end(); ++it2) {
      Monomial new_monomial = it1->first * it2->first;
      Expression new_coeff = it1->second * it2->second;
      if (it1 != it2) {
        // Two cross terms.
        new_coeff *= 2;
      }
      AddTermToPolynomial(new_monomial, new_coeff, &map_square);
    }
  }
  return map_square;
}

class DecomposePolynomialVisitor {
 public:
  // `vars` is a const set of variables, that will not be changed. It stays
  // the same during visiting each type of symbolic expressions.
  MonomialToCoefficientMapInternal Visit(const Expression& e,
                                         const Variables& vars) const {
    return VisitPolynomial<MonomialToCoefficientMapInternal>(*this, e, vars);
  }

  MonomialToCoefficientMapInternal operator()(
      const shared_ptr<ExpressionVar>& e, const Variables& vars) const {
    const auto& var = e->get_variable();
    Expression coeff{};
    int exponent{};
    if (vars.include(var)) {
      exponent = 1;
      coeff = 1;
    } else {
      exponent = 0;
      coeff = var;
    }
    return MonomialToCoefficientMapInternal({{Monomial(var, exponent), coeff}});
  }

  MonomialToCoefficientMapInternal operator()(
      const shared_ptr<ExpressionConstant>& e, const Variables& vars) const {
    return MonomialToCoefficientMapInternal({{Monomial(), e->get_value()}});
  }

  MonomialToCoefficientMapInternal operator()(
      const shared_ptr<ExpressionAdd>& e, const Variables& vars) const {
    MonomialToCoefficientMapInternal map;
    double e_constant = e->get_constant();
    if (e_constant != 0) {
      map.emplace(Monomial(), e_constant);
    }
    // For an expression 2*(3*x*y+4*x) + 4*y.
    // expr_to_coeff_map_[3*x*y+4*x] = 2
    // expr_to_coeff_map_[y] = 4
    for (const auto& p : e->get_expr_to_coeff_map()) {
      // For expr_to_coeff_map_[3*x*y+4*x] = 2
      // map_p[x*y] = 3
      // map_p[x] = 4
      const auto& map_p = Visit(p.first, vars);
      for (const auto& map_p_pair : map_p) {
        const Monomial& p_monomial = map_p_pair.first;
        const Expression& p_coefficient = map_p_pair.second;
        // a * (b * monomial) = (a * b) * monomial.
        AddTermToPolynomial(p_monomial, p_coefficient * p.second, &map);
      }
    }
    return map;
  }

  MonomialToCoefficientMapInternal operator()(const shared_ptr<ExpressionMul> e,
                                              const Variables& vars) const {
    MonomialToCoefficientMapInternal map;
    // We iterate through base_to_exponent_map
    // Suppose e = pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ, pₖ)
    // We first decompose the first k-1 products
    // pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ₋₁, pₖ₋₁) as a polynomial
    // (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ))
    // and the last term pow(eₖ, pₖ) as another polynomial
    // (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // And then multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x, kⱼ)
    // to get each monomial in the product.
    // TODO(hongkai.dai):
    // Alternatively, we can do divide and conquer here.
    // for an expression e = pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ, pₖ)
    // First decomposes the first ⌊k/2⌋ products, then decomposes the last
    // ⌈k/2⌉ products, so as to write e as the product of two polynomials
    // e = (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ)) *
    //     (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // Finally multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x, kⱼ)
    // to get each monomial in the product.
    // The divide and conquer approach requires splitting the map
    // base_to_exponent_map to two halves, which can be inefficient, so we do
    // not implement this approach.
    const auto& base_to_exponent_map = e->get_base_to_exponent_map();
    for (const auto& p : base_to_exponent_map) {
      if (map.empty()) {
        map = Visit(pow(p.first, p.second), vars);
      } else {
        const auto& map_p = Visit(pow(p.first, p.second), vars);
        MonomialToCoefficientMapInternal map_product;
        map_product.reserve(map.size() * map_p.size());
        // Now multiply each term in map, with each term in map_p.
        for (const auto& term_map : map) {
          for (const auto& term_map_p : map_p) {
            Monomial new_monomial = term_map.first * term_map_p.first;
            Expression new_coeff = term_map.second * term_map_p.second;
            AddTermToPolynomial(new_monomial, new_coeff, &map_product);
          }
        }
        map = std::move(map_product);
      }
    }
    // Finally multiply the constant coefficient.
    for (auto& p : map) {
      p.second *= e->get_constant();
    }
    return map;
  }

  MonomialToCoefficientMapInternal operator()(
      const shared_ptr<ExpressionPow>& e, const Variables& vars) const {
    // We use a divide and conquer approach here
    // pow(e, p) can be computed as pow(e, ⌊p/2⌋) * pow(e, ⌈p/2⌉)
    // We can decompose the first term as a polynomial
    // (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ))
    // and the second term as another polynomial
    // (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // We then multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x, kⱼ)
    // to get each monomial in the product.
    MonomialToCoefficientMapInternal map;
    const int exponent{
        static_cast<int>(get_constant_value(e->get_second_argument()))};
    if (exponent == 1) {
      return Visit(e->get_first_argument(), vars);
    } else if (exponent % 2 == 0) {
      // compute the square of a polynomial (c₀ + c₁ * pow(x, k₁) + ... + cₙ *
      // pow(x, kₙ)).
      const auto& map1 = Visit(
          pow(e->get_first_argument(), exponent / 2), vars);
      map = PolynomialSqaure(map1);
    } else {
      // For expression pow(e, k) with odd exponent k, compute
      // e1 = pow(e, ⌊k/2⌋) first, and then compute the square of e1, finally
      // multiply the squared result with e.
      const auto& map1 = Visit(
          pow(e->get_first_argument(), exponent / 2), vars);
      const auto& map1_square = PolynomialSqaure(map1);
      const auto& map2 = Visit(
          e->get_first_argument(), vars);
      map.reserve(map1_square.size() * map2.size());
      for (const auto& p1 : map1_square) {
        for (const auto& p2 : map2) {
          Monomial new_monomial = p1.first * p2.first;
          Expression new_coeff = p1.second * p2.second;
          AddTermToPolynomial(new_monomial, new_coeff, &map);
        }
      }
    }
    return map;
  }

  MonomialToCoefficientMapInternal operator()(const shared_ptr<ExpressionDiv> e,
                                              const Variables& vars) const {
    // Currently we can only handle the case of a monomial as the divisor.
    const auto& map1 =
        Visit(e->get_first_argument(), vars);
    const auto& map2 =
        Visit(e->get_second_argument(), vars);
    if (map2.size() != 1) {
      throw std::runtime_error(
          "The divisor is not a monomial. The Div expression cannot be "
          "decomposed as a polynomial.");
    }
    const auto& divisor_monomial = map2.begin()->first;
    const auto& divisor_monomial_powers = divisor_monomial.get_powers();
    const Expression& divisor_coeff = map2.begin()->second;
    MonomialToCoefficientMapInternal map;
    map.reserve(map1.size());
    for (const auto& p1 : map1) {
      // For each monomial in the dividend, compute the division from the
      // dividend monomial by the divisor monomial.
      const internal::Monomial dividend_monomial(p1.first);
      std::map<Variable::Id, int> division_monomial_powers =
          dividend_monomial.get_powers();
      for (const auto& p_divisor : divisor_monomial_powers) {
        // The variable in divisor has to appear in the dividend.
        auto it = division_monomial_powers.find(p_divisor.first);
        if (it == division_monomial_powers.end()) {
          throw std::runtime_error(
              "The variable in the divisor is not in the dividend.");
        } else {
          // xⁿ / xᵐ = xⁿ⁻ᵐ
          it->second -= p_divisor.second;
        }
      }
      Monomial division_monomial(division_monomial_powers);

      map.emplace(division_monomial, p1.second / divisor_coeff);
    }
    return map;
  }
};

MonomialToCoefficientMapInternal DecomposePolynomialInternal(
    const Expression& e, const Variables& vars) {
  DRAKE_DEMAND(e.is_polynomial());
  return DecomposePolynomialVisitor().Visit(e, vars);
}
}  // namespace internal

class DegreeVisitor {
 public:
  int Visit(const Expression& e, const Variables& vars) const {
    return VisitPolynomial<int>(*this, e, vars);
  }

  int operator()(const shared_ptr<ExpressionVar>& e,
                 const Variables& vars) const {
    return vars.include(e->get_variable()) ? 1 : 0;
  }

  int operator()(const shared_ptr<ExpressionConstant>& e,
                 const Variables& vars) const {
    return 0;
  }

  int operator()(const shared_ptr<ExpressionAdd>& e,
                 const Variables& vars) const {
    int degree = 0;
    for (const auto& p : e->get_expr_to_coeff_map()) {
      degree = std::max(degree, Visit(p.first, vars));
    }
    return degree;
  }

  int operator()(const shared_ptr<ExpressionMul>& e,
                 const Variables& vars) const {
    return accumulate(
        e->get_base_to_exponent_map().begin(),
        e->get_base_to_exponent_map().end(), 0,
        [this, &vars](const int& degree,
                      const pair<Expression, Expression>& p) {
          const Expression& base{p.first};
          const Expression& exponent{p.second};
          return degree +
                 Visit(base, vars) *
                     static_cast<int>(get_constant_value(exponent));
        });
  }

  int operator()(const shared_ptr<ExpressionDiv>& e,
                 const Variables& vars) const {
    return Visit(e->get_first_argument(), vars) -
           Visit(e->get_second_argument(), vars);
  }

  int operator()(const shared_ptr<ExpressionPow>& e,
                 const Variables& vars) const {
    const int exponent{
        static_cast<int>(get_constant_value(e->get_second_argument()))};
    return Visit(e->get_first_argument(), vars) * exponent;
  }
};

int Degree(const Expression& e, const Variables& vars) {
  return DegreeVisitor().Visit(e, vars);
}

int Degree(const Expression& e) { return Degree(e, e.GetVariables()); }

Expression GetMonomial(const unordered_map<Variable, int, hash_value<Variable>>&
                           map_var_to_exponent) {
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : map_var_to_exponent) {
    DRAKE_DEMAND(p.second > 0);
    base_to_exponent_map.emplace(Expression{p.first}, p.second);
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

Eigen::Matrix<Expression, Eigen::Dynamic, 1> MonomialBasis(
    const Variables& vars, const int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}

MonomialToCoefficientMap DecomposePolynomial(const Expression& e,
                                             const Variables& vars) {
  const auto& map_internal = internal::DecomposePolynomialInternal(e, vars);
  std::unordered_map<Variable::Id, Variable> map_id_to_var;
  map_id_to_var.reserve(vars.size());
  for (const auto& v : vars) {
    map_id_to_var.emplace(v.get_id(), v);
  }
  MonomialToCoefficientMap map;
  map.reserve(map_internal.size());
  for (const auto& p : map_internal) {
    map.emplace(p.first.ToExpression(map_id_to_var), p.second);
  }
  return map;
}

MonomialToCoefficientMap DecomposePolynomial(const Expression& e) {
  return DecomposePolynomial(e, e.GetVariables());
}
}  // namespace symbolic
}  // namespace drake
