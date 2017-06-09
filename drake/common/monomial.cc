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
using std::make_pair;
using std::map;
using std::ostream;
using std::ostringstream;
using std::out_of_range;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::unordered_map;

namespace {
// Computes the total degree of a monomial. This method is used in a
// constructor of Monomial to set its total degree at construction.
int TotalDegree(const map<Variable, int>& powers) {
  return accumulate(powers.begin(), powers.end(), 0,
                    [](const int degree, const pair<Variable, int>& p) {
                      return degree + p.second;
                    });
}

// Converts a symbolic expression @p e into an internal representation of
// Monomial class, a mapping from a base (Variable) to its exponent (int). This
// function is called inside of the constructor Monomial(const
// symbolic::Expression&).
map<Variable, int> ToMonomialPower(const Expression& e) {
  // TODO(soonho): Re-implement this function by using a Polynomial visitor.
  DRAKE_DEMAND(e.is_polynomial());
  map<Variable, int> powers;
  if (is_one(e)) {  // This block is deliberately left empty.
  } else if (is_constant(e)) {
    throw runtime_error("A constant not equal to 1, this is not a monomial.");
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
      throw runtime_error("The constant in the multiplication is not 1.");
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
    throw runtime_error("This expression cannot be converted to a monomial.");
  }
  return powers;
}
}  // namespace

Monomial::Monomial(const Variable& var, const int exponent)
    : total_degree_{exponent} {
  DRAKE_DEMAND(exponent >= 0);
  if (exponent > 0) {
    powers_.emplace(var, exponent);
  }
}

Monomial::Monomial(const map<Variable, int>& powers)
    : total_degree_{TotalDegree(powers)} {
  for (const auto& p : powers) {
    const int exponent{p.second};
    if (exponent > 0) {
      powers_.insert(p);
    } else if (exponent < 0) {
      throw std::runtime_error("The exponent is negative.");
    }
    // Ignore the entry if exponent == 0.
  }
}

Monomial::Monomial(const Expression& e)
    : Monomial(ToMonomialPower(e.Expand())) {}

size_t Monomial::GetHash() const {
  // To get a hash value for a Monomial, we re-use the hash value for
  // powers_. This is suitable because powers_ is the only independent
  // data-member of Monomial class while another data-member, total_degree_ is
  // determined by a given powers_.
  return hash_value<map<Variable, int>>{}(powers_);
}

bool Monomial::operator==(const Monomial& m) const {
  // TODO(soonho-tri): simplify this function to `return powers_ == m.powers_`.
  //
  // For now, the above one-liner doesn't work. std::map<Variable,
  // int>::operator== uses Variable::operator== to compare two keys (two
  // Variables) instead of std::equal_to<Variable>. The consequence is that we
  // end up with var_1 == var_2, which turns to a symbolic::Formula and calls
  // symbolic::Formula::Evaluate with an empty
  // environment. symbolic::Formula::Evaluate function will throw a
  // runtime_error because it cannot find an entry for `var_1` (nor `var_2`) in
  // an empty environment.
  //
  // I think the right approach is to refactor symbolic::Formula::Evaluate to
  // use structural equality when it handles equality (==) and inequality (!=)
  // after partially evaluating/substituting with a given environment. When this
  // fix is landed, I'll simplify this function.
  if (powers_.size() != m.powers_.size()) {
    return false;
  }
  for (const pair<Variable, int> p : powers_) {
    const Variable& var{p.first};
    const int exponent{p.second};
    const auto it = m.powers_.find(var);
    if (it == m.powers_.end()) {
      // Key var is not found in m.powers.
      return false;
    }
    if (exponent != it->second) {
      return false;
    }
  }
  return true;
}

bool Monomial::operator!=(const Monomial& m) const { return !(*this == m); }

double Monomial::Evaluate(const Environment& env) const {
  return accumulate(powers_.begin(), powers_.end(), 1.0,
                    [this, &env](const double v, const pair<Variable, int>& p) {
                      const Variable& var{p.first};
                      const auto it = env.find(var);
                      if (it == env.end()) {
                        ostringstream oss;
                        oss << "Monomial " << *this
                            << " cannot be evaluated with the given "
                               "environment which does not provide an entry "
                               "for variable = "
                            << var << ".";
                        throw out_of_range(oss.str());
                      } else {
                        const double base{it->second};
                        const int exponent{p.second};
                        return v * std::pow(base, exponent);
                      }
                    });
}

pair<double, Monomial> Monomial::Substitute(const Environment& env) const {
  double coeff{1.0};
  map<Variable, int> new_powers;
  for (const auto& p : powers_) {
    const Variable& var{p.first};
    const int exponent{p.second};
    auto it = env.find(var);
    if (it != env.end()) {
      double base{it->second};
      coeff *= std::pow(base, exponent);
    } else {
      new_powers.insert(p);
    }
  }
  return make_pair(coeff, Monomial(new_powers));
}

Expression Monomial::ToExpression() const {
  // It builds this base_to_exponent_map and uses ExpressionMulFactory to build
  // a multiplication expression.
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : powers_) {
    const Variable& var{p.first};
    const int exponent{p.second};
    base_to_exponent_map.emplace(Expression{var}, exponent);
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

Monomial& Monomial::operator*=(const Monomial& m) {
  for (const auto& p : m.get_powers()) {
    const Variable& var{p.first};
    const int exponent{p.second};
    auto it = powers_.find(var);
    if (it == powers_.end()) {
      powers_.insert(p);
    } else {
      it->second += exponent;
    }
    total_degree_ += exponent;
  }
  return *this;
}

Monomial& Monomial::pow_in_place(const int p) {
  if (p < 0) {
    ostringstream oss;
    oss << "Monomial::pow(int p) is called with a negative p = " << p;
    throw runtime_error(oss.str());
  }
  if (p == 0) {
    total_degree_ = 0;
    powers_.clear();
  } else if (p > 1) {
    for (auto& item : powers_) {
      int& exponent{item.second};
      exponent *= p;
      total_degree_ += (exponent * (p - 1));
    }
  }  // If p == 1, NO OP.
  return *this;
}

std::ostream& operator<<(std::ostream& out, const Monomial& m) {
  out << "{";
  for (const auto& power : m.powers_) {
    out << "(" << power.first << ", " << power.second << ") ";
  }
  return out << "}";
}

Monomial operator*(Monomial m1, const Monomial& m2) {
  m1 *= m2;
  return m1;
}

Monomial pow(Monomial m, const int p) { return m.pow_in_place(p); }

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
                         MonomialToCoefficientMap* polynomial) {
  auto it = polynomial->find(monomial);
  if (it == polynomial->end()) {
    polynomial->emplace_hint(it, monomial, coefficient);
  } else {
    Expression new_coeff = it->second + coefficient;
    if (is_zero(new_coeff)) {
      polynomial->erase(it);
    } else {
      it->second = new_coeff;
    }
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
MonomialToCoefficientMap PolynomialSqaure(const MonomialToCoefficientMap& map) {
  MonomialToCoefficientMap map_square;
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
  MonomialToCoefficientMap Visit(const Expression& e,
                                 const Variables& vars) const {
    return VisitPolynomial<MonomialToCoefficientMap>(*this, e, vars);
  }

  MonomialToCoefficientMap operator()(const shared_ptr<ExpressionVar>& e,
                                      const Variables& vars) const {
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
    return MonomialToCoefficientMap({{Monomial(var, exponent), coeff}});
  }

  MonomialToCoefficientMap operator()(const shared_ptr<ExpressionConstant>& e,
                                      const Variables&) const {
    if (e->get_value() != 0) {
      return MonomialToCoefficientMap({{Monomial(), e->get_value()}});
    }
    return MonomialToCoefficientMap();
  }

  MonomialToCoefficientMap operator()(const shared_ptr<ExpressionAdd>& e,
                                      const Variables& vars) const {
    MonomialToCoefficientMap map;
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

  MonomialToCoefficientMap operator()(const shared_ptr<ExpressionMul>& e,
                                      const Variables& vars) const {
    MonomialToCoefficientMap map;
    // We iterate through base_to_exponent_map
    // Suppose e = pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ, pₖ)
    // We first decompose the first k-1 products
    // pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ₋₁, pₖ₋₁) as a polynomial
    // (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ))
    // and the last term pow(eₖ, pₖ) as another polynomial
    // (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // And then multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x,
    // kⱼ) to get each monomial in the product.
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
        MonomialToCoefficientMap map_product;
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

  MonomialToCoefficientMap operator()(const shared_ptr<ExpressionPow>& e,
                                      const Variables& vars) const {
    // We use a divide and conquer approach here
    // pow(e, p) can be computed as pow(e, ⌊p/2⌋) * pow(e, ⌈p/2⌉)
    // We can decompose the first term as a polynomial
    // (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ))
    // and the second term as another polynomial
    // (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // We then multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x, kⱼ)
    // to get each monomial in the product.
    MonomialToCoefficientMap map;
    const int exponent{
        static_cast<int>(get_constant_value(e->get_second_argument()))};
    if (exponent == 1) {
      return Visit(e->get_first_argument(), vars);
    }
    if (exponent % 2 == 0) {
      // compute the square of a polynomial (c₀ + c₁ * pow(x, k₁) + ... + cₙ *
      // pow(x, kₙ)).
      const auto& map1 =
          Visit(pow(e->get_first_argument(), exponent / 2), vars);
      map = PolynomialSqaure(map1);
    } else {
      // For expression pow(e, k) with odd exponent k, compute
      // e1 = pow(e, ⌊k/2⌋) first, and then compute the square of e1, finally
      // multiply the squared result with e.
      const auto& map1 =
          Visit(pow(e->get_first_argument(), exponent / 2), vars);
      const auto& map1_square = PolynomialSqaure(map1);
      const auto& map2 = Visit(e->get_first_argument(), vars);
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

  MonomialToCoefficientMap operator()(const shared_ptr<ExpressionDiv>& e,
                                      const Variables& vars) const {
    // Currently we can only handle the case of a monomial as the divisor.
    const auto& map1 = Visit(e->get_first_argument(), vars);
    const auto& map2 = Visit(e->get_second_argument(), vars);
    if (map2.size() != 1) {
      throw std::runtime_error(
          "The divisor is not a monomial. The Div expression cannot be "
          "decomposed as a polynomial.");
    }
    const auto& divisor_monomial = map2.begin()->first;
    const auto& divisor_monomial_powers = divisor_monomial.get_powers();
    const Expression& divisor_coeff = map2.begin()->second;
    MonomialToCoefficientMap map;
    map.reserve(map1.size());
    for (const auto& p1 : map1) {
      // For each monomial in the dividend, compute the division from the
      // dividend monomial by the divisor monomial.
      const Monomial dividend_monomial(p1.first);
      std::map<Variable, int> division_monomial_powers =
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

MonomialToCoefficientMap DecomposePolynomialIntoMonomial(
    const Expression& e, const Variables& vars) {
  DRAKE_DEMAND(e.is_polynomial());
  MonomialToCoefficientMap map = DecomposePolynomialVisitor().Visit(e, vars);
  // Now loops through the map to remove the term with zero coefficient.
  for (auto it = map.begin(); it != map.end();) {
    bool is_zero_term = false;
    DRAKE_DEMAND(it->second.is_polynomial());
    if (!is_constant(it->second)) {
      // If the coefficient it->second is a polynomial, then determine if it
      // is a zero polynomial, by decomposing it->second into monomials,
      // and check if the constant coefficient for each term is zero.
      MonomialToCoefficientMap coeff_map = DecomposePolynomialVisitor().Visit(
          it->second, it->second.GetVariables());
      is_zero_term = true;
      for (const auto& p : coeff_map) {
        DRAKE_DEMAND(is_constant(p.second));
        if (!is_zero(p.second)) {
          is_zero_term = false;
          break;
        }
      }
    } else {
      // If the coefficient is a constant, then it cannot be zero, since we
      // have deleted term with zero constant coefficient already.
      DRAKE_ASSERT(!is_zero(it->second));
    }
    if (is_zero_term) {
      it = map.erase(it);
    } else {
      ++it;
    }
  }
  return map;
}

class DegreeVisitor {
 public:
  int Visit(const Expression& e, const Variables& vars) const {
    return VisitPolynomial<int>(*this, e, vars);
  }

  int operator()(const shared_ptr<ExpressionVar>& e,
                 const Variables& vars) const {
    return vars.include(e->get_variable()) ? 1 : 0;
  }

  int operator()(const shared_ptr<ExpressionConstant>&,
                 const Variables&) const {
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
          return degree + Visit(base, vars) *
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
  return ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}

MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
    const Expression& e, const Variables& vars) {
  const auto& map_internal = DecomposePolynomialIntoMonomial(e, vars);
  MonomialAsExpressionToCoefficientMap map;
  map.reserve(map_internal.size());
  for (const auto& p : map_internal) {
    map.emplace(p.first.ToExpression(), p.second);
  }
  return map;
}

MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
    const Expression& e) {
  return DecomposePolynomialIntoExpression(e, e.GetVariables());
}
}  // namespace symbolic
}  // namespace drake
