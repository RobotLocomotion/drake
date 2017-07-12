#include "drake/common/symbolic_polynomial.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "drake/common/symbolic_expression_visitor.h"

using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;

namespace drake {
namespace symbolic {

namespace {
// Helper function to add coeff * m to map (Monomial → Expression).
// Used in DecomposePolynomialVisitor::VisitAddition and Polynomial::Add.
void DoAdd(const Expression& coeff, const Monomial& m,
           Polynomial::MapType* const map) {
  auto it = map->find(m);
  if (it != map->end()) {
    // m ∈ dom(map)
    Expression& existing_coeff{it->second};
    // Note that `.Expand()` is needed in the following line. For example,
    // consider the following case:
    //     c1 := (a + b)²
    //     c2 := - (a² + 2ab + b²)
    // Without expanding the terms, we have `is_zero(c1 + c2) = false` while
    // it's clear that c1 + c2 is a zero polynomial. Using `Expand()` help us
    // identify those cases.
    if (is_zero(existing_coeff.Expand() + coeff.Expand())) {
      map->erase(it);
    } else {
      existing_coeff += coeff;
    }
  } else {
    // m ∉ dom(map)
    map->emplace_hint(it, m, coeff);
  }
}
// Visitor class to implement `Polynomial(const Expression& e, const
// Variables& indeterminates)` constructor which decomposes an expression e
// w.r.t. indeterminates.
class DecomposePolynomialVisitor {
 public:
  Polynomial::MapType Decompose(const Expression& e,
                                const Variables& vars) const {
    // Note that it calls `Expression::Expand()` here.
    return Visit(e.Expand(), vars);
  }

 private:
  Polynomial::MapType Visit(const Expression& e, const Variables& vars) const {
    return VisitPolynomial<Polynomial::MapType>(this, e, vars);
  }

  Polynomial::MapType VisitVariable(const Expression& e,
                                    const Variables& vars) const {
    const Variable& var{get_variable(e)};
    if (vars.include(var)) {
      // Monomial : var, coefficient : 1
      return Polynomial::MapType{{{Monomial{var}, 1}}};
    } else {
      // Monomial : 1, coefficient : var
      return Polynomial::MapType{{{Monomial{}, var}}};
    }
  }

  Polynomial::MapType VisitConstant(const Expression& e,
                                    const Variables&) const {
    const double v{get_constant_value(e)};
    if (v != 0) {
      return Polynomial::MapType{{{Monomial(), v}}};  // = v.
    }
    return Polynomial::MapType{};  // = 0.
  }

  Polynomial::MapType VisitAddition(const Expression& e,
                                    const Variables& vars) const {
    // e = c₀ + ∑ᵢ (cᵢ * eᵢ)
    Polynomial::MapType new_map;
    const double c_0{get_constant_in_addition(e)};
    if (c_0 != 0) {
      new_map.emplace(Monomial{}, c_0);
    }
    for (const pair<Expression, double>& p :
         get_expr_to_coeff_map_in_addition(e)) {
      const Expression& e_i{p.first};
      const double c_i{p.second};
      // e = c₀ + ∑ᵢ (cᵢ * eᵢ) = c₀ + ∑ᵢ (cᵢ * (∑ⱼ mⱼ * cⱼ))
      //                                   ~~~~~~~~~~~
      //                                  Monomial of eᵢ
      //                     = c₀ + ∑ᵢ ∑ⱼ ((cᵢ * cⱼ) * mⱼ)
      // Note that we have cᵢ ≠ 0 ∧ cⱼ ≠ 0 → (cᵢ * cⱼ) ≠ 0.
      const Polynomial::MapType map_i = Visit(e_i, vars);
      for (const pair<Monomial, Expression>& term : map_i) {
        const Monomial& m_j{term.first};
        const Expression& c_j{term.second};
        // Add (cᵢ * cⱼ) * mⱼ.
        DoAdd(c_i * c_j, m_j, &new_map);
      }
    }
    return new_map;
  }

  Polynomial::MapType VisitMultiplication(const Expression& e,
                                          const Variables& vars) const {
    // e = c * ∏ᵢ pow(baseᵢ, exponentᵢ).
    const double c = get_constant_in_multiplication(e);
    Expression coeff{c};
    Monomial m{};
    for (const pair<Expression, Expression>& p :
         get_base_to_exponent_map_in_multiplication(e)) {
      const Expression& base_i{p.first};
      const Expression& exponent_i{p.second};
      const Polynomial::MapType map_i{Visit(pow(base_i, exponent_i), vars)};
      DRAKE_DEMAND(map_i.size() == 1);
      const Monomial& m_i{map_i.begin()->first};
      const Expression& coeff_i{map_i.begin()->second};
      m *= m_i;
      coeff *= coeff_i;
    }
    return Polynomial::MapType{{m, coeff}};
  }

  Polynomial::MapType VisitPow(const Expression& e,
                               const Variables& vars) const {
    // e = pow(base, exponent)
    //   = pow(∏ᵢ varᵢ, n)          where n is of integer.
    // It holds because e is an expanded polynomial.
    const Expression& base{get_first_argument(e)};
    const Expression& exponent{get_second_argument(e)};
    DRAKE_DEMAND(is_constant(exponent));
    // The following static_cast<int> does not lose information because of the
    // preconditions.
    const int n{static_cast<int>(get_constant_value(exponent))};
    Expression coeff{1.0};
    Monomial m{};
    DRAKE_DEMAND(is_variable(base) || is_multiplication(base));
    for (const Variable& var : base.GetVariables()) {
      if (vars.include(var)) {
        m *= Monomial{var, n};
      } else {
        coeff *= pow(Expression{var}, exponent);
      }
    }
    return Polynomial::MapType{{m, coeff}};
  }

  Polynomial::MapType VisitDivision(const Expression&, const Variables&) const {
    // Because e.is_polynomial() is true. We have e = e₁ / c. Since e is already
    // expanded, e₁ is expanded as well. With those preconditions, the following
    // case analysis concludes that this method is not called in run-time.
    //
    //  - e = c₁ / c
    //    (This case should've been constant-folded so e₁ is not a constant.)
    //
    //  - e = (c₀ + ∑ᵢ(cᵢ * eᵢ)) / c
    //      = c₀/c + ∑ᵢ(cᵢ/c * eᵢ)     // By Expand()
    //    (As a result, e is not a division at the top level.)
    //
    //  - e = (c₀ * ∏ᵢ(pow(e₁ᵢ, e₂ᵢ)) / c
    //      = c₀/c * ∏ᵢ(pow(e₁ᵢ, e₂ᵢ)  // By Expand()
    //    (As a result, e is not a division at the top level.)
    //
    //  - e = (e' / c') / c
    //      = e' / (c' * c)      // Simplified in construction.
    //      = (1 / c' * c) * e'  // Expand()
    //    (As a result, e is not a division at the top level.)
    //
    //  - e = v / c
    //      = (1 / c) * v  // By Expand()
    //    (As a result, e is not a division at the top level.)
    //
    //  - e = pow(base, exponent) / c
    //      = (1 / c) * pow(base, exponent)  // By Expand()
    //    (As a result, e is not a division at the top level.)
    //
    throw runtime_error(
        "This should not be reachable because of preconditions.");
  }

  // Makes VisitPolynomial a friend of this class so that it can use private
  // methods.
  friend Polynomial::MapType
  drake::symbolic::VisitPolynomial<Polynomial::MapType>(
      const DecomposePolynomialVisitor*, const Expression&, const Variables&);
};
}  // namespace

Polynomial::Polynomial(MapType init)
    : monomial_to_coefficient_map_{move(init)} {
  CheckInvariant();
};

Polynomial::Polynomial(const Monomial& m)
    : monomial_to_coefficient_map_{{m, 1}} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

Polynomial::Polynomial(const Expression& e) : Polynomial{e, e.GetVariables()} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

Polynomial::Polynomial(const Expression& e, const Variables& indeterminates)
    : monomial_to_coefficient_map_{
          DecomposePolynomialVisitor{}.Decompose(e, indeterminates)} {
  // No need to call CheckInvariant() because DecomposePolynomialVisitor is
  // supposed to make sure the invariant holds as a post-condition.
}

Variables Polynomial::indeterminates() const {
  Variables vars;
  for (const pair<Monomial, Expression>& p : monomial_to_coefficient_map_) {
    const Monomial& m_i{p.first};
    vars += m_i.GetVariables();
  }
  return vars;
}

Variables Polynomial::decision_variables() const {
  Variables vars;
  for (const pair<Monomial, Expression>& p : monomial_to_coefficient_map_) {
    const Expression& e_i{p.second};
    vars += e_i.GetVariables();
  }
  return vars;
}

int Polynomial::Degree(const Variable& v) const {
  int degree{0};
  for (const pair<Monomial, Expression>& p : monomial_to_coefficient_map_) {
    const Monomial& m{p.first};
    degree = std::max(degree, m.degree(v));
  }
  return degree;
}

int Polynomial::TotalDegree() const {
  int degree{0};
  for (const pair<Monomial, Expression>& p : monomial_to_coefficient_map_) {
    const Monomial& m{p.first};
    degree = std::max(degree, m.total_degree());
  }
  return degree;
}

const Polynomial::MapType& Polynomial::monomial_to_coefficient_map() const {
  return monomial_to_coefficient_map_;
}

Expression Polynomial::ToExpression() const {
  // Returns ∑ᵢ (cᵢ * mᵢ).
  return accumulate(
      monomial_to_coefficient_map_.begin(), monomial_to_coefficient_map_.end(),
      Expression{0.0},
      [](const Expression& init, const pair<Monomial, Expression>& p) {
        const Monomial& m{p.first};
        const Expression& coeff{p.second};
        return init + (coeff * m.ToExpression());
      });
}

Polynomial& Polynomial::operator+=(const Polynomial& p) {
  for (const pair<Monomial, Expression>& item :
       p.monomial_to_coefficient_map_) {
    const Monomial& m{item.first};
    const Expression& coeff{item.second};
    DoAdd(coeff, m, &monomial_to_coefficient_map_);
  }
  CheckInvariant();
  return *this;
}

Polynomial& Polynomial::operator+=(const Monomial& m) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return Add(1.0, m);
}

Polynomial& Polynomial::operator+=(const double c) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return Add(c, Monomial{});
}

Polynomial& Polynomial::operator-=(const Polynomial& p) {
  // No need to call CheckInvariant() since it's called inside of operator+=.
  return *this += -p;
}

Polynomial& Polynomial::operator-=(const Monomial& m) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return Add(-1.0, m);
}

Polynomial& Polynomial::operator-=(const double c) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return Add(-c, Monomial{});
}

Polynomial& Polynomial::operator*=(const Polynomial& p) {
  // (c₁₁ * m₁₁ + ... + c₁ₙ * m₁ₙ) * (c₂₁ * m₂₁ + ... + c₂ₘ * m₂ₘ)
  // = (c₁₁ * m₁₁ + ... + c₁ₙ * m₁ₙ) * c₂₁ * m₂₁ + ... +
  //   (c₁₁ * m₁₁ + ... + c₁ₙ * m₁ₙ) * c₂ₘ * m₂ₘ
  MapType new_map{};
  for (const auto& p1 : monomial_to_coefficient_map_) {
    for (const auto& p2 : p.monomial_to_coefficient_map()) {
      const Monomial new_monomial{p1.first * p2.first};
      const Expression new_coeff{p1.second * p2.second};
      DoAdd(new_coeff, new_monomial, &new_map);
    }
  }
  monomial_to_coefficient_map_ = std::move(new_map);
  CheckInvariant();
  return *this;
}

Polynomial& Polynomial::operator*=(const Monomial& m) {
  MapType new_map;
  for (const pair<Monomial, Expression>& p : monomial_to_coefficient_map_) {
    const Monomial& m_i{p.first};
    const Expression& coeff_i{p.second};
    new_map.emplace(m * m_i, coeff_i);
  }
  monomial_to_coefficient_map_ = std::move(new_map);
  CheckInvariant();
  return *this;
}

Polynomial& Polynomial::operator*=(const double c) {
  for (pair<const Monomial, Expression>& p : monomial_to_coefficient_map_) {
    Expression& coeff{p.second};
    coeff *= c;
  }  // No need to call CheckInvariant() since `c` doesn't include a variable.
  return *this;
}

bool Polynomial::EqualTo(const Polynomial& p) const {
  return monomial_to_coefficient_map_ == p.monomial_to_coefficient_map();
}

Formula Polynomial::operator==(Polynomial p) const {
  // 1) Update p' = p - (this polynomial).
  // 2) Extract the condition where p' is zero.
  //    That is, all coefficients should be zero.
  p -= *this;
  Formula ret{Formula::True()};
  for (const pair<Monomial, Expression>& item :
       p.monomial_to_coefficient_map_) {
    const Expression& coeff{item.second};
    ret = ret && (coeff == 0.0);
  }
  return ret;
}

Polynomial& Polynomial::Add(const Expression& coeff, const Monomial& m) {
  DoAdd(coeff, m, &monomial_to_coefficient_map_);
  CheckInvariant();
  return *this;
}

void Polynomial::CheckInvariant() const {
  Variables vars{intersect(decision_variables(), indeterminates())};
  if (!vars.empty()) {
    ostringstream oss;
    oss << "Polynomial " << *this
        << " does not satisfy the invariant because the following variable(s) "
           "are used as decision variables and indeterminates at the same "
           "time:\n"
        << vars << ".";
    throw runtime_error(oss.str());
  }
}

Polynomial operator-(Polynomial p) { return -1 * p; }
Polynomial operator+(Polynomial p1, const Polynomial& p2) { return p1 += p2; }
Polynomial operator+(Polynomial p, const Monomial& m) { return p += m; }
Polynomial operator+(const Monomial& m, Polynomial p) { return p += m; }
Polynomial operator+(const Monomial& m1, const Monomial& m2) {
  return Polynomial(m1) + m2;
}
Polynomial operator+(Polynomial p, const double c) { return p += c; }
Polynomial operator+(const double c, Polynomial p) { return p += c; }
Polynomial operator-(Polynomial p1, const Polynomial& p2) { return p1 -= p2; }
Polynomial operator-(Polynomial p, const Monomial& m) { return p -= m; }
Polynomial operator-(const Monomial& m, Polynomial p) {
  return p = -1 * p + m;  // p' = m - p = -1 * p + m.
}
Polynomial operator-(const Monomial& m1, const Monomial& m2) {
  return Polynomial(m1) - m2;
}
Polynomial operator-(Polynomial p, const double c) { return p -= c; }
Polynomial operator-(const double c, Polynomial p) { return p = -p + c; }
Polynomial operator*(Polynomial p1, const Polynomial& p2) { return p1 *= p2; }
Polynomial operator*(Polynomial p, const Monomial& m) { return p *= m; }
Polynomial operator*(const Monomial& m, Polynomial p) { return p *= m; }
Polynomial operator*(const double c, Polynomial p) { return p *= c; }
Polynomial operator*(Polynomial p, const double c) { return p *= c; }
Polynomial operator*(const Monomial& m, double c) { return Polynomial(m) * c; }
Polynomial operator*(double c, const Monomial& m) { return c * Polynomial(m); }

Polynomial pow(const Polynomial& p, int n) {
  // TODO(soonho-tri): Optimize this by not relying on ToExpression() method.
  return Polynomial{pow(p.ToExpression(), n), p.indeterminates()};
}

ostream& operator<<(ostream& os, const Polynomial& p) {
  const Polynomial::MapType& map{p.monomial_to_coefficient_map()};
  if (map.empty()) {
    return os << 0;
  }
  auto it = map.begin();
  os << it->second << "*" << it->first;
  for (++it; it != map.end(); ++it) {
    os << " + " << it->second << "*" << it->first;
  }
  return os;
}
}  // namespace symbolic
}  // namespace drake
