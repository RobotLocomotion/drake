// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <algorithm>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

using std::accumulate;
using std::make_pair;
using std::map;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::to_string;

namespace drake {
namespace symbolic {

namespace {
// Helper function to add coeff * m to a map (Monomial → Expression).
// Used to implement DecomposePolynomialVisitor::VisitAddition and
// Polynomial::Add.
void DoAddProduct(const Expression& coeff, const Monomial& m,
                  Polynomial::MapType* const map) {
  if (is_zero(coeff)) {
    return;
  }
  auto it = map->find(m);
  if (it != map->end()) {
    // m ∈ dom(map)
    Expression& existing_coeff = it->second;
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
                                const Variables& indeterminates) const {
    // Note that it calls `Expression::Expand()` here.
    return Visit(e.Expand(), indeterminates);
  }

 private:
  Polynomial::MapType Visit(const Expression& e,
                            const Variables& indeterminates) const {
    return VisitExpression<Polynomial::MapType>(this, e, indeterminates);
  }

  Polynomial::MapType VisitVariable(const Expression& e,
                                    const Variables& indeterminates) const {
    const Variable& var{get_variable(e)};
    if (indeterminates.include(var)) {
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
                                    const Variables& indeterminates) const {
    // e = c₀ + ∑ᵢ (cᵢ * eᵢ)
    Polynomial::MapType new_map;
    const double c_0{get_constant_in_addition(e)};
    if (c_0 != 0) {
      new_map.emplace(Monomial{}, c_0);
    }
    for (const pair<const Expression, double>& p :
         get_expr_to_coeff_map_in_addition(e)) {
      const Expression& e_i{p.first};
      const double c_i{p.second};
      // e = c₀ + ∑ᵢ (cᵢ * eᵢ) = c₀ + ∑ᵢ (cᵢ * (∑ⱼ mⱼ * cⱼ))
      //                                   ~~~~~~~~~~~
      //                                  Monomial of eᵢ
      //                     = c₀ + ∑ᵢ ∑ⱼ ((cᵢ * cⱼ) * mⱼ)
      // Note that we have cᵢ ≠ 0 ∧ cⱼ ≠ 0 → (cᵢ * cⱼ) ≠ 0.
      const Polynomial::MapType map_i = Visit(e_i, indeterminates);
      for (const pair<const Monomial, Expression>& term : map_i) {
        const Monomial& m_j{term.first};
        const Expression& c_j{term.second};
        // Add (cᵢ * cⱼ) * mⱼ.
        DoAddProduct(c_i * c_j, m_j, &new_map);
      }
    }
    return new_map;
  }

  Polynomial::MapType VisitMultiplication(
      const Expression& e, const Variables& indeterminates) const {
    // e = c * ∏ᵢ pow(baseᵢ, exponentᵢ).
    const double c = get_constant_in_multiplication(e);
    Expression coeff{c};
    Monomial m{};
    for (const pair<const Expression, Expression>& p :
         get_base_to_exponent_map_in_multiplication(e)) {
      const Expression& base_i{p.first};
      const Expression& exponent_i{p.second};
      const pair<Monomial, Expression> result_i{
          VisitPow(base_i, exponent_i, indeterminates)};
      const Monomial& m_i{result_i.first};
      const Expression& coeff_i{result_i.second};
      m *= m_i;
      coeff *= coeff_i;
    }
    return Polynomial::MapType{{m, coeff}};
  }

  pair<Monomial, Expression> VisitPow(const Expression& base,
                                      const Expression& exponent,
                                      const Variables& indeterminates) const {
    if (intersect(base.GetVariables(), indeterminates).empty()) {
      // Case: vars(baseᵢ) ∩ indeterminates = ∅.
      if (!intersect(exponent.GetVariables(), indeterminates).empty()) {
        // An indeterminate should not be in an exponent for the whole
        // expression to be a polynomial. For example, aˣ is not a
        // polynomial. That is, vars(exponentᵢ) ∩ indeterminates = ∅ should
        // hold.
        ostringstream oss;
        oss << "Exponent " << exponent << " includes an indeterminates "
            << indeterminates << ".";
        throw runtime_error(oss.str());
      }
      return make_pair(Monomial{}, pow(base, exponent));
    } else {
      // Case: vars(baseᵢ) ∩ indeterminates ≠ ∅.
      // exponentᵢ should be a positive integer.
      if (!is_constant(exponent) ||
          !is_positive_integer(get_constant_value(exponent))) {
        ostringstream oss;
        oss << "Given the base " << base << ", the Exponent " << exponent
            << " should be a positive integer but it is not the case.";
        throw runtime_error(oss.str());
      }

      const int n{static_cast<int>(get_constant_value(exponent))};
      Expression coeff{1.0};
      Monomial m{};
      // `base` should be a product of indeterminates because `e` is a
      // pre-expanded term.
      if (!is_variable(base) && !is_multiplication(base)) {
        ostringstream oss;
        oss << "Base " << base << " is not a product of indeterminates, "
            << indeterminates;
        throw runtime_error(oss.str());
      }
      for (const Variable& var : base.GetVariables()) {
        if (indeterminates.include(var)) {
          m *= Monomial{var, n};
        } else {
          coeff *= pow(Expression{var}, exponent);
        }
      }
      return make_pair(m, coeff);
    }
  }

  Polynomial::MapType VisitPow(const Expression& e,
                               const Variables& indeterminates) const {
    const Expression& base{get_first_argument(e)};
    const Expression& exponent{get_second_argument(e)};
    const pair<Monomial, Expression> result{
        VisitPow(base, exponent, indeterminates)};
    return Polynomial::MapType{{{result.first, result.second}}};
  }

  Polynomial::MapType VisitDivision(const Expression& e,
                                    const Variables& indeterminates) const {
    // e = e₁ / e₂
    const Expression& e1{get_first_argument(e)};
    const Expression& e2{get_second_argument(e)};

    // We require that the denominator e₂ is free of indeterminates for e to be
    // a polynomial. This is because canceling a common factor is not a sound
    // simplification. For example, `(x² + x) / x` is not equivalent to `x + 1`
    // since the former is not defined at x = 0 while the latter is a total
    // function over R.

    // vars(e₂) ∩ indeterminates = ∅.
    if (!intersect(e2.GetVariables(), indeterminates).empty()) {
      ostringstream oss;
      oss << "In " << e1 << " / " << e2 << ", the denominator " << e2
          << " should be free of the indeterminates, " << indeterminates << ".";
      throw runtime_error(oss.str());
    }

    // Since e₁ is already expanded, we have:
    //
    //     e = e₁ / e₂
    //       = (∑ᵢ cᵢ * monomialᵢ) / e₂
    //       = (∑ᵢ (cᵢ/e₂) * monomialᵢ
    //
    // where monomialᵢ is a monomial of indeterminates and cᵢ/e₂ is an
    // expression free of indeterminates (which possibly includes decision
    // variables).
    Polynomial::MapType map{Visit(e1, indeterminates)};
    for (auto& item : map) {
      item.second /= e2;
    }
    return map;
  }

  // For a non-polynomial term, e, we return a map {1 ↦ e}. We require e to be
  // free of indeterminates. For example, `VisitNonPolynomialTerm(sin(a + b),
  // {x})` returns `{1 ↦ sin(a + b)}`. However, `VisitNonPolynomialTerm(sin(a +
  // x), {x})` throws an exception because `sin(a + x)` includes an
  // indeterminate `x`.
  Polynomial::MapType VisitNonPolynomialTerm(
      const Expression& e, const Variables& indeterminates) const {
    // vars(e) ∩ indeterminates = ∅.
    if (!intersect(e.GetVariables(), indeterminates).empty()) {
      ostringstream oss;
      oss << "The non-polynomial term " << e
          << " should be free of the indeterminates " << indeterminates << ".";
      throw runtime_error(oss.str());
    }
    return {{Monomial{}, e}};  // = {1 ↦ e}.
  }

  Polynomial::MapType VisitAbs(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitLog(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitExp(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitSqrt(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitSin(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitCos(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitTan(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitAsin(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitAcos(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitAtan(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitAtan2(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitSinh(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitCosh(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitTanh(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitMin(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitMax(const Expression& e,
                               const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitCeil(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitFloor(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitIfThenElse(const Expression& e,
                                      const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  Polynomial::MapType VisitUninterpretedFunction(
      const Expression& e, const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }

  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend Polynomial::MapType
  drake::symbolic::VisitExpression<Polynomial::MapType>(
      const DecomposePolynomialVisitor*, const Expression&, const Variables&);
};

Variables GetIndeterminates(const Polynomial::MapType& m) {
  Variables vars;
  for (const pair<const Monomial, Expression>& p : m) {
    const Monomial& m_i{p.first};
    vars += m_i.GetVariables();
  }
  return vars;
}

Variables GetDecisionVariables(const Polynomial::MapType& m) {
  Variables vars;
  for (const pair<const Monomial, Expression>& p : m) {
    const Expression& e_i{p.second};
    vars += e_i.GetVariables();
  }
  return vars;
}

}  // namespace

Polynomial::Polynomial(MapType init)
    : monomial_to_coefficient_map_{move(init)},
      indeterminates_{GetIndeterminates(monomial_to_coefficient_map_)},
      decision_variables_{GetDecisionVariables(monomial_to_coefficient_map_)} {
  DRAKE_ASSERT_VOID(CheckInvariant());
};

Polynomial::Polynomial(const Monomial& m)
    : monomial_to_coefficient_map_{{m, 1}},
      indeterminates_{m.GetVariables()},
      decision_variables_{} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

Polynomial::Polynomial(const Expression& e) : Polynomial{e, e.GetVariables()} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

Polynomial::Polynomial(const Expression& e, Variables indeterminates)
    : monomial_to_coefficient_map_{DecomposePolynomialVisitor{}.Decompose(
          e, indeterminates)},
      indeterminates_{std::move(indeterminates)},
      decision_variables_{GetDecisionVariables(monomial_to_coefficient_map_)} {
  // No need to call CheckInvariant() because DecomposePolynomialVisitor is
  // supposed to make sure the invariant holds as a post-condition.
}

const Variables& Polynomial::indeterminates() const { return indeterminates_; }

void Polynomial::SetIndeterminates(const Variables& new_indeterminates) {
  if (new_indeterminates.IsSupersetOf(indeterminates_) &&
      intersect(decision_variables_, new_indeterminates).empty()) {
    indeterminates_ = new_indeterminates;
  } else {
    // TODO(soonho-tri): Optimize this part.
    *this = Polynomial{ToExpression(), new_indeterminates};
  }
}

const Variables& Polynomial::decision_variables() const {
  return decision_variables_;
}

int Polynomial::Degree(const Variable& v) const {
  int degree{0};
  for (const pair<const Monomial, Expression>& p :
       monomial_to_coefficient_map_) {
    const Monomial& m{p.first};
    degree = std::max(degree, m.degree(v));
  }
  return degree;
}

int Polynomial::TotalDegree() const {
  int degree{0};
  for (const pair<const Monomial, Expression>& p :
       monomial_to_coefficient_map_) {
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
      [](const Expression& init, const pair<const Monomial, Expression>& p) {
        const Monomial& m{p.first};
        const Expression& coeff{p.second};
        return init + (coeff * m.ToExpression());
      });
}

namespace {
// Differentiates a monomial `m` with respect to a variable `x`. This is a
// helper function to implement Polynomial::Differentiate() method. It returns a
// pair `(n, m₁ * xⁿ⁻¹ * m₂)` where `d/dx (m₁ * xⁿ * m₂) = n * m₁ * xⁿ⁻¹ * m₂`
// holds. For example, d/dx x²y = 2xy and `DifferentiateMonomial(x²y, x)`
// returns `(2, xy)`.
pair<int, Monomial> DifferentiateMonomial(const Monomial& m,
                                          const Variable& x) {
  if (m.get_powers().count(x) == 0) {
    // x does not appear in m. Returns (0, 1).
    return make_pair(0, Monomial{});
  }
  map<Variable, int> powers{m.get_powers()};
  auto it = powers.find(x);
  DRAKE_ASSERT(it != powers.end() && it->second >= 1);
  const int n{it->second--};
  if (it->second == 0) {
    powers.erase(it);
  }
  return make_pair(n, Monomial{powers});
}
}  // namespace

Polynomial Polynomial::Differentiate(const Variable& x) const {
  if (indeterminates().include(x)) {
    // Case: x is an indeterminate.
    // d/dx ∑ᵢ (cᵢ * mᵢ) = ∑ᵢ d/dx (cᵢ * mᵢ)
    //                   = ∑ᵢ (cᵢ * d/dx mᵢ)
    Polynomial::MapType map;
    for (const pair<const Monomial, Expression>& term :
         monomial_to_coefficient_map_) {
      const Monomial& m{term.first};
      const Expression& coeff{term.second};
      const pair<int, Monomial> m_prime{
          DifferentiateMonomial(m, x)};  // = d/dx m.
      DoAddProduct(coeff * m_prime.first, m_prime.second,
                   &map);  // Add cᵢ * d/dx m.
    }
    return Polynomial{map};
  } else if (decision_variables().include(x)) {
    // Case: x is a decision variable.
    // d/dx ∑ᵢ (cᵢ * mᵢ) = ∑ᵢ d/dx (cᵢ * mᵢ)
    //                   = ∑ᵢ ((d/dx cᵢ) * mᵢ)
    Polynomial::MapType map;
    for (const pair<const Monomial, Expression>& term :
         monomial_to_coefficient_map_) {
      const Monomial& m{term.first};
      const Expression& coeff{term.second};
      DoAddProduct(coeff.Differentiate(x), m, &map);  // Add (d/dx cᵢ) * m.
    }
    return Polynomial{map};
  } else {
    // The variable `x` does not appear in this polynomial.
    return Polynomial{};  // Zero polynomial.
  }
}

Polynomial Polynomial::Integrate(const Variable& x) const {
  if (decision_variables().include(x)) {
    ostringstream oss;
    oss << x << " is a decision variable of polynomial " << *this
        << ".  Integration with respect to decision variables is not "
        << "supported yet.";
    throw runtime_error(oss.str());
  }

  // Case: x is an indeterminate (or does not appear).
  // ∫ ∑ᵢ (cᵢ * mᵢ) dx = ∑ᵢ (cᵢ * ∫ mᵢ dx)
  Polynomial::MapType map;
  for (const pair<const Monomial, Expression>& term :
        monomial_to_coefficient_map_) {
    const Monomial& m{term.first};
    const Expression& coeff{term.second};
    int n = 0;
    auto new_powers = m.get_powers();
    auto it = new_powers.find(x);
    if (it != new_powers.end()) {
      n = it->second++;
    } else {
      new_powers.emplace_hint(it, x, 1);
    }
    DoAddProduct((coeff / (n + 1)).Expand(), Monomial(new_powers), &map);
  }
  return Polynomial{map};
}

Polynomial Polynomial::Integrate(const Variable& x, double a, double b) const {
  // Note: This is still correct if a > b.
  const auto p = this->Integrate(x);
  return p.EvaluatePartial(x, b) - p.EvaluatePartial(x, a);
}

double Polynomial::Evaluate(const Environment& env) const {
  return accumulate(
      monomial_to_coefficient_map_.begin(), monomial_to_coefficient_map_.end(),
      0.0,
      [&env](const double v, const pair<const Monomial, Expression>& item) {
        const Monomial& monomial{item.first};
        const Expression& coeff{item.second};
        return v + monomial.Evaluate(env) * coeff.Evaluate(env);
      });
}

Polynomial Polynomial::EvaluatePartial(const Environment& env) const {
  MapType new_map;  // Will use this to construct the return value.
  for (const auto& product_i : monomial_to_coefficient_map_) {
    const Expression& coeff_i{product_i.second};
    const Expression coeff_i_partial_evaluated{coeff_i.EvaluatePartial(env)};
    const Monomial& monomial_i{product_i.first};
    const pair<double, Monomial> partial_eval_result{
        monomial_i.EvaluatePartial(env)};
    const double coeff_from_subst{partial_eval_result.first};
    const Monomial& monomial_from_subst{partial_eval_result.second};
    const Expression new_coeff_i{coeff_i_partial_evaluated * coeff_from_subst};

    auto it = new_map.find(monomial_from_subst);
    if (it == new_map.end()) {
      new_map.emplace_hint(it, monomial_from_subst, new_coeff_i);
    } else {
      it->second += new_coeff_i;
    }
  }
  return Polynomial{new_map};
}

Polynomial Polynomial::EvaluatePartial(const Variable& var,
                                       const double c) const {
  return EvaluatePartial({{{var, c}}});
}

Polynomial& Polynomial::operator+=(const Polynomial& p) {
  for (const pair<const Monomial, Expression>& item :
       p.monomial_to_coefficient_map_) {
    const Monomial& m{item.first};
    const Expression& coeff{item.second};
    DoAddProduct(coeff, m, &monomial_to_coefficient_map_);
  }
  indeterminates_ += p.indeterminates();
  decision_variables_ += p.decision_variables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

Polynomial& Polynomial::operator+=(const Monomial& m) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return AddProduct(1.0, m);
}

Polynomial& Polynomial::operator+=(const double c) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return AddProduct(c, Monomial{});
}

Polynomial& Polynomial::operator+=(const Variable& v) {
  if (indeterminates().include(v)) {
    return AddProduct(1.0, Monomial{v});
  } else {
    return AddProduct(v, Monomial{});
  }
}

Polynomial& Polynomial::operator-=(const Polynomial& p) {
  // No need to call CheckInvariant() since it's called inside of operator+=.
  return *this += -p;
}

Polynomial& Polynomial::operator-=(const Monomial& m) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return AddProduct(-1.0, m);
}

Polynomial& Polynomial::operator-=(const double c) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return AddProduct(-c, Monomial{});
}

Polynomial& Polynomial::operator-=(const Variable& v) {
  if (indeterminates().include(v)) {
    return AddProduct(-1.0, Monomial{v});
  } else {
    return AddProduct(-v, Monomial{});
  }
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
      DoAddProduct(new_coeff, new_monomial, &new_map);
    }
  }
  monomial_to_coefficient_map_ = std::move(new_map);
  indeterminates_ += p.indeterminates();
  decision_variables_ += p.decision_variables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

Polynomial& Polynomial::operator*=(const Monomial& m) {
  MapType new_map;
  for (const pair<const Monomial, Expression>& p :
       monomial_to_coefficient_map_) {
    const Monomial& m_i{p.first};
    const Expression& coeff_i{p.second};
    new_map.emplace(m * m_i, coeff_i);
  }
  monomial_to_coefficient_map_ = std::move(new_map);
  indeterminates_ += m.GetVariables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

Polynomial& Polynomial::operator*=(const double c) {
  for (pair<const Monomial, Expression>& p : monomial_to_coefficient_map_) {
    Expression& coeff = p.second;
    coeff *= c;
  }  // No need to call CheckInvariant() since `c` doesn't include a variable.
  return *this;
}

Polynomial& Polynomial::operator*=(const Variable& v) {
  if (indeterminates().include(v)) {
    return *this *= Monomial{v};
  } else {
    for (auto& p : monomial_to_coefficient_map_) {
      Expression& coeff = p.second;
      coeff *= v;
    }
    return *this;
  }
}

namespace {
bool PolynomialEqual(const Polynomial& p1, const Polynomial& p2,
                     bool do_expansion) {
  // We do not use unordered_map<Monomial, Expression>::operator== as it uses
  // Expression::operator== (which returns a symbolic formula) instead of
  // Expression::EqualTo(which returns a bool), when the coefficient is a
  // symbolic expression.
  const Polynomial::MapType& map1{p1.monomial_to_coefficient_map()};
  const Polynomial::MapType& map2{p2.monomial_to_coefficient_map()};
  if (map1.size() != map2.size()) {
    return false;
  }
  for (const auto& pair1 : map1) {
    const Monomial& m{pair1.first};
    const Expression& e1{pair1.second};
    const auto it = map2.find(m);
    if (it == map2.end()) {
      // m is not in map2, so map1 and map2 are not the same.
      return false;
    }
    const Expression& e2{it->second};
    if (do_expansion) {
      if (!e1.Expand().EqualTo(e2.Expand())) {
        return false;
      }
    } else {
      if (!e1.EqualTo(e2)) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

bool Polynomial::EqualTo(const Polynomial& p) const {
  return PolynomialEqual(*this, p, false);
}

bool Polynomial::EqualToAfterExpansion(const Polynomial& p) const {
  return PolynomialEqual(*this, p, true);
}

bool Polynomial::CoefficientsAlmostEqual(const Polynomial& p,
                                         double tol) const {
  return PolynomialEqual((*this - p).RemoveTermsWithSmallCoefficients(tol),
                         Polynomial(0), true);
}

Formula Polynomial::operator==(const Polynomial& p) const {
  // 1) Let diff = p - (this polynomial).
  // 2) Extract the condition where diff is zero.
  //    That is, all coefficients should be zero.
  const Polynomial diff{p - *this};
  Formula ret{Formula::True()};
  for (const pair<const Monomial, Expression>& item :
       diff.monomial_to_coefficient_map_) {
    const Expression& coeff{item.second};
    ret = ret && (coeff == 0.0);
  }
  return ret;
}

Formula Polynomial::operator!=(const Polynomial& p) const {
  return !(*this == p);
}

Polynomial& Polynomial::AddProduct(const Expression& coeff, const Monomial& m) {
  DoAddProduct(coeff, m, &monomial_to_coefficient_map_);
  indeterminates_ += m.GetVariables();
  decision_variables_ += coeff.GetVariables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

Polynomial Polynomial::RemoveTermsWithSmallCoefficients(
    double coefficient_tol) const {
  DRAKE_DEMAND(coefficient_tol > 0);
  MapType cleaned_polynomial{};
  for (const auto& term : monomial_to_coefficient_map_) {
    if (is_constant(term.second) &&
        std::abs(get_constant_value(term.second)) <= coefficient_tol) {
      // The coefficients are small.
      continue;
    } else {
      cleaned_polynomial.emplace_hint(cleaned_polynomial.end(), term.first,
                                      term.second);
    }
  }
  return Polynomial(cleaned_polynomial);
}

void Polynomial::CheckInvariant() const {
  // TODO(hongkai.dai and soonho.kong): improves the computation time of
  // CheckInvariant(). See github issue
  // https://github.com/RobotLocomotion/drake/issues/10229
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

Polynomial operator-(const Polynomial& p) { return -1 * p; }
Polynomial operator+(Polynomial p1, const Polynomial& p2) { return p1 += p2; }
Polynomial operator+(Polynomial p, const Monomial& m) { return p += m; }
Polynomial operator+(const Monomial& m, Polynomial p) { return p += m; }
Polynomial operator+(const Monomial& m1, const Monomial& m2) {
  return Polynomial(m1) + m2;
}
Polynomial operator+(Polynomial p, const double c) { return p += c; }
Polynomial operator+(const double c, Polynomial p) { return p += c; }
Polynomial operator+(const Monomial& m, const double c) {
  return Polynomial(m) + c;
}
Polynomial operator+(const double c, const Monomial& m) {
  return c + Polynomial(m);
}
Polynomial operator+(Polynomial p, const Variable& v) { return p += v; }
Polynomial operator+(const Variable& v, Polynomial p) { return p += v; }

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
Polynomial operator-(const Monomial& m, const double c) {
  return Polynomial(m) - c;
}
Polynomial operator-(const double c, const Monomial& m) {
  return c - Polynomial(m);
}
Polynomial operator-(Polynomial p, const Variable& v) { return p -= v; }
Polynomial operator-(const Variable& v, const Polynomial& p) {
  return Polynomial(v, p.indeterminates()) - p;
}

Polynomial operator*(Polynomial p1, const Polynomial& p2) { return p1 *= p2; }
Polynomial operator*(Polynomial p, const Monomial& m) { return p *= m; }
Polynomial operator*(const Monomial& m, Polynomial p) { return p *= m; }
Polynomial operator*(const double c, Polynomial p) { return p *= c; }
Polynomial operator*(Polynomial p, const double c) { return p *= c; }
Polynomial operator*(const Monomial& m, double c) { return Polynomial(m) * c; }
Polynomial operator*(double c, const Monomial& m) { return c * Polynomial(m); }
Polynomial operator*(Polynomial p, const Variable& v) { return p *= v; }
Polynomial operator*(const Variable& v, Polynomial p) { return p *= v; }

Polynomial operator/(Polynomial p, const double v) {
  for (auto& item : p.monomial_to_coefficient_map_) {
    item.second /= v;
  }
  return p;
}

Polynomial pow(const Polynomial& p, int n) {
  // TODO(soonho-tri): Optimize this by not relying on ToExpression() method.
  return Polynomial{pow(p.ToExpression(), n), p.indeterminates()};
}

MatrixX<Polynomial> Jacobian(const Eigen::Ref<const VectorX<Polynomial>>& f,
                             const Eigen::Ref<const VectorX<Variable>>& vars) {
  DRAKE_DEMAND(vars.size() != 0);
  const auto n{f.size()};
  const auto m{vars.size()};
  MatrixX<Polynomial> J(n, m);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      J(i, j) = f[i].Differentiate(vars[j]);
    }
  }
  return J;
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
