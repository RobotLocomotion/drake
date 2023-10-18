#include "drake/common/symbolic/polynomial.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>

#define DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER
#include "drake/common/symbolic/expression/expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER

#include "drake/common/symbolic/decompose.h"
#include "drake/common/text_logging.h"

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
// Note that `.Expand()` is needed in the following kinds of cases:
//     e1 := (a + b)²
//     e2 := - (a² + 2ab + b²)
// Without expanding the terms, they would not report as EqualTo.
bool AreEqualAfterExpanding(const Expression& e1, const Expression& e2) {
  const Expression& e1_expanded = e1.is_expanded() ? e1 : e1.Expand();
  const Expression& e2_expanded = e2.is_expanded() ? e2 : e2.Expand();
  return e1_expanded.EqualTo(e2_expanded);
}

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
    if (AreEqualAfterExpanding(-coeff, existing_coeff)) {
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
    DRAKE_DEMAND(c != 0);
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
    DRAKE_DEMAND(!symbolic::is_zero(coeff));
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
    vars.insert(m_i.GetVariables());
  }
  return vars;
}

Variables GetDecisionVariables(const Polynomial::MapType& m) {
  Variables vars;
  for (const pair<const Monomial, Expression>& p : m) {
    const Expression& e_i{p.second};
    vars.insert(e_i.GetVariables());
  }
  return vars;
}

}  // namespace

Polynomial::Polynomial(MapType map)
    : monomial_to_coefficient_map_{std::move(map)},
      indeterminates_{GetIndeterminates(monomial_to_coefficient_map_)},
      decision_variables_{GetDecisionVariables(monomial_to_coefficient_map_)} {
  // Remove all [monomial, coeff] pair in monomial_to_coefficient_map_ if
  // symbolic::is_zero(coeff) is true;
  for (auto it = monomial_to_coefficient_map_.begin();
       it != monomial_to_coefficient_map_.end();) {
    if (symbolic::is_zero(it->second)) {
      it = monomial_to_coefficient_map_.erase(it);
    } else {
      ++it;
    }
  }
  DRAKE_ASSERT_VOID(CheckInvariant());
};

Polynomial::Polynomial(const Monomial& m)
    : monomial_to_coefficient_map_{{m, 1}},
      indeterminates_{m.GetVariables()},
      decision_variables_{} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

Polynomial::Polynomial(const Variable& v) : Polynomial{v, {v}} {
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
  DRAKE_ASSERT_VOID(CheckInvariant());
}

const Variables& Polynomial::indeterminates() const {
  return indeterminates_;
}

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

Eigen::VectorXd Polynomial::EvaluateIndeterminates(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
    const Eigen::Ref<const Eigen::MatrixXd>& indeterminates_values) const {
  Eigen::VectorXd polynomial_values =
      Eigen::VectorXd::Zero(indeterminates_values.cols());
  for (const auto& [monomial, coeff] : monomial_to_coefficient_map_) {
    const symbolic::Expression& coeff_expanded =
        coeff.is_expanded() ? coeff : coeff.Expand();
    if (!is_constant(coeff_expanded)) {
      throw std::runtime_error(
          fmt::format("Polynomial::EvaluateIndeterminates: the coefficient {} "
                      "is not a constant",
                      coeff_expanded.to_string()));
    }
    const double coeff_val = get_constant_value(coeff_expanded);
    polynomial_values +=
        coeff_val * monomial.Evaluate(indeterminates, indeterminates_values);
  }
  return polynomial_values;
}

void Polynomial::EvaluateWithAffineCoefficients(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
    const Eigen::Ref<const Eigen::MatrixXd>& indeterminates_values,
    Eigen::MatrixXd* A, VectorX<symbolic::Variable>* decision_variables,
    Eigen::VectorXd* b) const {
  // First put all the decision variables into an Eigen vector.
  decision_variables->resize(decision_variables_.size());
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  int variable_count = 0;
  for (const auto& var : decision_variables_) {
    (*decision_variables)(variable_count) = var;
    map_var_to_index.emplace(var.get_id(), variable_count);
    variable_count++;
  }
  const int num_indeterminate_samples = indeterminates_values.cols();
  A->resize(num_indeterminate_samples, variable_count);
  A->setZero();
  b->resize(num_indeterminate_samples);
  b->setZero();
  // Each term in the polynomial is m(x) * c, where m(x) is the
  // monomial and c is the coefficient of the monomial. Since each coefficient
  // c can be written as c = a_coeff * decision_variables + b_coeff, this
  // term can be written as m(x)*a_coeff * decision_variables + m(x)*b_coeff.
  Eigen::RowVectorXd a_coeff(decision_variables->rows());
  double b_coeff;
  for (const auto& [monomial, monomial_coeff] : monomial_to_coefficient_map_) {
    a_coeff.setZero();
    b_coeff = 0;
    const symbolic::Expression monomial_coeff_expand = monomial_coeff.Expand();
    DecomposeAffineExpression(monomial_coeff_expand, map_var_to_index, &a_coeff,
                              &b_coeff);
    const Eigen::VectorXd monomial_vals =
        monomial.Evaluate(indeterminates, indeterminates_values);
    *A += monomial_vals * a_coeff;
    *b += monomial_vals * b_coeff;
  }
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
  if (c == 0) {
    this->monomial_to_coefficient_map_.clear();
    return *this;
  }
  for (pair<const Monomial, Expression>& p : monomial_to_coefficient_map_) {
    Expression& coeff = p.second;
    coeff *= c;
  }
  // No need to call CheckInvariant() since `c` doesn't include a variable
  // and c != 0.
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
    decision_variables_.insert(v);
    return *this;
  }
}

bool Polynomial::EqualTo(const Polynomial& p) const {
  // We do not use unordered_map<Monomial, Expression>::operator== as it uses
  // Expression::operator== (which returns a symbolic formula) instead of
  // Expression::EqualTo(which returns a bool), when the coefficient is a
  // symbolic expression.
  const Polynomial::MapType& map1{monomial_to_coefficient_map_};
  const Polynomial::MapType& map2{p.monomial_to_coefficient_map()};
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
    if (!e1.EqualTo(e2)) {
      return false;
    }
  }
  return true;
}

bool Polynomial::CoefficientsAlmostEqual(const Polynomial& p,
                                         double tolerance) const {
  return (*this - p)
      .Expand()
      .RemoveTermsWithSmallCoefficients(tolerance)
      .EqualTo(Polynomial());
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

Polynomial Polynomial::SubstituteAndExpand(
    const std::unordered_map<Variable, Polynomial>& indeterminate_substitution,
    SubstituteAndExpandCacheData* substitutions_cached_data) const {
  SubstituteAndExpandCacheData substitutions_default_obj;
  SubstituteAndExpandCacheData* cached_data_ptr =
      substitutions_cached_data == nullptr ? &substitutions_default_obj
                                           : substitutions_cached_data;
  std::map<Monomial, Polynomial, internal::CompareMonomial>* substitutions =
      cached_data_ptr->get_data();

  for (const auto& var : indeterminates_) {
    DRAKE_DEMAND(indeterminate_substitution.find(var) !=
                 indeterminate_substitution.cend());
    const Polynomial cur_sub{indeterminate_substitution.at(var).Expand()};
    const Monomial cur_monomial{var};
    if (substitutions->find(cur_monomial) != substitutions->cend()) {
      if (!substitutions->at(cur_monomial).EqualTo(cur_sub)) {
        drake::log()->warn(
            "SubstituteAndExpand(): the passed substitutions_cached_data "
            "contains a different expansion for {} than is contained in "
            "indeterminate_substitutions. Substitutions_cached_data contains "
            "{}, but indeterminate_substitutions contains {}. It is very "
            "likely that substitutions_cached_data is storing expansions which "
            "are inconsistent and so you should not trust the output of this "
            "method.",
            cur_monomial, substitutions->at(cur_monomial), cur_sub);
      }
    } else {
      substitutions->emplace(cur_monomial, cur_sub);
    }
  }

  MapType new_polynomial_coeff_map;
  // Ensures the base case of the constant term monomial is always reached.
  substitutions->insert_or_assign(Monomial(), Polynomial(1));

  // Find the largest (in the lexicographic order) monomial for which we
  // have already computed the expansion.
  auto find_nearest_cached_monomial = [&substitutions](
                                          const Monomial& monomial) {
    auto nearest_cached_monomial_iter =
        std::prev(substitutions->lower_bound(monomial));
    while (!((*nearest_cached_monomial_iter).first)
                .GetVariables()
                .IsSubsetOf(monomial.GetVariables())) {
      nearest_cached_monomial_iter = std::prev(nearest_cached_monomial_iter);
    }
    return (*nearest_cached_monomial_iter).first;
  };

  // Given a monomial, compute its expansion using the saved expansions in
  // substitutions and by repeated squaring. If the total degree of monomial is
  // not 1 (i.e. monomial is not a pure indeterminate), then the expanded
  // substitution will be added to the substitutions map.
  auto compute_substituted_monomial_expansion =
      [&indeterminate_substitution, &substitutions,
       &find_nearest_cached_monomial](
          const Monomial& monomial,
          auto&& compute_substituted_monomial_expansion_recursion_handle)
      -> const Polynomial& {
    // If the monomial total degree is 1, it is an indeterminate and so we can
    // just find the substitution.
    if (monomial.total_degree() == 1) {
      return indeterminate_substitution.at(*monomial.GetVariables().begin());
    }
    // Base case. Since the map substitutions is non-empty and contains the
    // monomial 1 the recursion is guaranteed to terminate.
    if (substitutions->find(monomial) != substitutions->cend()) {
      return substitutions->at(monomial);
    }

    // Find the largest (in the lexicographic order) monomial for which we
    // have already computed the expansion.
    Monomial nearest_cached_monomial = find_nearest_cached_monomial(monomial);

    // If the nearest cached monomial is 1, then we do not have a cached
    // substitution for it. In this case, we will compute the expansion
    // using the successive squaring method. For example, x³y⁵ would be
    // computed as (xy²)²(xy).
    //
    // If the nearest cached monomial is not 1, we recurse on the remaining
    // powers. The reason we do not perform successive squaring immediately
    // is to enable us to potentially find a larger power immediately. For
    // example, if we are expanding x³y⁵, and substitutions only contains
    // the keys {1, xy²}, then the nearest cached monomial is xy².
    // The remaining monomial would be x²y³. Applying the successive
    // squaring method would require us to compute the expansion of xy.
    // Recursing immediately would enable us to again use the substitution
    // of xy². We wish to use the stored substitutions as much as possible,
    // and so we prefer to recurse immediately.
    if (nearest_cached_monomial == Monomial()) {
      Polynomial expanded_substitution{1};
      std::map<Variable, int> halved_powers;
      for (const auto& [var, power] : monomial.get_powers()) {
        halved_powers.emplace(var, static_cast<int>(std::floor(power / 2)));
        // If the current power is odd, we perform a substitution of the
        // degree 1 monomial.
        if (power % 2 == 1) {
          expanded_substitution *= indeterminate_substitution.at(var);
        }
      }
      const Monomial halved_monomials{halved_powers};
      const Monomial& halved_monomials_squared{pow(halved_monomials, 2)};
      const Polynomial& halved_power_substitution{
          compute_substituted_monomial_expansion_recursion_handle(
              halved_monomials,
              compute_substituted_monomial_expansion_recursion_handle)};
      // Store the remaining substitution in case it is useful for later. We do
      // not need to attempt to store the halved_power_substitutions since they
      // should already be stored by the recursive call to
      // compute_substituted_monomial_expansion_recursion_handle.
      if (substitutions->find(halved_monomials_squared) ==
          substitutions->cend()) {
        substitutions->emplace(halved_monomials_squared,
                               pow(halved_power_substitution, 2).Expand());
      }
      expanded_substitution *= substitutions->at(halved_monomials_squared);
      substitutions->emplace(monomial, expanded_substitution.Expand());
    } else {
      std::map<Variable, int> remaining_powers;
      const std::map<Variable, int>& cached_powers{
          nearest_cached_monomial.get_powers()};
      for (const auto& [var, power] : monomial.get_powers()) {
        if (cached_powers.find(var) != cached_powers.cend()) {
          remaining_powers.emplace(var, power - cached_powers.at(var));
        } else {
          remaining_powers.emplace(var, power);
        }
      }
      const Monomial remaining_monomials{remaining_powers};
      const Polynomial& remaining_substitution{
          compute_substituted_monomial_expansion_recursion_handle(
              remaining_monomials,
              compute_substituted_monomial_expansion_recursion_handle)};
      substitutions->emplace(
          monomial,
          (substitutions->at(nearest_cached_monomial) * remaining_substitution)
              .Expand());
    }
    return substitutions->at(monomial);
  };

  for (const auto& [old_monomial, old_coeff] : monomial_to_coefficient_map_) {
    // If substitutions doesn't contain the current substitution create it
    // now.
    if (old_monomial.total_degree() != 1 &&
        substitutions->find(old_monomial) == substitutions->cend()) {
      compute_substituted_monomial_expansion(
          old_monomial, compute_substituted_monomial_expansion);
    }

    // Now go through and add the substitution to the appropriate monomial
    // in the new polynomial.
    const Polynomial& substitution_map =
        (old_monomial.total_degree() == 1
             ? indeterminate_substitution.at(
                   *old_monomial.GetVariables().begin())
             : substitutions->at(old_monomial));
    for (const auto& [new_monomial, new_coeff] :
         substitution_map.monomial_to_coefficient_map()) {
      if (new_polynomial_coeff_map.find(new_monomial) ==
          new_polynomial_coeff_map.cend()) {
        new_polynomial_coeff_map.insert({new_monomial, Expression()});
      }
      new_polynomial_coeff_map.at(new_monomial) +=
          (new_coeff * old_coeff).Expand();
    }
  }
  return Polynomial{new_polynomial_coeff_map};
}

Polynomial Polynomial::Expand() const {
  Polynomial::MapType expanded_poly_map;
  for (const auto& [monomial, coeff] : monomial_to_coefficient_map_) {
    const symbolic::Expression coeff_expanded = coeff.Expand();
    if (!symbolic::is_zero(coeff_expanded)) {
      expanded_poly_map.emplace(monomial, coeff_expanded);
    }
  }
  return symbolic::Polynomial(std::move(expanded_poly_map));
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

namespace {
bool IsEvenOrOdd(const Polynomial& p, bool check_even) {
  for (const auto& [monomial, coeff] : p.monomial_to_coefficient_map()) {
    // If we check p is even/odd, then we only need to check monomials with
    // odd/even-degrees have coefficient = 0
    if (monomial.total_degree() % 2 == static_cast<int>(check_even)) {
      const symbolic::Expression& coeff_expanded =
          coeff.is_expanded() ? coeff : coeff.Expand();
      if (!(is_constant(coeff_expanded) &&
            get_constant_value(coeff_expanded) == 0)) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

bool Polynomial::IsEven() const {
  return IsEvenOrOdd(*this, true /* check_even=true */);
}

bool Polynomial::IsOdd() const {
  return IsEvenOrOdd(*this, false /* check_even=false*/);
}

Eigen::VectorXcd Polynomial::Roots() const {
  if (indeterminates().size() != 1) {
    throw runtime_error(fmt::format(
        "{} is not a univariate polynomial; it has indeterminates {}.", *this,
        indeterminates()));
  }

  // We find the roots by computing the eigenvalues of the companion matrix.
  // See https://en.wikipedia.org/wiki/Polynomial_root-finding_algorithms and
  // https://www.mathworks.com/help/matlab/ref/roots.html.

  const int degree = TotalDegree();

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(degree, degree);
  for (int i = 0; i < degree - 1; ++i) {
    C(i + 1, i) = 1;
  }
  double leading_coefficient = 0;
  for (const auto& [monomial, coeff] : monomial_to_coefficient_map()) {
    if (!is_constant(coeff)) {
      throw runtime_error(fmt::format(
          "Polynomial::Roots() only supports polynomials with constant "
          "coefficients. This polynomial has coefficient {} for the "
          "monomial {}.",
          coeff, monomial));
    }
    const int power = monomial.total_degree();
    if (power == degree) {
      leading_coefficient = get_constant_value(coeff);
    } else {
      C(0, degree - power - 1) = -get_constant_value(coeff);
    }
  }
  C.row(0) /= leading_coefficient;
  return C.eigenvalues();
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
  // Check if any [monomial, coeff] pair has symbolic::is_zero(coeff)
  for (const auto& [monomial, coeff] : monomial_to_coefficient_map_) {
    if (symbolic::is_zero(coeff)) {
      ostringstream oss;
      oss << "Polynomial " << *this
          << " does not satisfy the invariant because the coefficient of the "
             "monomial "
          << monomial << " is 0.\n";
      throw runtime_error(oss.str());
    }
  }
}

Polynomial operator-(const Polynomial& p) {
  return -1 * p;
}
Polynomial operator+(Polynomial p1, const Polynomial& p2) {
  return p1 += p2;
}
Polynomial operator+(Polynomial p, const Monomial& m) {
  return p += m;
}
Polynomial operator+(const Monomial& m, Polynomial p) {
  return p += m;
}
Polynomial operator+(const Monomial& m1, const Monomial& m2) {
  return Polynomial(m1) + m2;
}
Polynomial operator+(Polynomial p, const double c) {
  return p += c;
}
Polynomial operator+(const double c, Polynomial p) {
  return p += c;
}
Polynomial operator+(const Monomial& m, const double c) {
  return Polynomial(m) + c;
}
Polynomial operator+(const double c, const Monomial& m) {
  return c + Polynomial(m);
}
Polynomial operator+(Polynomial p, const Variable& v) {
  return p += v;
}
Polynomial operator+(const Variable& v, Polynomial p) {
  return p += v;
}
Expression operator+(const Expression& e, const Polynomial& p) {
  return e + p.ToExpression();
}
Expression operator+(const Polynomial& p, const Expression& e) {
  return p.ToExpression() + e;
}

Polynomial operator-(Polynomial p1, const Polynomial& p2) {
  return p1 -= p2;
}
Polynomial operator-(Polynomial p, const Monomial& m) {
  return p -= m;
}
Polynomial operator-(const Monomial& m, Polynomial p) {
  return p = -1 * p + m;  // p' = m - p = -1 * p + m.
}
Polynomial operator-(const Monomial& m1, const Monomial& m2) {
  return Polynomial(m1) - m2;
}
Polynomial operator-(Polynomial p, const double c) {
  return p -= c;
}
Polynomial operator-(const double c, Polynomial p) {
  return p = -p + c;
}
Polynomial operator-(const Monomial& m, const double c) {
  return Polynomial(m) - c;
}
Polynomial operator-(const double c, const Monomial& m) {
  return c - Polynomial(m);
}
Polynomial operator-(Polynomial p, const Variable& v) {
  return p -= v;
}
Polynomial operator-(const Variable& v, const Polynomial& p) {
  return Polynomial(v, p.indeterminates()) - p;
}
Expression operator-(const Expression& e, const Polynomial& p) {
  return e - p.ToExpression();
}
Expression operator-(const Polynomial& p, const Expression& e) {
  return p.ToExpression() - e;
}

Polynomial operator*(Polynomial p1, const Polynomial& p2) {
  return p1 *= p2;
}
Polynomial operator*(Polynomial p, const Monomial& m) {
  return p *= m;
}
Polynomial operator*(const Monomial& m, Polynomial p) {
  return p *= m;
}
Polynomial operator*(const double c, Polynomial p) {
  return p *= c;
}
Polynomial operator*(Polynomial p, const double c) {
  return p *= c;
}
Polynomial operator*(const Monomial& m, double c) {
  return Polynomial(m) * c;
}
Polynomial operator*(double c, const Monomial& m) {
  return c * Polynomial(m);
}
Polynomial operator*(Polynomial p, const Variable& v) {
  return p *= v;
}
Polynomial operator*(const Variable& v, Polynomial p) {
  return p *= v;
}
Expression operator*(const Expression& e, const Polynomial& p) {
  return e * p.ToExpression();
}
Expression operator*(const Polynomial& p, const Expression& e) {
  return p.ToExpression() * e;
}

Polynomial operator/(Polynomial p, const double v) {
  for (auto& item : p.monomial_to_coefficient_map_) {
    item.second /= v;
  }
  return p;
}
Expression operator/(const double v, const Polynomial& p) {
  return v / p.ToExpression();
}
Expression operator/(const Expression& e, const Polynomial& p) {
  return e / p.ToExpression();
}
Expression operator/(const Polynomial& p, const Expression& e) {
  return p.ToExpression() / e;
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

// We must define this in the cc file so that symbolic_formula.h is fully
// defined (not just forward declared) when comparing.
namespace Eigen {
namespace numext {
template <>
bool equal_strict(const drake::symbolic::Polynomial& x,
                  const drake::symbolic::Polynomial& y) {
  return static_cast<bool>(x == y);
}
}  // namespace numext
}  // namespace Eigen
