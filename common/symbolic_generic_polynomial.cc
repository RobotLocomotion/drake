// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

using std::accumulate;
using std::map;
using std::ostringstream;
using std::pair;
using std::runtime_error;

namespace drake {
namespace symbolic {
namespace {
// Helper function to add coeff * m to a map (BasisElement→ Expression).
// Used to implement DecomposePolynomialVisitor::VisitAddition and
// GenericPolynomial::Add.
template <typename BasisElement>
void DoAddProduct(
    const Expression& coeff, const BasisElement& basis_element,
    typename GenericPolynomial<BasisElement>::MapType* const map) {
  if (is_zero(coeff)) {
    return;
  }
  auto it = map->find(basis_element);
  if (it != map->end()) {
    // basis_element ∈ dom(map)
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
    // basis_element ∉ dom(map)
    map->emplace_hint(it, basis_element, coeff);
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
template <typename BasisElement>
Variables GetIndeterminates(
    const typename GenericPolynomial<BasisElement>::MapType& m) {
  Variables vars;
  for (const pair<const BasisElement, Expression>& p : m) {
    const BasisElement& m_i{p.first};
    vars += m_i.GetVariables();
  }
  return vars;
}

template <typename BasisElement>
Variables GetDecisionVariables(
    const typename GenericPolynomial<BasisElement>::MapType& m) {
  Variables vars;
  for (const pair<const BasisElement, Expression>& p : m) {
    const Expression& e_i{p.second};
    vars += e_i.GetVariables();
  }
  return vars;
}
}  // namespace

template <typename BasisElement>
GenericPolynomial<BasisElement>::GenericPolynomial(MapType init)
    : basis_element_to_coefficient_map_{move(init)},
      indeterminates_{
          GetIndeterminates<BasisElement>(basis_element_to_coefficient_map_)},
      decision_variables_{GetDecisionVariables<BasisElement>(
          basis_element_to_coefficient_map_)} {
  DRAKE_ASSERT_VOID(CheckInvariant());
}

template <typename BasisElement>
void GenericPolynomial<BasisElement>::CheckInvariant() const {
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

template <typename BasisElement>
GenericPolynomial<BasisElement>::GenericPolynomial(const BasisElement& m)
    : basis_element_to_coefficient_map_{{m, 1}},
      indeterminates_{m.GetVariables()},
      decision_variables_{} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

template <typename BasisElement>
int GenericPolynomial<BasisElement>::Degree(const Variable& v) const {
  int degree{0};
  for (const pair<const BasisElement, Expression>& p :
       basis_element_to_coefficient_map_) {
    degree = std::max(degree, p.first.degree(v));
  }
  return degree;
}

template <typename BasisElement>
int GenericPolynomial<BasisElement>::TotalDegree() const {
  int degree{0};
  for (const pair<const BasisElement, Expression>& p :
       basis_element_to_coefficient_map_) {
    degree = std::max(degree, p.first.total_degree());
  }
  return degree;
}

template <typename BasisElement>
GenericPolynomial<BasisElement> GenericPolynomial<BasisElement>::Differentiate(
    const Variable& x) const {
  if (this->indeterminates_.include(x)) {
    // x is an indeterminate.
    GenericPolynomial<BasisElement>::MapType map;
    for (const auto& [basis_element, coeff] :
         this->basis_element_to_coefficient_map_) {
      // Take the derivative of basis_element m as dm/dx = sum_{key}
      // basis_element_gradient[key] * key.
      const std::map<BasisElement, double> basis_element_gradient =
          basis_element.Differentiate(x);

      for (const auto& p : basis_element_gradient) {
        DoAddProduct(coeff * p.second, p.first, &map);
      }
    }
    return GenericPolynomial<BasisElement>(map);
  } else if (decision_variables_.include(x)) {
    // x is a decision variable.
    GenericPolynomial<BasisElement>::MapType map;
    for (const auto& [basis_element, coeff] :
         basis_element_to_coefficient_map_) {
      DoAddProduct(coeff.Differentiate(x), basis_element, &map);
    }
    return GenericPolynomial<BasisElement>(map);
  } else {
    // The variable `x` does not appear in this polynomial. Returns zero
    // polynomial.
    return GenericPolynomial<BasisElement>();
  }
}

template <typename BasisElement>
double GenericPolynomial<BasisElement>::Evaluate(const Environment& env) const {
  return accumulate(
      basis_element_to_coefficient_map_.begin(),
      basis_element_to_coefficient_map_.end(), 0.0,
      [&env](const double v, const pair<BasisElement, Expression>& item) {
        const BasisElement& basis_element{item.first};
        const Expression& coeff{item.second};
        return v + basis_element.Evaluate(env) * coeff.Evaluate(env);
      });
}

template <typename BasisElement>
GenericPolynomial<BasisElement>
GenericPolynomial<BasisElement>::EvaluatePartial(const Environment& env) const {
  MapType new_map;  // Will use this to construct the return value.
  for (const auto& [basis_element, coeff] : basis_element_to_coefficient_map_) {
    const Expression coeff_partial_evaluated{coeff.EvaluatePartial(env)};
    const pair<double, BasisElement> partial_eval_result{
        basis_element.EvaluatePartial(env)};
    const Expression new_coeff{coeff_partial_evaluated *
                               partial_eval_result.first};
    const BasisElement& new_basis_element{partial_eval_result.second};

    auto it = new_map.find(new_basis_element);
    if (it == new_map.end()) {
      new_map.emplace_hint(it, new_basis_element, new_coeff);
    } else {
      it->second += new_coeff;
    }
  }
  return GenericPolynomial<BasisElement>(new_map);
}

template <typename BasisElement>
GenericPolynomial<BasisElement> GenericPolynomial<BasisElement>::AddProduct(
    const Expression& coeff, const BasisElement& m) {
  DoAddProduct(coeff, m, &basis_element_to_coefficient_map_);
  indeterminates_ += m.GetVariables();
  decision_variables_ += coeff.GetVariables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>
GenericPolynomial<BasisElement>::RemoveTermsWithSmallCoefficients(
    double coefficient_tol) const {
  DRAKE_DEMAND(coefficient_tol > 0);
  MapType cleaned_polynomial{};
  for (const auto& [basis_element, coeff] : basis_element_to_coefficient_map_) {
    if (is_constant(coeff) &&
        std::abs(get_constant_value(coeff)) <= coefficient_tol) {
      // The coefficients are small.
      continue;
    } else {
      cleaned_polynomial.emplace_hint(cleaned_polynomial.end(), basis_element,
                                      coeff);
    }
  }
  return GenericPolynomial<BasisElement>(cleaned_polynomial);
}

template <typename BasisElement>
GenericPolynomial<BasisElement>
GenericPolynomial<BasisElement>::EvaluatePartial(const Variable& var,
                                                 const double c) const {
  return EvaluatePartial({{{var, c}}});
}

namespace {
template <typename BasisElement>
bool GenericPolynomialEqual(const GenericPolynomial<BasisElement>& p1,
                            const GenericPolynomial<BasisElement>& p2,
                            bool do_expansion) {
  const typename GenericPolynomial<BasisElement>::MapType& map1{
      p1.basis_element_to_coefficient_map()};
  const typename GenericPolynomial<BasisElement>::MapType& map2{
      p2.basis_element_to_coefficient_map()};
  if (map1.size() != map2.size()) {
    return false;
  }
  // Since both map1 and map2 are ordered map, we can compare them one by one in
  // an ordered manner.
  auto it1 = map1.begin();
  auto it2 = map2.begin();
  while (it1 != map1.end()) {
    if (it1->first != (it2->first)) {
      return false;
    }
    const Expression& e1{it1->second};
    const Expression& e2{it2->second};
    if (do_expansion) {
      if (!e1.Expand().EqualTo(e2.Expand())) {
        return false;
      }
    } else {
      if (!e1.EqualTo(e2)) {
        return false;
      }
    }
    it1++;
    it2++;
  }
  return true;
}
}  // namespace

template <typename BasisElement>
bool GenericPolynomial<BasisElement>::EqualTo(
    const GenericPolynomial<BasisElement>& p) const {
  return GenericPolynomialEqual<BasisElement>(*this, p, false);
}

template class GenericPolynomial<MonomialBasisElement>;
template class GenericPolynomial<ChebyshevBasisElement>;
}  // namespace symbolic
}  // namespace drake
