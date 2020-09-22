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
using std::make_pair;
using std::map;
using std::ostringstream;
using std::pair;
using std::runtime_error;

namespace drake {
namespace symbolic {
namespace {
using MonomialBasisMapType = GenericPolynomial<MonomialBasisElement>::MapType;

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
  MonomialBasisMapType Decompose(const Expression& e,
                                 const Variables& indeterminates) const {
    // Note that it calls `Expression::Expand()` here.
    return Visit(e.Expand(), indeterminates);
  }

 private:
  MonomialBasisMapType Visit(const Expression& e,
                             const Variables& indeterminates) const {
    return VisitExpression<MonomialBasisMapType>(this, e, indeterminates);
  }

  MonomialBasisMapType VisitVariable(const Expression& e,
                                     const Variables& indeterminates) const {
    const Variable& var{get_variable(e)};
    if (indeterminates.include(var)) {
      // Monomial : var, coefficient : 1
      return MonomialBasisMapType{{{MonomialBasisElement{var}, 1}}};
    } else {
      // Monomial : 1, coefficient : var
      return MonomialBasisMapType{{{MonomialBasisElement{}, var}}};
    }
  }

  MonomialBasisMapType VisitConstant(const Expression& e,
                                     const Variables&) const {
    const double v{get_constant_value(e)};
    if (v != 0) {
      return MonomialBasisMapType{{{MonomialBasisElement(), v}}};  // = v.
    }
    return MonomialBasisMapType{};  // = 0.
  }

  MonomialBasisMapType VisitAddition(const Expression& e,
                                     const Variables& indeterminates) const {
    // e = c₀ + ∑ᵢ (cᵢ * eᵢ)
    MonomialBasisMapType new_map;
    const double c_0{get_constant_in_addition(e)};
    if (c_0 != 0) {
      new_map.emplace(MonomialBasisElement{}, c_0);
    }
    for (const auto& [e_i, c_i] : get_expr_to_coeff_map_in_addition(e)) {
      // e = c₀ + ∑ᵢ (cᵢ * eᵢ) = c₀ + ∑ᵢ (cᵢ * (∑ⱼ mⱼ * cⱼ))
      //                                   ~~~~~~~~~~~
      //                                  Monomial of eᵢ
      //                     = c₀ + ∑ᵢ ∑ⱼ ((cᵢ * cⱼ) * mⱼ)
      // Note that we have cᵢ ≠ 0 ∧ cⱼ ≠ 0 → (cᵢ * cⱼ) ≠ 0.
      const MonomialBasisMapType map_i = Visit(e_i, indeterminates);
      for (const auto& [m_j, c_j] : map_i) {
        // Add (cᵢ * cⱼ) * mⱼ.
        DoAddProduct(c_i * c_j, m_j, &new_map);
      }
    }
    return new_map;
  }

  MonomialBasisMapType VisitMultiplication(
      const Expression& e, const Variables& indeterminates) const {
    // e = c * ∏ᵢ pow(baseᵢ, exponentᵢ).
    const double c = get_constant_in_multiplication(e);
    Expression coeff{c};
    MonomialBasisElement m{};
    for (const auto& [base_i, exponent_i] :
         get_base_to_exponent_map_in_multiplication(e)) {
      const auto [m_i, coeff_i] = VisitPow(base_i, exponent_i, indeterminates);
      m.MergeBasisElementInPlace(m_i);
      coeff *= coeff_i;
    }
    return MonomialBasisMapType{{m, coeff}};
  }

  pair<MonomialBasisElement, Expression> VisitPow(
      const Expression& base, const Expression& exponent,
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
      return make_pair(MonomialBasisElement{}, pow(base, exponent));
    } else {
      // Case: vars(baseᵢ) ∩ indeterminates ≠ ∅. Moreover, we have
      // vars(baseᵢ) ⊆ indeterminates as baseᵢ is already expanded.
      // exponentᵢ should be a positive integer.
      if (!is_constant(exponent) ||
          !is_positive_integer(get_constant_value(exponent))) {
        ostringstream oss;
        oss << "Given the base " << base << ", the Exponent " << exponent
            << " should be a positive integer but it is not the case.";
        throw runtime_error(oss.str());
      }

      const int n{static_cast<int>(get_constant_value(exponent))};
      // `base` should be an indeterminate because `e` is a pre-expanded term.
      if (!is_variable(base)) {
        ostringstream oss;
        oss << "Base " << base << " is not an indeterminate, "
            << indeterminates;
        throw runtime_error(oss.str());
      }
      // Since we call e.Expand() before `Visit` function, `base` is already
      // expanded. If the variables in base intersect with indeterminates, then
      // it has to be a subset of indeterminates.
      DRAKE_ASSERT(base.GetVariables().IsSubsetOf(indeterminates));
      DRAKE_ASSERT(base.GetVariables().size() == 1);
      return make_pair(MonomialBasisElement{get_variable(base), n}, 1.0);
    }
  }

  MonomialBasisMapType VisitPow(const Expression& e,
                                const Variables& indeterminates) const {
    const Expression& base{get_first_argument(e)};
    const Expression& exponent{get_second_argument(e)};
    const pair<MonomialBasisElement, Expression> result{
        VisitPow(base, exponent, indeterminates)};
    return MonomialBasisMapType{{{result.first, result.second}}};
  }

  MonomialBasisMapType VisitDivision(const Expression& e,
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
    MonomialBasisMapType map{Visit(e1, indeterminates)};
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
  MonomialBasisMapType VisitNonPolynomialTerm(
      const Expression& e, const Variables& indeterminates) const {
    // vars(e) ∩ indeterminates = ∅.
    if (!intersect(e.GetVariables(), indeterminates).empty()) {
      ostringstream oss;
      oss << "The non-polynomial term " << e
          << " should be free of the indeterminates " << indeterminates << ".";
      throw runtime_error(oss.str());
    }
    return {{MonomialBasisElement{}, e}};  // = {1 ↦ e}.
  }

  MonomialBasisMapType VisitAbs(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitLog(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitExp(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitSqrt(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitSin(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitCos(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitTan(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitAsin(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitAcos(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitAtan(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitAtan2(const Expression& e,
                                  const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitSinh(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitCosh(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitTanh(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitMin(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitMax(const Expression& e,
                                const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitCeil(const Expression& e,
                                 const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitFloor(const Expression& e,
                                  const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitIfThenElse(const Expression& e,
                                       const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }
  MonomialBasisMapType VisitUninterpretedFunction(
      const Expression& e, const Variables& indeterminates) const {
    return VisitNonPolynomialTerm(e, indeterminates);
  }

  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend MonomialBasisMapType
  drake::symbolic::VisitExpression<MonomialBasisMapType>(
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
GenericPolynomial<BasisElement>::GenericPolynomial(const Expression& e)
    : GenericPolynomial<BasisElement>{e, e.GetVariables()} {
  // No need to call CheckInvariant() because the following should hold.
  DRAKE_ASSERT(decision_variables().empty());
}

template <typename BasisElement>
GenericPolynomial<BasisElement>::GenericPolynomial(const Expression& e,
                                                   Variables indeterminates)
    : indeterminates_{std::move(indeterminates)} {
  const std::map<MonomialBasisElement, Expression> monomial_to_coefficient_map =
      DecomposePolynomialVisitor{}.Decompose(e, indeterminates_);
  if constexpr (std::is_same_v<BasisElement, MonomialBasisElement>) {
    basis_element_to_coefficient_map_ = std::move(monomial_to_coefficient_map);
  } else {
    for (const auto& [monomial, coeff] : monomial_to_coefficient_map) {
      const std::map<BasisElement, double> monomial_to_basis_element =
          monomial.template ToBasis<BasisElement>();
      for (const auto& [basis_element, coeff2] : monomial_to_basis_element) {
        DoAddProduct(coeff2 * coeff, basis_element,
                     &basis_element_to_coefficient_map_);
      }
    }
  }
  decision_variables_ =
      GetDecisionVariables<BasisElement>(basis_element_to_coefficient_map_);
  // No need to call CheckInvariant() because DecomposePolynomialVisitor is
  // supposed to make sure the invariant holds as a post-condition.
}

template <typename BasisElement>
void GenericPolynomial<BasisElement>::SetIndeterminates(
    const Variables& new_indeterminates) {
  if (new_indeterminates.IsSupersetOf(indeterminates_) &&
      intersect(decision_variables_, new_indeterminates).empty()) {
    indeterminates_ = new_indeterminates;
  } else {
    // TODO(soonho-tri): Optimize this part.
    *this = GenericPolynomial<BasisElement>{ToExpression(), new_indeterminates};
  }
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
Expression GenericPolynomial<BasisElement>::ToExpression() const {
  // Returns ∑ᵢ (cᵢ * mᵢ).
  return accumulate(basis_element_to_coefficient_map_.begin(),
                    basis_element_to_coefficient_map_.end(), Expression{0.0},
                    [](const Expression& init,
                       const pair<const BasisElement, Expression>& p) {
                      const BasisElement& m{p.first};
                      const Expression& coeff{p.second};
                      return init + (coeff * m.ToExpression());
                    })
      .Expand();
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
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator+=(
    const GenericPolynomial<BasisElement>& p) {
  for (const auto& [basis_element, coeff] :
       p.basis_element_to_coefficient_map()) {
    DoAddProduct(coeff, basis_element, &basis_element_to_coefficient_map_);
  }
  indeterminates_ += p.indeterminates();
  decision_variables_ += p.decision_variables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator+=(
    const BasisElement& m) {
  // No need to call CheckInvariant since it's called inside of AddProduct.
  return AddProduct(1.0, m);
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator+=(
    const double c) {
  // No need to call CheckInvariant since it's called inside of AddProduct.
  return AddProduct(c, BasisElement{});
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator+=(
    const Variable& v) {
  if (indeterminates().include(v)) {
    this->AddProduct(1.0, BasisElement{v});
  } else {
    this->AddProduct(v, BasisElement{});
  }
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator-=(
    const GenericPolynomial<BasisElement>& p) {
  return *this += -p;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator-=(
    const BasisElement& m) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return AddProduct(-1.0, m);
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator-=(
    double c) {
  // No need to call CheckInvariant() since it's called inside of Add.
  return AddProduct(-c, BasisElement{});
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator-=(
    const Variable& v) {
  if (indeterminates().include(v)) {
    return AddProduct(-1.0, BasisElement{v});
  } else {
    return AddProduct(-v, BasisElement{});
  }
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator*=(
    const GenericPolynomial<BasisElement>& p) {
  // (c₁₁ * m₁₁ + ... + c₁ₙ * m₁ₙ) * (c₂₁ * m₂₁ + ... + c₂ₘ * m₂ₘ)
  // = (c₁₁ * m₁₁ + ... + c₁ₙ * m₁ₙ) * c₂₁ * m₂₁ + ... +
  //   (c₁₁ * m₁₁ + ... + c₁ₙ * m₁ₙ) * c₂ₘ * m₂ₘ
  // Notice that m₁ᵢ * m₂ⱼ = ∑ₖ dᵢⱼₖ mᵢⱼₖ, namely the product of two basis
  // elements m₁ᵢ * m₂ⱼ is the weighted sum of a series of new basis elements.
  // For example, when we use MonomialBasisElement, m₁ᵢ * m₂ⱼ = 1 * mᵢⱼ; when we
  // use ChebyshevBasisElement, Tᵢ(x) * Tⱼ(x) = 0.5 * Tᵢ₊ⱼ(x) + 0.5 * Tᵢ₋ⱼ(x)
  MapType new_map{};
  for (const auto& [basis_element1, coeff1] :
       basis_element_to_coefficient_map()) {
    for (const auto& [basis_element2, coeff2] :
         p.basis_element_to_coefficient_map()) {
      // basis_element_products stores the product of two basis elements as a
      // weighted sum of new basis elements.
      const std::map<BasisElement, double> basis_element_products{
          basis_element1 * basis_element2};
      const Expression coeff_product{coeff1 * coeff2};
      for (const auto& [new_basis, new_basis_coeff] : basis_element_products) {
        DoAddProduct(new_basis_coeff * coeff_product, new_basis, &new_map);
      }
    }
  }
  basis_element_to_coefficient_map_ = std::move(new_map);
  indeterminates_ += p.indeterminates();
  decision_variables_ += p.decision_variables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator*=(
    const BasisElement& m) {
  MapType new_map;
  for (const auto& [basis_element, coeff] : basis_element_to_coefficient_map_) {
    const std::map<BasisElement, double> basis_element_product{basis_element *
                                                               m};
    for (const auto& [new_basis_element, coeff_product] :
         basis_element_product) {
      DoAddProduct(coeff_product * coeff, new_basis_element, &new_map);
    }
  }
  basis_element_to_coefficient_map_ = std::move(new_map);
  indeterminates_ += m.GetVariables();
  DRAKE_ASSERT_VOID(CheckInvariant());
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator*=(
    double c) {
  for (auto& p : basis_element_to_coefficient_map_) {
    Expression& coeff = p.second;
    coeff *= c;
  }
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator*=(
    const Variable& v) {
  if (indeterminates().include(v)) {
    return *this *= BasisElement{v};
  } else {
    for (auto& p : basis_element_to_coefficient_map_) {
      Expression& coeff = p.second;
      coeff *= v;
    }
    return *this;
  }
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::operator/=(
    double c) {
  for (auto& item : basis_element_to_coefficient_map_) {
    item.second /= c;
  }
  return *this;
}

template <typename BasisElement>
GenericPolynomial<BasisElement>& GenericPolynomial<BasisElement>::AddProduct(
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
  DRAKE_DEMAND(coefficient_tol >= 0);
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

template <typename BasisElement>
bool GenericPolynomial<BasisElement>::EqualToAfterExpansion(
    const GenericPolynomial<BasisElement>& p) const {
  return GenericPolynomialEqual<BasisElement>(*this, p, true);
}

template <typename BasisElement>
bool GenericPolynomial<BasisElement>::CoefficientsAlmostEqual(
    const GenericPolynomial<BasisElement>& p, double tol) const {
  auto it1 = this->basis_element_to_coefficient_map_.begin();
  auto it2 = p.basis_element_to_coefficient_map_.begin();
  while (it1 != this->basis_element_to_coefficient_map_.end() &&
         it2 != this->basis_element_to_coefficient_map().end()) {
    if (it1->first == it2->first) {
      const symbolic::Expression coeff_diff = it1->second - it2->second;
      if (is_constant(coeff_diff) &&
          std::abs(get_constant_value(coeff_diff)) <= tol) {
        it1++;
        it2++;
        continue;
      } else {
        return false;
      }
    } else if (it1->first < it2->first) {
      if (is_constant(it1->second) &&
          std::abs(get_constant_value(it1->second)) < tol) {
        it1++;
        continue;
      } else {
        return false;
      }
    } else {
      if (is_constant(it2->second) &&
          std::abs(get_constant_value(it2->second)) < tol) {
        it2++;
        continue;
      } else {
        return false;
      }
    }
  }

  while (it1 != this->basis_element_to_coefficient_map().end()) {
    if (is_constant(it1->second) &&
        std::abs(get_constant_value(it1->second)) < tol) {
      it1++;
      continue;
    } else {
      return false;
    }
  }

  while (it2 != p.basis_element_to_coefficient_map().end()) {
    if (is_constant(it2->second) &&
        std::abs(get_constant_value(it2->second)) < tol) {
      it2++;
      continue;
    } else {
      return false;
    }
  }

  return true;
}

template <typename BasisElement>
Formula GenericPolynomial<BasisElement>::operator==(
    const GenericPolynomial<BasisElement>& p) const {
  // 1) Let diff = p - (this polynomial).
  // 2) Extract the condition where diff is zero.
  //    That is, all coefficients should be zero.
  const GenericPolynomial<BasisElement> diff{p - *this};
  Formula ret{Formula::True()};
  for (const pair<const BasisElement, Expression>& item :
       diff.basis_element_to_coefficient_map_) {
    const Expression& coeff{item.second};
    // ret is the conjunction of symbolic formulas. Don't confuse `&&` here
    // with the "logical and" operation between booleans.
    ret = ret && (coeff == 0.0);
  }
  return ret;
}

template <typename BasisElement>
Formula GenericPolynomial<BasisElement>::operator!=(
    const GenericPolynomial<BasisElement>& p) const {
  return !(*this == p);
}

template class GenericPolynomial<MonomialBasisElement>;
template class GenericPolynomial<ChebyshevBasisElement>;
}  // namespace symbolic
}  // namespace drake
