#include "drake/util/SystemIdentification.h"

#include <algorithm>

namespace drake {
namespace util {

template<typename T>
std::set<typename SystemIdentification<T>::MonomialType>
SystemIdentification<T>::GetAllCombinationsOfVars(
    const std::vector<PolyType>& polys,
    const std::set<VarType>& vars_of_interest) {
  std::set<MonomialType> interest_monomials;
  for (const PolyType& poly : polys) {
    for (const MonomialType& monomial : poly.getMonomials()) {
      MonomialType interest_monomial;
      interest_monomial.coefficient = 1;
      // For each term in the monomial, iff that term's variable is in
      // vars_of_interest multiply it in.
      for (const TermType& term : monomial.terms) {
        if (vars_of_interest.count(term.var)) {
          interest_monomial.terms.push_back(term);
        }
      }
      interest_monomials.insert(interest_monomial);
    }
  }
  return interest_monomials;
}

template<typename T>
bool SystemIdentification<T>::MonomialMatches(
    const MonomialType& haystack,
    const MonomialType& needle,
    const std::set<VarType>& vars_of_interest) {
  // By factoring the needle out of the haystack, we either get a failure (in
  // which case return false) or a residue monomial (the parts of haystack not
  // in needle).  If the resuidue contains a var of interest, return false.
  // Otherwise we meet the criteria and return true.
  const MonomialType residue = haystack.factor(needle);
  if (residue.coefficient == 0) {
    return false;
  }
  for (const VarType& var : vars_of_interest) {
    if (residue.getDegreeOf(var) > 0) {
      return false;
    }
  }
  return true;
}

template<typename T>
std::pair<T, typename SystemIdentification<T>::PolyType>
SystemIdentification<T>::CanonicalizePolynomial(const PolyType& poly) {
  std::vector<MonomialType> monomials = poly.getMonomials();
  const T min_coefficient = std::min_element(
      monomials.begin(), monomials.end(),
      [&](const MonomialType& l, const MonomialType& r){
        return l.coefficient < r.coefficient; })->coefficient;
  for (MonomialType& monomial : monomials) {
    monomial.coefficient /= min_coefficient;
  }
  return std::make_pair(min_coefficient, PolyType(monomials.begin(),
                                                  monomials.end()));
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomial(
    const PolyType& poly,
    const std::set<VarType>& vars_of_interest) {
  // Just dispatch to the set version.
  const std::vector<Polynomial<T>> polys = {poly};
  return SystemIdentification<T>::GetLumpedParametersFromPolynomials(
      polys, vars_of_interest);
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomials(
    const std::vector<PolyType>& polys,
    const std::set<VarType>& vars_of_interest) {
  // Before we begin, find all the VarTypes in use so that we can create
  // unique ones for our lumped parameters.
  std::set<VarType> all_vars;
  for (const PolyType& poly : polys) {
    const auto& poly_vars = poly.getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }

  // Extract every combination of the vars_of_interest.
  const std::set<MonomialType> interest_monomials =
      GetAllCombinationsOfVars(polys, vars_of_interest);

  // For each of those combinations, find the corresponding polynomials of
  // parameter (ie, non of-interest) variables in each polynomial.
  std::set<PolyType> lumped_parameters;
  for (const MonomialType& interest_monomial : interest_monomials) {
    for (const PolyType& poly : polys) {
      std::vector<MonomialType> lumped_parameter;
      for (const MonomialType& monomial : poly.getMonomials()) {
        // NOTE: This may be a performance hotspot if this method is called in
        // a tight loop, due to the nested for loops here and in
        // MonomialMatches and its callees.  If so it can be sped up via loop
        // reordering and intermediate maps at some cost to readability.
        if (MonomialMatches(monomial, interest_monomial, vars_of_interest)) {
          lumped_parameter.push_back(monomial.factor(interest_monomial));
        }
      }
      if (!lumped_parameter.size()) { continue; }
      // Factor out any coefficients, so that 'a' and '2*a' are not both
      // considered lumped parameters.
      PolyType lumped_parameter_polynomial(lumped_parameter.begin(),
                                           lumped_parameter.end());
      PolyType normalized =
          CanonicalizePolynomial(lumped_parameter_polynomial).second;
      lumped_parameters.insert(normalized);
    }
  }

  // For each such parameter polynomial, create a lumped parameter with a
  // unique VarType id.
  int lump_index = 1;
  static const std::string kLumpedVariablePrefix = "lump";
  LumpingMapType lumping_map;
  for (const PolyType& lump : lumped_parameters) {
    while (true) {
      VarType lump_var = PolyType(kLumpedVariablePrefix,
                                  lump_index).getSimpleVariable();
      lump_index++;
      if (!all_vars.count(lump_var)) {
        lumping_map[lump] = lump_var;
        all_vars.insert(lump_var);
        break;
      }
    }  // Loop termination: If every id is already used, PolyType() will throw.
  }

  return lumping_map;
}


template<typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    const PolyType& poly,
    const LumpingMapType& lumped_parameters) {
  // Reconstruct vars_of_interest, the variables in poly that are not
  // mentioned by the lumped_parameters.
  std::set<VarType> vars_of_interest = poly.getVariables();
  for (const auto& lump_name_pair : lumped_parameters) {
    std::set<VarType> parameters_in_lump = lump_name_pair.first.getVariables();
    for (const VarType& var : parameters_in_lump) {
      vars_of_interest.erase(var);
    }
  }

  // Loop over the combinations of the variables-of-interest, constructing the
  // polynomial of parameters that multiply by each combination; if that
  // polynomial is a lumped variable, substitute in a new monomial of the
  // lumped variable times the combination instead.
  std::set<MonomialType> interest_monomials =
      GetAllCombinationsOfVars({poly}, vars_of_interest);
  std::vector<MonomialType> working_monomials = poly.getMonomials();
  for (const MonomialType& interest_monomial : interest_monomials) {
    // Because we must iterate over working_monomials, we cannot alter it in
    // place.  Instead we build up two lists in parallel: The updated value of
    // working_monomials (new_working_monomials) unchanged by rewriting and
    // the monomials of non-interest variables that might form the polynomial
    // of a lumped parameter.
    //
    // If (and only if) the polynomial of factored monomials matches a lumped
    // parameter, we construct a new working_monomials list from the
    // new_working_monomials list plus a lumped-parameter term.
    std::vector<MonomialType> new_working_monomials;
    std::vector<MonomialType> factor_monomials;
    for (const MonomialType& working_monomial : working_monomials) {
      if (MonomialMatches(working_monomial, interest_monomial,
                          vars_of_interest)) {
        // This monomial matches our interest monomial; we will factor it by
        // the interest monomial and add the resulting monomial of parameters
        // to our factor list.
        factor_monomials.push_back(working_monomial.factor(interest_monomial));
      } else {
        // This monomial does not match our interest monomial; copy it
        // unchanged.
        new_working_monomials.push_back(working_monomial);
      }
    }
    const PolyType factor_polynomial(factor_monomials.begin(),
                                     factor_monomials.end());
    const auto& canonicalization = CanonicalizePolynomial(factor_polynomial);
    const T coefficient = canonicalization.first;
    const PolyType& canonicalized = canonicalization.second;

    if (!lumped_parameters.count(canonicalized)) {
      // Factoring out this combination yielded a parameter polynomial that
      // does not correspond to a lumped variable.  Ignore it, because we
      // cannot rewrite it correctly.
      //
      // This can happen if `poly` was not one of the polynomials used to
      // construct `lumped_parameters`.
      continue;
    }

    // We have a lumped parameter, so construct a new monomial and replace the
    // working monomials list.
    TermType lump_term;
    lump_term.var = lumped_parameters.at(canonicalized);
    lump_term.power = 1;
    MonomialType lumped_monomial;
    lumped_monomial.terms = interest_monomial.terms;
    lumped_monomial.terms.push_back(lump_term);
    lumped_monomial.coefficient = coefficient;
    new_working_monomials.push_back(lumped_monomial);
    working_monomials = new_working_monomials;
  }

  return PolyType(working_monomials.begin(), working_monomials.end());
}

}  // namespace util
}  // namespace drake

template class DRAKEPOLYNOMIAL_EXPORT drake::util::SystemIdentification<double>;
