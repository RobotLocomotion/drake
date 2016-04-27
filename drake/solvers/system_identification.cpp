#include "drake/solvers/system_identification.h"

#include <algorithm>

namespace drake {
namespace solvers {

template<typename T>
std::set<typename SystemIdentification<T>::MonomialType>
SystemIdentification<T>::GetAllCombinationsOfVars(
    const std::vector<PolyType>& polys,
    const std::set<VarType>& vars) {
  std::set<MonomialType> result_monomials;
  for (const PolyType& poly : polys) {
    for (const MonomialType& monomial : poly.getMonomials()) {
      MonomialType monomial_of_vars;
      monomial_of_vars.coefficient = 1;
      // For each term in the monomial, iff that term's variable is in
      // vars then multiply it in.
      for (const TermType& term : monomial.terms) {
        if (vars.count(term.var)) {
          monomial_of_vars.terms.push_back(term);
        }
      }
      result_monomials.insert(monomial_of_vars);
    }
  }
  return result_monomials;
}

template<typename T>
bool SystemIdentification<T>::MonomialMatches(
    const MonomialType& haystack,
    const MonomialType& needle,
    const std::set<VarType>& active_vars) {
  // By factoring the needle out of the haystack, we either get a failure (in
  // which case return false) or a residue monomial (the parts of haystack not
  // in needle).  If the resuidue contains an active variable, return false.
  // Otherwise we meet the criteria and return true.
  const MonomialType residue = haystack.factor(needle);
  if (residue.coefficient == 0) {
    return false;
  }
  for (const VarType& var : active_vars) {
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
    const std::set<VarType>& parameters) {
  // Just dispatch to the set version.
  const std::vector<Polynomial<T>> polys = {poly};
  return SystemIdentification<T>::GetLumpedParametersFromPolynomials(
      polys, parameters);
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomials(
    const std::vector<PolyType>& polys,
    const std::set<VarType>& parameters) {
  // Before we begin, find all the VarTypes in use so that we can create
  // unique ones for our lumped parameters, and find the list of active
  // variables..
  std::set<VarType> all_vars;
  for (const PolyType& poly : polys) {
    const auto& poly_vars = poly.getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }
  std::set<VarType> active_vars = all_vars;
  for (const VarType& parameter : parameters) {
    active_vars.erase(parameter);
  }

  // Extract every combination of the active_vars.
  const std::set<MonomialType> active_var_monomials =
      GetAllCombinationsOfVars(polys, active_vars);

  // For each of those combinations, find the corresponding polynomials of
  // parameter variables in each polynomial.
  std::set<PolyType> lumped_parameters;
  for (const MonomialType& active_var_monomial : active_var_monomials) {
    for (const PolyType& poly : polys) {
      std::vector<MonomialType> lumped_parameter;
      for (const MonomialType& monomial : poly.getMonomials()) {
        // NOTE: This may be a performance hotspot if this method is called in
        // a tight loop, due to the nested for loops here and in
        // MonomialMatches and its callees.  If so it can be sped up via loop
        // reordering and intermediate maps at some cost to readability.
        if (MonomialMatches(monomial, active_var_monomial, active_vars)) {
          lumped_parameter.push_back(monomial.factor(active_var_monomial));
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
  LumpingMapType lumping_map;
  for (const PolyType& lump : lumped_parameters) {
    VarType lump_var = CreateLumpVar(all_vars);
    lumping_map[lump] = lump_var;
    all_vars.insert(lump_var);
  }

  return lumping_map;
}

template<typename T>
typename SystemIdentification<T>::VarType
SystemIdentification<T>::CreateLumpVar(
    const std::set<VarType>& vars_in_use) {
  int lump_index = 1;
  static const std::string kLumpedVariablePrefix = "lump";
  while (true) {
    VarType lump_var = PolyType(kLumpedVariablePrefix,
                                lump_index).getSimpleVariable();
    lump_index++;
    if (!vars_in_use.count(lump_var)) {
      return lump_var;
    }
  }  // Loop termination: If every id is already used, PolyType() will throw.
}


template<typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    const PolyType& poly,
    const LumpingMapType& lumped_parameters) {
  // Reconstruct active_vars, the variables in poly that are not
  // mentioned by the lumped_parameters.
  std::set<VarType> active_vars = poly.getVariables();
  for (const auto& lump_name_pair : lumped_parameters) {
    std::set<VarType> parameters_in_lump = lump_name_pair.first.getVariables();
    for (const VarType& var : parameters_in_lump) {
      active_vars.erase(var);
    }
  }

  // Loop over the combinations of the active variables, constructing the
  // polynomial of parameters that multiply by each combination; if that
  // polynomial is a lumped variable, substitute in a new monomial of the
  // lumped variable times the combination instead.
  std::set<MonomialType> active_var_monomials =
      GetAllCombinationsOfVars({poly}, active_vars);
  std::vector<MonomialType> working_monomials = poly.getMonomials();
  for (const MonomialType& active_var_monomial : active_var_monomials) {
    // Because we must iterate over working_monomials, we cannot alter it in
    // place.  Instead we build up two lists in parallel: The updated value of
    // working_monomials (new_working_monomials) unchanged by rewriting and
    // the monomials of parameter variables that might form the polynomial
    // of a lumped parameter.
    //
    // If (and only if) the polynomial of factored monomials matches a lumped
    // parameter, we construct a new working_monomials list from the
    // new_working_monomials list plus a lumped-parameter term.
    std::vector<MonomialType> new_working_monomials;
    std::vector<MonomialType> factor_monomials;
    for (const MonomialType& working_monomial : working_monomials) {
      if (MonomialMatches(working_monomial, active_var_monomial,
                          active_vars)) {
        // This monomial matches our active vars monomial; we will factor it
        // by the active vars monomial and add the resulting monomial of
        // parameters to our factor list.
        factor_monomials.push_back(
            working_monomial.factor(active_var_monomial));
      } else {
        // This monomial does not match our active vars monomial; copy it
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
    lumped_monomial.terms = active_var_monomial.terms;
    lumped_monomial.terms.push_back(lump_term);
    lumped_monomial.coefficient = coefficient;
    new_working_monomials.push_back(lumped_monomial);
    working_monomials = new_working_monomials;
  }

  return PolyType(working_monomials.begin(), working_monomials.end());
}

}  // namespace solvers
}  // namespace drake

template class DRAKEOPTIMIZTION_EXPORT
drake::solvers::SystemIdentification<double>;
