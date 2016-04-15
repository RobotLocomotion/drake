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
      typename PolyType::Monomial interest_monomial;
      interest_monomial.coefficient = 1;
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
SystemIdentification<T>::NormalizePolynomial(const PolyType& poly) {
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
  // Before we begin, check that we can reserve some names (VarType values)
  // for our lumped parameters.
  std::set<VarType> all_vars;
  for (const PolyType& poly : polys) {
    const auto& poly_vars = poly.getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }
  const VarType reservation_start = Polynomiald("lump", 1).getSimpleVariable();
  const VarType reservation_end = Polynomiald("lump", 1000).getSimpleVariable();
  for (const VarType& var : all_vars) {
    if ((var >= reservation_start) && (var <= reservation_end)) {
      throw std::runtime_error(
          "Lumped parameters failed because variable name already in use");
    }
  }

  // First, extract every combination of the vars_of_interest.
  const std::set<typename PolyType::Monomial> interest_monomials =
      GetAllCombinationsOfVars(polys, vars_of_interest);

  // Second, for each of those combinations, find the corresponding
  // polynomials of parameter (ie, non of-interest) variables in each
  // polynomial.
  std::set<PolyType> lumped_parameters;
  for (const MonomialType& interest_monomial : interest_monomials) {
    for (const PolyType& poly : polys) {
      std::vector<MonomialType> lumped_parameter;
      for (const MonomialType& monomial : poly.getMonomials()) {
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
          NormalizePolynomial(lumped_parameter_polynomial).second;
      lumped_parameters.insert(normalized);
    }
  }

  // Third, for the set of such parameter polynomials, create a lumped
  // parameter for each.
  int lump_index = 1;
  typename SystemIdentification<T>::LumpingMapType lumping_map;

  for (const PolyType& lump : lumped_parameters) {
    VarType lump_var = PolyType("lump", lump_index).getSimpleVariable();
    lumping_map[lump] = lump_var;
    lump_index++;
  }

  return lumping_map;
}

template<typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    const PolyType& poly,
    const LumpingMapType& lumped_parameters) {
  std::set<VarType> vars_of_interest = poly.getVariables();
  for (auto lump_name_pair : lumped_parameters) {
    std::set<VarType> parameters_in_lump = lump_name_pair.first.getVariables();
    for (const VarType& var : parameters_in_lump) {
      vars_of_interest.erase(var);
    }
  }
  std::set<typename PolyType::Monomial> interest_monomials =
      GetAllCombinationsOfVars({poly}, vars_of_interest);
  std::vector<MonomialType> working_monomials = poly.getMonomials();
  for (const MonomialType& interest_monomial : interest_monomials) {
    std::vector<MonomialType> new_working_monomials;
    std::vector<int> indices_to_erase;
    std::vector<MonomialType> factor_monomials;
    for (const MonomialType& working_monomial : working_monomials) {
      if (MonomialMatches(working_monomial, interest_monomial,
                          vars_of_interest)) {
        factor_monomials.push_back(working_monomial.factor(interest_monomial));
      } else {
        new_working_monomials.push_back(working_monomial);
      }
    }
    const PolyType factor_polynomial(factor_monomials.begin(),
                               factor_monomials.end());
    const auto& normalization = NormalizePolynomial(factor_polynomial);
    const T factor = normalization.first;
    const PolyType& normalized = normalization.second;
    if (!lumped_parameters.count(normalized)) {
      // No lumping possible for this interest monomial.
      continue;
    }
    TermType lump_term;
    lump_term.var = lumped_parameters.at(normalized);
    lump_term.power = 1;
    MonomialType lumped_monomial;
    lumped_monomial.terms = interest_monomial.terms;
    lumped_monomial.terms.push_back(lump_term);
    lumped_monomial.coefficient = factor;
    new_working_monomials.push_back(lumped_monomial);
    working_monomials = new_working_monomials;
  }

  return PolyType(working_monomials.begin(), working_monomials.end());
}

};
};

template class DRAKEPOLYNOMIAL_EXPORT drake::util::SystemIdentification<double>;
