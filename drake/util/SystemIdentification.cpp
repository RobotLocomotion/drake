#include "drake/util/SystemIdentification.h"

#include <algorithm>

// TODO DEFECT ggould
#include <iostream>

namespace drake {
namespace util {

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomial(
    PolyType poly,
    std::set<VarType> vars_of_interest) {
  // Just dispatch to the set version.
  std::vector<Polynomial<T>> polys = {poly};
  return SystemIdentification<T>::GetLumpedParametersFromPolynomials(
      polys, vars_of_interest);
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomials(
    std::vector<PolyType> polys,
    std::set<VarType> vars_of_interest) {
  // Before we begin, check that we can reserve some names (VarType values)
  // for our lumped parameters.
  std::set<VarType> all_vars;
  for (const PolyType& poly : polys) {
    auto poly_vars = poly.getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }
  VarType reservation_start = Polynomiald("lump", 1).getSimpleVariable();
  VarType reservation_end = Polynomiald("lump", 1000).getSimpleVariable();
  for (VarType var : all_vars) {
    if ((var >= reservation_start) && (var <= reservation_end)) {
      throw std::runtime_error(
          "Lumped parameters failed because variable name already in use");
    }
  }

  // First, extract every combination of the vars_of_interest.
  std::set<typename PolyType::Monomial> interest_monomials;
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

  // Second, for each of those combinations, find the corresponding
  // polynomials of parameter (ie, non of-interest) variables in each
  // polynomial.
  std::set<PolyType> lumped_parameters;
  for (const MonomialType& interest_monomial : interest_monomials) {
    for (const PolyType& poly : polys) {
      std::vector<MonomialType> lumped_parameter;
      for (const MonomialType& monomial : poly.getMonomials()) {
        MonomialType residue = monomial.factor(interest_monomial);
        if (residue.coefficient == 0) { continue; }
        bool reject = false;
        for (VarType var : vars_of_interest) {
          if (residue.getDegreeOf(var) > 0) {
            reject = true;
            break;
          }
        }
        if (reject) { continue; }
        lumped_parameter.push_back(residue);
      }
      if (!lumped_parameter.size()) { continue; }
      // Factor out any coefficients, so that 'a' and '2*a' are not both
      // considered lumped parameters.
      T min_coefficient = std::min_element(
          lumped_parameter.begin(), lumped_parameter.end(),
          [&](MonomialType l, MonomialType r){
            return l.coefficient < r.coefficient; })->coefficient;
      for (MonomialType& monomial : lumped_parameter) {
        monomial.coefficient /= min_coefficient;
      }
      PolyType lumped_parameter_polynomial(lumped_parameter.begin(),
                                           lumped_parameter.end());
      lumped_parameters.insert(lumped_parameter_polynomial);
    }
  }

  // Third, for the set of such parameter polynomials, create a lumped
  // parameter for each.
  int lump_index = 1;
  typename SystemIdentification<T>::LumpingMapType lumping_map;

  for (const PolyType& lump : lumped_parameters) {
    VarType lump_var = PolyType("lump", lump_index).getSimpleVariable();
    lumping_map[lump] = lump_var;
  }

  return lumping_map;
}

template<typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    PolyType poly,
    LumpingMapType lumped_parameters) {
  return PolyType();
}

};
};

template class DRAKEPOLYNOMIAL_EXPORT drake::util::SystemIdentification<double>;
