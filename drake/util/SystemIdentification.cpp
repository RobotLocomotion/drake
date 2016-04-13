#include "drake/util/SystemIdentification.h"

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
  for (PolyType poly : polys) {
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
  for (PolyType poly : polys) {
    for (MonomialType monomial : poly.getMonomials()) {
      typename PolyType::Monomial interest_monomial;
      interest_monomial.coefficient = 1;
      for (TermType term : monomial.terms) {
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
  for (MonomialType interest_monomial : interest_monomials) {
    std::vector<MonomialType> lumped_parameter;
    for (PolyType poly : polys) {
      for (MonomialType monomial : poly.getMonomials()) {
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
    }
    lumped_parameters.insert(PolyType(lumped_parameter.begin(),
                                      lumped_parameter.end()));
  }

  // Third, for the set of such parameter polynomials, create a lumped
  // parameter for each.
  int lump_index = 1;
  SystemIdentification<T>::LumpingMapType lumping_map;

  for (PolyType lump : lumped_parameters) {
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
