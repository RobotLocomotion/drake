#include "drake/solvers/system_identification.h"

#include <algorithm>
#include <cmath>
#include <iostream>  // For LUMPED_SYSTEM_IDENTIFICATION_VERBOSE below.
#include <list>

#include "drake/common/drake_assert.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {

template <typename T>
std::set<typename SystemIdentification<T>::MonomialType>
SystemIdentification<T>::GetAllCombinationsOfVars(
    const std::vector<PolyType>& polys, const std::set<VarType>& vars) {
  std::set<MonomialType> result_monomials;
  for (const PolyType& poly : polys) {
    for (const MonomialType& monomial : poly.GetMonomials()) {
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

template <typename T>
bool SystemIdentification<T>::MonomialMatches(
    const MonomialType& haystack, const MonomialType& needle,
    const std::set<VarType>& active_vars) {
  // By factoring the needle out of the haystack, we either get a failure (in
  // which case return false) or a residue monomial (the parts of haystack not
  // in needle).  If the resuidue contains an active variable, return false.
  // Otherwise we meet the criteria and return true.
  const MonomialType residue = haystack.Factor(needle);
  if (residue.coefficient == 0) {
    return false;
  }
  for (const VarType& var : active_vars) {
    if (residue.GetDegreeOf(var) > 0) {
      return false;
    }
  }
  return true;
}

template <typename T>
std::pair<T, typename SystemIdentification<T>::PolyType>
SystemIdentification<T>::CanonicalizePolynomial(const PolyType& poly) {
  std::vector<MonomialType> monomials = poly.GetMonomials();
  const T min_coefficient =
      std::min_element(monomials.begin(), monomials.end(),
                       [&](const MonomialType& l, const MonomialType& r) {
                         return l.coefficient < r.coefficient;
                       })
          ->coefficient;
  for (MonomialType& monomial : monomials) {
    monomial.coefficient /= min_coefficient;
  }
  return std::make_pair(min_coefficient,
                        PolyType(monomials.begin(), monomials.end()));
}

template <typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomial(
    const PolyType& poly, const std::set<VarType>& parameters) {
  // Just dispatch to the set version.
  const std::vector<Polynomial<T>> polys = {poly};
  return SystemIdentification<T>::GetLumpedParametersFromPolynomials(
      polys, parameters);
}

template <typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomials(
    const std::vector<PolyType>& polys, const std::set<VarType>& parameters) {
  // Before we begin, find all the VarTypes in use so that we can create
  // unique ones for our lumped parameters, and find the list of active
  // variables..
  std::set<VarType> all_vars;
  for (const PolyType& poly : polys) {
    const auto& poly_vars = poly.GetVariables();
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
      for (const MonomialType& monomial : poly.GetMonomials()) {
        // NOTE: This may be a performance hotspot if this method is called in
        // a tight loop, due to the nested for loops here and in
        // MonomialMatches and its callees.  If so it can be sped up via loop
        // reordering and intermediate maps at some cost to readability.
        if (MonomialMatches(monomial, active_var_monomial, active_vars)) {
          const MonomialType& candidate = monomial.Factor(active_var_monomial);
          if (candidate.GetDegree() > 0) {  // Don't create lumped constants!
            lumped_parameter.push_back(candidate);
          }
        }
      }
      if (!lumped_parameter.size()) {
        continue;
      }
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
    VarType lump_var = CreateUnusedVar("lump", all_vars);
    lumping_map[lump] = lump_var;
    all_vars.insert(lump_var);
  }

  return lumping_map;
}

template <typename T>
typename SystemIdentification<T>::VarType
SystemIdentification<T>::CreateUnusedVar(const std::string& prefix,
                                         const std::set<VarType>& vars_in_use) {
  int lump_index = 1;
  while (true) {
    VarType lump_var = PolyType(prefix, lump_index).GetSimpleVariable();
    lump_index++;
    if (!vars_in_use.count(lump_var)) {
      return lump_var;
    }
  }  // Loop termination: If every id is already used, PolyType() will throw.
}

template <typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    const PolyType& poly, const LumpingMapType& lumped_parameters) {
  // Reconstruct active_vars, the variables in poly that are not
  // mentioned by the lumped_parameters.
  std::set<VarType> active_vars = poly.GetVariables();
  for (const auto& lump_name_pair : lumped_parameters) {
    std::set<VarType> parameters_in_lump = lump_name_pair.first.GetVariables();
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
  std::vector<MonomialType> working_monomials = poly.GetMonomials();
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
      if (MonomialMatches(working_monomial, active_var_monomial, active_vars)) {
        // This monomial matches our active vars monomial; we will factor it
        // by the active vars monomial and add the resulting monomial of
        // parameters to our factor list.
        factor_monomials.push_back(
            working_monomial.Factor(active_var_monomial));
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

template <typename T>
std::pair<typename SystemIdentification<T>::PartialEvalType, T>
SystemIdentification<T>::EstimateParameters(
    const VectorXPoly& polys,
    const std::vector<PartialEvalType>& active_var_values) {
  DRAKE_ASSERT(active_var_values.size() > 0);
  const int num_data = active_var_values.size();
  const int num_err_terms = num_data * polys.rows();

  std::vector<Polynomiald> polys_vec;
  for (int i = 0; i < polys.rows(); i++) {
    polys_vec.push_back(polys[i]);
  }
  const auto var_sets = ClassifyVars(polys_vec, active_var_values);
  const std::set<VarType>& vars_to_estimate_set = std::get<1>(var_sets);

  std::vector<VarType> vars_to_estimate(vars_to_estimate_set.begin(),
                                        vars_to_estimate_set.end());
  int num_to_estimate = vars_to_estimate.size();

  // Make sure we have as many data points as vars we are estimating, or else
  // our solution will be meaningless.
  DRAKE_ASSERT(num_data >= num_to_estimate);

  // Build up our optimization problem's decision variables.
  MathematicalProgram problem;
  VectorXDecisionVariable parameter_variables =
      problem.NewContinuousVariables(num_to_estimate, "param");
  VectorXDecisionVariable error_variables =
      problem.NewContinuousVariables(num_err_terms, "error");

  // Create any necessary VarType IDs.  We build up two lists of VarType:
  //  * problem_vartypes holds a VarType for each decision variable.  This
  //    list will be used to build the constraints.
  //  * error_vartypes holds a VarType for each error variable.  This list
  //    will be used to build the objective function.
  // In addition a temporary set, vars_to_estimate_set, is used for ensuring
  // unique names.
  std::vector<VarType> problem_vartypes = vars_to_estimate;
  std::vector<VarType> error_vartypes;
  std::set<VarType> vars_in_problem = vars_to_estimate_set;
  for (int i = 0; i < num_err_terms; i++) {
    VarType error_var = CreateUnusedVar("err", vars_in_problem);
    vars_in_problem.insert(error_var);
    error_vartypes.push_back(error_var);
    problem_vartypes.push_back(error_var);
  }

  // For each datum, build a constraint with an error term.
  for (int datum_num = 0; datum_num < num_data; datum_num++) {
    VectorXPoly constraint_polys(polys.rows(), 1);
    const PartialEvalType& partial_eval_map = active_var_values[datum_num];
    for (int poly_num = 0; poly_num < polys.rows(); poly_num++) {
      PolyType partial_poly = polys[poly_num].EvaluatePartial(partial_eval_map);
      PolyType constraint_poly =
          partial_poly +
          PolyType(1, error_vartypes[datum_num * polys.rows() + poly_num]);
      constraint_polys[poly_num] = constraint_poly;
    }
    problem.AddPolynomialConstraint(constraint_polys, problem_vartypes,
                                    Eigen::VectorXd::Zero(polys.rows()),
                                    Eigen::VectorXd::Zero(polys.rows()),
                                    {parameter_variables, error_variables});
  }

  // Create a cost function that is least-squares on the error terms.
  auto cost = problem.AddQuadraticCost(
      Eigen::MatrixXd::Identity(num_err_terms, num_err_terms),
      Eigen::VectorXd::Zero(num_err_terms), error_variables).evaluator();

  // Solve the problem and copy out the result.
  const auto result = Solve(problem);
  if (!result.is_success()) {
    std::ostringstream oss;
    oss << "Solution failed: " << result.get_solution_result();
    throw std::runtime_error(oss.str());
  }
  PartialEvalType estimates;
  for (int i = 0; i < num_to_estimate; i++) {
    VarType var = vars_to_estimate[i];
    estimates[var] = result.GetSolution(parameter_variables(i));
  }
  T error_squared = 0;
  for (int i = 0; i < num_err_terms; i++) {
    error_squared += std::pow(result.GetSolution(error_variables(i)), 2);
  }

  return std::make_pair(estimates, std::sqrt(error_squared / num_err_terms));
}

template <typename T>
typename SystemIdentification<T>::SystemIdentificationResult
SystemIdentification<T>::LumpedSystemIdentification(
    const VectorXTrigPoly& trigpolys,
    const std::vector<PartialEvalType>& active_var_values) {
  SystemIdentificationResult result;

// Tracing this method is a very useful way to debug otherwise-obscure
// system identification problems, so use a custom debug output idiom.
// This is a hack until #1895 is resolved.
#define LUMPED_SYSTEM_IDENTIFICATION_VERBOSE 0
// NOLINTNEXTLINE(*)  Don't lint the deliberately evil macro below.
#define debug if (!LUMPED_SYSTEM_IDENTIFICATION_VERBOSE) {} else std::cout

  // Restructure everything as Polynomial plus a unified SinCosMap.
  TrigPolyd::SinCosMap original_sin_cos_map;
  std::vector<Polynomiald> polys;
  for (int i = 0; i < trigpolys.rows(); i++) {
    const TrigPolyd& trigpoly = trigpolys[i];
    debug << "polynomial for ID: " << trigpoly << std::endl;
    polys.push_back(trigpoly.poly());
    for (const auto& k_v_pair : trigpoly.sin_cos_map()) {
      if (original_sin_cos_map.count(k_v_pair.first)) {
        // Make sure that different TrigPolys don't have inconsistent
        // sin_cos_maps (eg, `s = sin(x)` vs. `s = cos(y)`).  Note that
        // nesting violations (`s = sin(y), y = cos(z)`) will be caught by
        // the TrigPoly constructor.
        DRAKE_DEMAND(k_v_pair.second.s ==
                     original_sin_cos_map[k_v_pair.first].s);
        DRAKE_DEMAND(k_v_pair.second.c ==
                     original_sin_cos_map[k_v_pair.first].c);
      } else {
        original_sin_cos_map[k_v_pair.first] = k_v_pair.second;
      }
    }
  }

  // Figure out what vars we are estimating.
  const auto var_sets = ClassifyVars(polys, active_var_values);
  std::set<VarType> parameter_vars = std::get<1>(var_sets);
  for (const auto& k_v_pair : original_sin_cos_map) {
    // If x isn't a param, neither are sin(x) and cos(x).
    if (!parameter_vars.count(k_v_pair.first)) {
      parameter_vars.erase(k_v_pair.second.s);
      parameter_vars.erase(k_v_pair.second.c);
    }
  }
  debug << "Params to estimate: ";
  for (const auto& pv : parameter_vars) {
    debug << Polynomiald::IdToVariableName(pv) << " ";
  }
  debug << std::endl;

  // Compute lumped parameters.
  result.lumped_parameters =
      GetLumpedParametersFromPolynomials(polys, parameter_vars);
  for (const auto& k_v_pair : result.lumped_parameters) {
    debug << "Lumped parameter: "
          << Polynomiald::IdToVariableName(k_v_pair.second)
          << " == " << k_v_pair.first << std::endl;
  }

  // Compute lumped polynomials.
  result.lumped_polys.resize(trigpolys.rows());
  for (int i = 0; i < trigpolys.rows(); i++) {
    result.lumped_polys[i] = TrigPolyd(RewritePolynomialWithLumpedParameters(
                                           polys[i], result.lumped_parameters),
                                       original_sin_cos_map);
    debug << "Lumped polynomial: " << result.lumped_polys[i] << std::endl;
  }

  // Before we can estimated lumped parameters, we need to augment the
  // active_var_values list with the values of any sin and cos variables.
  std::vector<PartialEvalType> augmented_values = active_var_values;
  for (auto& partial_eval : augmented_values) {
    for (const auto& k_v_pair : partial_eval) {
      if (original_sin_cos_map.find(k_v_pair.first) !=
          original_sin_cos_map.end()) {
        partial_eval[original_sin_cos_map[k_v_pair.first].s] =
            sin(k_v_pair.second);
        partial_eval[original_sin_cos_map[k_v_pair.first].c] =
            cos(k_v_pair.second);
      }
    }
  }

  // Estimate the lumped parameters.
  VectorXPoly polys_as_eigen(polys.size());
  for (size_t i = 0; i < polys.size(); i++) {
    polys_as_eigen[i] = result.lumped_polys[i].poly();
  }
  std::tie(result.lumped_parameter_values, result.rms_error) =
      EstimateParameters(polys_as_eigen, augmented_values);
  for (const auto& k_v_pair : result.lumped_parameter_values) {
    debug << "Parameter estimate: "
          << Polynomiald::IdToVariableName(k_v_pair.first)
          << " ~= " << k_v_pair.second << std::endl;
  }
  debug << "Estimation error: " << result.rms_error << std::endl;

  // Substitute the estimates back into the lumped polynomials.
  result.partially_evaluated_polys.resize(trigpolys.rows());
  for (int i = 0; i < trigpolys.rows(); i++) {
    result.partially_evaluated_polys[i] =
        result.lumped_polys[i].EvaluatePartial(result.lumped_parameter_values);
    debug << "Estimated polynomial: " << result.partially_evaluated_polys[i]
          << std::endl;
  }

  return result;
#undef LUMPED_SYSTEM_IDENTIFICATION_VERBOSE
#undef debug
}

template <typename T>
std::tuple<const std::set<typename SystemIdentification<T>::VarType>,
           const std::set<typename SystemIdentification<T>::VarType>,
           const std::set<typename SystemIdentification<T>::VarType>>
SystemIdentification<T>::ClassifyVars(
    const std::vector<Polynomiald>& polys,
    const std::vector<PartialEvalType>& active_var_values) {
  std::set<VarType> all_vars;
  for (const auto& poly : polys) {
    const std::set<VarType> poly_vars = poly.GetVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }

  std::set<VarType> parameter_vars;
  for (const PartialEvalType& partial_eval_map : active_var_values) {
    std::set<VarType> unspecified_vars = all_vars;
    for (auto const& element : partial_eval_map) {
      unspecified_vars.erase(element.first);
    }
    parameter_vars.insert(unspecified_vars.begin(), unspecified_vars.end());
  }

  std::set<VarType> active_vars(all_vars.begin(), all_vars.end());
  for (const auto& param : parameter_vars) {
    active_vars.erase(param);
  }

  return std::make_tuple(all_vars, parameter_vars, active_vars);
}

}  // namespace solvers
}  // namespace drake

template class drake::solvers::SystemIdentification<double>;
