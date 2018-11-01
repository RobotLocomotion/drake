#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * Stores options for solvers.  This interface does not
 * do any verification of solver parameters beyond what an
 * individual solver does for itself.  It does not even verify that
 * the specified solver exists.  Use this only when you have
 * particular knowledge of what solver is being invoked, and exactly
 * what tuning is required.
 *
 * Supported solver names/options:
 *
 * "SNOPT" -- Parameter names and values as specified in SNOPT
 * User's Guide section 7.7 "Description of the optional parameters",
 * used as described in section 7.5 for snSet().
 *
 * "IPOPT" -- Parameter names and values as specified in IPOPT users
 * guide section "Options Reference"
 * http://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * "GUROBI" -- Parameter name and values as specified in Gurobi Reference
 * Manual, section 10.2 "Parameter Descriptions"
 * https://www.gurobi.com/documentation/7.5/refman/parameters.html
 */
class SolverOptions {
 public:
  SolverOptions();

  SolverOptions(const SolverOptions& other) = default;

  SolverOptions(SolverOptions&& other) = default;

  SolverOptions& operator=(const SolverOptions&);

  SolverOptions& operator=(SolverOptions&&);

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, double option_value);

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, int option_value);

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option,
                       const std::string& option_value);

  const std::unordered_map<std::string, double>& GetSolverOptionsDouble(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, int>& GetSolverOptionsInt(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, std::string>& GetSolverOptionsStr(
      const SolverId& solver_id) const;

 private:
  std::map<SolverId, std::unordered_map<std::string, double>>
      solver_options_double_{};
  std::map<SolverId, std::unordered_map<std::string, int>>
      solver_options_int_{};
  std::map<SolverId, std::unordered_map<std::string, std::string>>
      solver_options_str_{};

  const std::unordered_map<std::string, double> solver_options_double_empty_{};
  const std::unordered_map<std::string, int> solver_options_int_empty_{};
  const std::unordered_map<std::string, std::string>
      solver_options_str_empty_{};
};
}  // namespace solvers
}  // namespace drake
