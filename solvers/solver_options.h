#pragma once

#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * Stores options for multiple solvers.  This interface does not
 * do any verification of solver parameters. It does not even verify that
 * the specified solver exists.  Use this only when you have
 * particular knowledge of what solver is being invoked, and exactly
 * what tuning is required.
 *
 * Supported solver names/options:
 *
 * "SNOPT" -- Parameter names and values as specified in SNOPT
 * User's Guide section 7.7 "Description of the optional parameters",
 * used as described in section 7.5 for snSet().
 * The SNOPT user guide can be obtained from
 * https://web.stanford.edu/group/SOL/guides/sndoc7.pdf
 *
 * "IPOPT" -- Parameter names and values as specified in IPOPT users
 * guide section "Options Reference"
 * http://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * "GUROBI" -- Parameter name and values as specified in Gurobi Reference
 * Manual, section 10.2 "Parameter Descriptions"
 * https://www.gurobi.com/documentation/8.0/refman/parameters.html
 *
 * "SCS" -- Parameter name and values as specified in the struct SCS_SETTINGS in
 * SCS header file https://github.com/cvxgrp/scs/blob/master/include/scs.h
 * Note that the SCS code on github master might be more up-to-date than the
 * version used in Drake.
 *
 * "MOSEK" -- Parameter name and values as specified in Mosek Reference
 * https://docs.mosek.com/9.0/capi/parameters.html
 */
class SolverOptions {
 public:
  SolverOptions() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverOptions)

  void SetOption(const SolverId& solver_id, const std::string& solver_option,
                 double option_value);

  void SetOption(const SolverId& solver_id, const std::string& solver_option,
                 int option_value);

  void SetOption(const SolverId& solver_id, const std::string& solver_option,
                 const std::string& option_value);

  /// Set common options for all solvers supporting that option (for example,
  /// printing the progress in each iteration). If the solver doesn't support
  /// the option, the option is ignored.
  void SetOption(CommonSolverOption key,
                 const std::variant<double, int, std::string>& value);

  const std::unordered_map<std::string, double>& GetOptionsDouble(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, int>& GetOptionsInt(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, std::string>& GetOptionsStr(
      const SolverId& solver_id) const;

  /**
   * Gets the common options for all solvers. Refer to CommonSolverOption for
   * more details.
   */
  const std::unordered_map<CommonSolverOption,
                           std::variant<double, int, std::string>>&
  common_solver_options() const {
    return common_solver_options_;
  }

  template <typename T>
  const std::unordered_map<std::string, T>& GetOptions(
      const SolverId& solver_id) const {
    if constexpr (std::is_same_v<T, double>) {
      return GetOptionsDouble(solver_id);
    } else if constexpr (std::is_same_v<T, int>) {
      return GetOptionsInt(solver_id);
    } else if constexpr (std::is_same_v<T, std::string>) {
      return GetOptionsStr(solver_id);
    }
    DRAKE_UNREACHABLE();
  }

  /** Returns the IDs that have any option set. */
  std::unordered_set<SolverId> GetSolverIds() const;

  /**
   * Merges the other solver options into this. If `other` and `this` option
   * both define the same option for the same solver, we ignore then one from
   * `other` and keep the one from `this`.
   */
  void Merge(const SolverOptions& other);

  /**
   * Returns true if `this` and `other` have exactly the same solvers, with
   * exactly the same keys and values for the options for each solver.
   */
  bool operator==(const SolverOptions& other) const;

  /**
   * Negate operator==.
   */
  bool operator!=(const SolverOptions& other) const;

  /**
   * Check if for a given solver_id, the option keys are included in
   * double_keys, int_keys and str_keys.
   * @param solver_id If this SolverOptions has set options for this solver_id,
   * then we check if the option keys are a subset of `double_keys`, `int_keys`
   * and `str_keys`.
   * @param double_keys The set of allowable keys for double options.
   * @param int_keys The set of allowable keys for int options.
   * @param str_keys The set of allowable keys for string options.
   * @throw invalid_argument if the solver contains un-allowed options.
   */
  void CheckOptionKeysForSolver(
      const SolverId& solver_id,
      const std::unordered_set<std::string>& allowable_double_keys,
      const std::unordered_set<std::string>& allowable_int_keys,
      const std::unordered_set<std::string>& allowable_str_keys) const;

 private:
  std::unordered_map<SolverId, std::unordered_map<std::string, double>>
      solver_options_double_{};
  std::unordered_map<SolverId, std::unordered_map<std::string, int>>
      solver_options_int_{};
  std::unordered_map<SolverId, std::unordered_map<std::string, std::string>>
      solver_options_str_{};

  std::unordered_map<CommonSolverOption, std::variant<double, int, std::string>>
      common_solver_options_{};
};

std::string to_string(const SolverOptions&);
std::ostream& operator<<(std::ostream&, const SolverOptions&);

}  // namespace solvers
}  // namespace drake
