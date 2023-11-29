#pragma once

#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/fmt_ostream.h"
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
 * https://coin-or.github.io/Ipopt/OPTIONS.html
 *
 * "NLOPT" -- Parameter names and values are specified in
 * https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/ (in the
 * Stopping criteria section). Besides these parameters, the user can specify
 * "algorithm" using a string of the algorithm name. The complete set of
 * algorithms is listed in "nlopt_algorithm_to_string()" function in
 * github.com/stevengj/nlopt/blob/master/src/api/general.c. If you would like to
 * use certain algorithm, for example NLOPT_LD_SLSQP, call
 * `SetOption(NloptSolver::id(), NloptSolver::AlgorithmName(), "LD_SLSQP");`
 *
 * "GUROBI" -- Parameter name and values as specified in Gurobi Reference
 * Manual, section 10.2 "Parameter Descriptions"
 * https://www.gurobi.com/documentation/10.0/refman/parameters.html
 *
 * "SCS" -- Parameter name and values as specified in the struct SCS_SETTINGS in
 * SCS header file https://github.com/cvxgrp/scs/blob/master/include/scs.h
 * Note that the SCS code on github master might be more up-to-date than the
 * version used in Drake.
 *
 * "MOSEK™" -- Parameter name and values as specified in Mosek Reference
 * https://docs.mosek.com/9.3/capi/parameters.html
 *
 * "OSQP" -- Parameter name and values as specified in OSQP Reference
 * https://osqp.org/docs/interfaces/solver_settings.html#solver-settings
 *
 * "Clarabel" -- Parameter name and values as specified in Clarabel
 * https://oxfordcontrol.github.io/ClarabelDocs/stable/api_settings/
 * Note that `direct_solve_method` is not supported in Drake yet.
 * Clarabel's boolean options should be passed as integers (0 or 1).
 */
class SolverOptions {
 public:
  SolverOptions() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverOptions)

  /** The values stored in SolverOptions can be double, int, or string.
   * In the future, we might re-order or add more allowed types without any
   * deprecation period, so be sure to use std::visit or std::get<T> to
   * retrieve the variant's value in a future-proof way. */
  using OptionValue = std::variant<double, int, std::string>;

  /** Sets a double-valued solver option for a specific solver.
   * @pydrake_mkdoc_identifier{double_option}
   */
  void SetOption(const SolverId& solver_id, const std::string& solver_option,
                 double option_value);

  /** Sets an integer-valued solver option for a specific solver.
   * @pydrake_mkdoc_identifier{int_option}
   */
  void SetOption(const SolverId& solver_id, const std::string& solver_option,
                 int option_value);

  /** Sets a string-valued solver option for a specific solver.
   * @pydrake_mkdoc_identifier{str_option}
   */
  void SetOption(const SolverId& solver_id, const std::string& solver_option,
                 const std::string& option_value);

  /** Sets a common option for all solvers supporting that option (for example,
   * printing the progress in each iteration). If the solver doesn't support
   * the option, the option is ignored.
   * @pydrake_mkdoc_identifier{common_option} */
  void SetOption(CommonSolverOption key, OptionValue value);

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
  const std::unordered_map<CommonSolverOption, OptionValue>&
  common_solver_options() const {
    return common_solver_options_;
  }

  /** Returns the kPrintFileName set via CommonSolverOption, or else an empty
   * string if the option has not been set. */
  std::string get_print_file_name() const;

  /** Returns the kPrintToConsole set via CommonSolverOption, or else false if
   * the option has not been set. */
  bool get_print_to_console() const;

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
   * @throws std::exception if the solver contains un-allowed options.
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

  std::unordered_map<CommonSolverOption, OptionValue> common_solver_options_{};
};

std::string to_string(const SolverOptions&);
std::ostream& operator<<(std::ostream&, const SolverOptions&);

}  // namespace solvers
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::solvers::SolverOptions> : drake::ostream_formatter {};
}  // namespace fmt
