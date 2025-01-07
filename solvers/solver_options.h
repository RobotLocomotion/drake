#pragma once

#include <string>
#include <variant>

// Remove on 2025-09-01 upon completion of deprecation.
#include <optional>
#include <ostream>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"
#include "drake/common/name_value.h"
#include "drake/common/string_unordered_map.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {

/** Stores options for multiple solvers. This interface does not do any
verification of solver parameters. It does not even verify that the specified
solver exists. Use this only when you have particular knowledge of what solver
is being invoked, and exactly what tuning is required.

Supported solver names/options:

"SNOPT" -- Parameter names and values as specified in SNOPT User's Guide section
7.7 "Description of the optional parameters", used as described in section 7.5
for snSet(). The SNOPT user guide can be obtained from
https://web.stanford.edu/group/SOL/guides/sndoc7.pdf

"IPOPT" -- Parameter names and values as specified in IPOPT users guide section
"Options Reference" https://coin-or.github.io/Ipopt/OPTIONS.html

"NLOPT" -- Parameter names and values are specified in
https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/ (in the
Stopping criteria section). Besides these parameters, the user can specify
"algorithm" using a string of the algorithm name. The complete set of algorithms
is listed in "nlopt_algorithm_to_string()" function in
github.com/stevengj/nlopt/blob/master/src/api/general.c. If you would like to
use certain algorithm, for example NLOPT_LD_SLSQP, call
`SetOption(NloptSolver::id(), NloptSolver::AlgorithmName(), "LD_SLSQP");`

"GUROBI" -- Parameter name and values as specified in Gurobi Reference Manual,
section 10.2 "Parameter Descriptions"
https://www.gurobi.com/documentation/10.0/refman/parameters.html

"SCS" -- Parameter name and values as specified in the struct SCS_SETTINGS in
SCS header file https://github.com/cvxgrp/scs/blob/master/include/scs.h Note
that the SCS code on github master might be more up-to-date than the version
used in Drake.

"MOSEK™" -- Parameter name and values as specified in Mosek Reference
https://docs.mosek.com/9.3/capi/parameters.html

"OSQP" -- Parameter name and values as specified in OSQP Reference
https://osqp.org/docs/interfaces/solver_settings.html#solver-settings

"Clarabel" -- Parameter name and values as specified in Clarabel
https://oxfordcontrol.github.io/ClarabelDocs/stable/api_settings/
Note that `direct_solve_method` is not supported in Drake yet. Clarabel's
boolean options should be passed as integers (0 or 1).

"CSDP" -- Parameter name and values as specified at
https://manpages.ubuntu.com/manpages/focal/en/man1/csdp-randgraph.1.html */
struct SolverOptions final {
 public:
  /** The values stored in SolverOptions can be double, int, or string.
  In the future, we might re-order or add more allowed types without any
  deprecation period, so be sure to use std::visit or std::get<T> to
  retrieve the variant's value in a future-proof way. */
  using OptionValue = std::variant<double, int, std::string>;

  /** Sets a solver option for a specific solver. If the solver doesn't support
  the option, it will throw an exception during the Solve (not when setting the
  option here). */
  void SetOption(const SolverId& solver_id, std::string key, OptionValue value);

  /** Sets a common option for all solvers supporting that option (for example,
  printing the progress in each iteration). If the solver doesn't support the
  option, the option is ignored. */
  void SetOption(CommonSolverOption key, OptionValue value);

  /** Merges the other solver options into this. If `other` and `this` option
  both define the same option for the same solver, we ignore the one from
  `other` and keep the one from `this`. */
  void Merge(const SolverOptions& other);

  bool operator==(const SolverOptions& other) const;
  bool operator!=(const SolverOptions& other) const;
  std::string to_string() const;

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(options));
  }

  /** The options are indexed first by the solver name and second by the key.
  In the case of Drake's common options, the solver name is "Drake". */
  string_unordered_map<string_unordered_map<OptionValue>> options;

  // ==========================================================================
  // EVERYTHING AFTER THIS POINT IN THIS STRUCT IS DEPRECATION GOOP; IGNORE IT.
  // ==========================================================================

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::unordered_map<std::string, double> GetOptionsDouble(
      const SolverId& solver_id) const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::unordered_map<std::string, int> GetOptionsInt(
      const SolverId& solver_id) const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::unordered_map<std::string, std::string> GetOptionsStr(
      const SolverId& solver_id) const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::unordered_map<CommonSolverOption, OptionValue> common_solver_options()
      const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::string get_print_file_name() const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  bool get_print_to_console() const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::string get_standalone_reproduction_file_name() const;

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::optional<int> get_max_threads() const;

  template <typename T>
  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::unordered_map<std::string, T> GetOptions(
      const SolverId& solver_id) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    if constexpr (std::is_same_v<T, double>) {
      return GetOptionsDouble(solver_id);
    } else if constexpr (std::is_same_v<T, int>) {
      return GetOptionsInt(solver_id);
    } else if constexpr (std::is_same_v<T, std::string>) {
      return GetOptionsStr(solver_id);
    }
#pragma GCC diagnostic pop
    DRAKE_UNREACHABLE();
  }

  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  std::unordered_set<SolverId> GetSolverIds() const;

  // Check if for a given solver_id, the option keys are included in
  // double_keys, int_keys and str_keys.
  // @param solver_id If this SolverOptions has set options for this solver_id,
  // then we check if the option keys are a subset of `double_keys`, `int_keys`
  // and `str_keys`.
  // @param double_keys The set of allowable keys for double options.
  // @param int_keys The set of allowable keys for int options.
  // @param str_keys The set of allowable keys for string options.
  // @throws std::exception if the solver contains un-allowed options.
  DRAKE_DEPRECATED("2025-09-01", "Access the 'options' directly, instead.")
  void CheckOptionKeysForSolver(
      const SolverId& solver_id,
      const std::unordered_set<std::string>& allowable_double_keys,
      const std::unordered_set<std::string>& allowable_int_keys,
      const std::unordered_set<std::string>& allowable_str_keys) const;

 private:
  // Remove this field on 2025-09-01 upon completion of deprecation.
  std::unordered_set<SolverId> solver_ids;
};

DRAKE_DEPRECATED("2025-09-01", "Use options.to_string(), instead.")
std::string to_string(const SolverOptions&);

DRAKE_DEPRECATED("2025-09-01", "Use options.to_string(), instead.")
std::ostream& operator<<(std::ostream&, const SolverOptions&);

namespace internal {
/* Converts the option_value to a string. */
std::string OptionValueToString(const SolverOptions::OptionValue& option_value);
}  // namespace internal

}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, SolverOptions, x, x.to_string())
