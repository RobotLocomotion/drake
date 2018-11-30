#pragma once

#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
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
 * https://www.gurobi.com/documentation/7.5/refman/parameters.html
 *
 * "SCS" -- Parameter name and values as specified in the struct SCS_SETTINGS in
 * SCS header file https://github.com/cvxgrp/scs/blob/master/include/scs.h
 * Note that the SCS code on github master might be more up-to-date than the
 * version used in Drake.
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

  const std::unordered_map<std::string, double>& GetOptionsDouble(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, int>& GetOptionsInt(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, std::string>& GetOptionsStr(
      const SolverId& solver_id) const;

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

 private:
  std::unordered_map<SolverId, std::unordered_map<std::string, double>>
      solver_options_double_{};
  std::unordered_map<SolverId, std::unordered_map<std::string, int>>
      solver_options_int_{};
  std::unordered_map<SolverId, std::unordered_map<std::string, std::string>>
      solver_options_str_{};
};

std::string to_string(const SolverOptions&);
std::ostream& operator<<(std::ostream&, const SolverOptions&);

}  // namespace solvers
}  // namespace drake
