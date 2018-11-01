#include "drake/solvers/solver_option.h"

namespace drake {
namespace solvers {
void SolverOption::SetSolverOption(const SolverId& solver_id,
                                   const std::string& solver_option,
                                   double option_value) {
  solver_options_double_[solver_id][solver_option] = option_value;
}

void SolverOption::SetSolverOption(const SolverId& solver_id,
                                   const std::string& solver_option,
                                   int option_value) {
  solver_options_int_[solver_id][solver_option] = option_value;
}

void SolverOption::SetSolverOption(const SolverId& solver_id,
                                   const std::string& solver_option,
                                   const std::string& option_value) {
  solver_options_str_[solver_id][solver_option] = option_value;
}

const std::unordered_map<std::string, double>&
SolverOption::GetSolverOptionsDouble(const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_double_;
  const auto& empty = solver_options_double_empty_;
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second : empty;
}

const std::unordered_map<std::string, int>& SolverOption::GetSolverOptionsInt(
    const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_int_;
  const auto& empty = solver_options_int_empty_;
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second : empty;
}

const std::unordered_map<std::string, std::string>&
SolverOption::GetSolverOptionsStr(const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_str_;
  const auto& empty = solver_options_str_empty_;
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second : empty;
}
}  // namespace solvers
}  // namespace drake
