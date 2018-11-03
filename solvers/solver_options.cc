#include "drake/solvers/solver_options.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {
void SolverOptions::SetSolverOption(const SolverId& solver_id,
                                    const std::string& solver_option,
                                    double option_value) {
  solver_options_double_[solver_id][solver_option] = option_value;
}

void SolverOptions::SetSolverOption(const SolverId& solver_id,
                                    const std::string& solver_option,
                                    int option_value) {
  solver_options_int_[solver_id][solver_option] = option_value;
}

void SolverOptions::SetSolverOption(const SolverId& solver_id,
                                    const std::string& solver_option,
                                    const std::string& option_value) {
  solver_options_str_[solver_id][solver_option] = option_value;
}

const std::unordered_map<std::string, double>&
SolverOptions::GetSolverOptionsDouble(const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_double_;
  static never_destroyed<std::unordered_map<std::string, double>>
      solver_options_double_empty{};
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second
                                 : solver_options_double_empty.access();
}

const std::unordered_map<std::string, int>& SolverOptions::GetSolverOptionsInt(
    const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_int_;
  static never_destroyed<std::unordered_map<std::string, int>>
      solver_options_int_empty{};
  const auto iter = options.find(solver_id);
  // If the option hasn't been set for this solver, return the empty map. Here
  // we save the effort to create an empty map, but return the static one by
  // reference.
  return (iter != options.end()) ? iter->second
                                 : solver_options_int_empty.access();
}

const std::unordered_map<std::string, std::string>&
SolverOptions::GetSolverOptionsStr(const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_str_;
  static never_destroyed<std::unordered_map<std::string, std::string>>
      solver_options_str_empty{};
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second
                                 : solver_options_str_empty.access();
}
}  // namespace solvers
}  // namespace drake
