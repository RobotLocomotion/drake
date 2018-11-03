#include "drake/solvers/solver_options.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {
void SolverOptions::SetOption(const SolverId& solver_id,
                              const std::string& solver_option,
                              double option_value) {
  solver_options_double_[solver_id][solver_option] = option_value;
}

void SolverOptions::SetOption(const SolverId& solver_id,
                              const std::string& solver_option,
                              int option_value) {
  solver_options_int_[solver_id][solver_option] = option_value;
}

void SolverOptions::SetOption(const SolverId& solver_id,
                              const std::string& solver_option,
                              const std::string& option_value) {
  solver_options_str_[solver_id][solver_option] = option_value;
}

const std::unordered_map<std::string, double>& SolverOptions::GetOptionsDouble(
    const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_double_;
  // For "unset" solver ids, the options are implicitly an empty map. To avoid
  // copying (by always returning a const reference), and to avoid cluttering up
  // each SolverOptions instance with empty maps, we keep a static, empty map
  // for returning when the input solver_id isn't a key in the map.
  static never_destroyed<std::unordered_map<std::string, double>>
      solver_options_double_empty{};
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second
                                 : solver_options_double_empty.access();
}

const std::unordered_map<std::string, int>& SolverOptions::GetOptionsInt(
    const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_int_;
  // For "unset" solver ids, the options are implicitly an empty map. To avoid
  // copying (by always returning a const reference), and to avoid cluttering up
  // each SolverOptions instance with empty maps, we keep a static, empty map
  // for returning when the input solver_id isn't a key in the map.
  static never_destroyed<std::unordered_map<std::string, int>>
      solver_options_int_empty{};
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second
                                 : solver_options_int_empty.access();
}

const std::unordered_map<std::string, std::string>&
SolverOptions::GetOptionsStr(const SolverId& solver_id) const {
  // Aliases for brevity.
  const auto& options = solver_options_str_;
  // For "unset" solver ids, the options are implicitly an empty map. To avoid
  // copying (by always returning a const reference), and to avoid cluttering up
  // each SolverOptions instance with empty maps, we keep a static, empty map
  // for returning when the input solver_id isn't a key in the map.
  static never_destroyed<std::unordered_map<std::string, std::string>>
      solver_options_str_empty{};
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second
                                 : solver_options_str_empty.access();
}
}  // namespace solvers
}  // namespace drake
