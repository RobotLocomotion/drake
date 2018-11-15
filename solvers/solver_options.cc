#include "drake/solvers/solver_options.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {

// A shorthand for our member field type, for options typed as T's, as in
// MapMap[SolverId][string] => T.
template <typename T>
using MapMap = std::unordered_map<SolverId, std::unordered_map<std::string, T>>;

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

namespace {
// If options has an entry for the given solver_id, returns a reference to the
// mapped value.  Otherwise, returns a long-lived reference to an empty value.
template <typename T>
const std::unordered_map<std::string, T>& GetOptionsHelper(
    const SolverId& solver_id, const MapMap<T>& options) {
  static never_destroyed<std::unordered_map<std::string, T>> empty;
  const auto iter = options.find(solver_id);
  return (iter != options.end()) ? iter->second : empty.access();
}
}  // namespace

const std::unordered_map<std::string, double>& SolverOptions::GetOptionsDouble(
    const SolverId& solver_id) const {
  return GetOptionsHelper(solver_id, solver_options_double_);
}

const std::unordered_map<std::string, int>& SolverOptions::GetOptionsInt(
    const SolverId& solver_id) const {
  return GetOptionsHelper(solver_id, solver_options_int_);
}

const std::unordered_map<std::string, std::string>&
SolverOptions::GetOptionsStr(const SolverId& solver_id) const {
  return GetOptionsHelper(solver_id, solver_options_str_);
}

std::unordered_set<SolverId> SolverOptions::GetSolverIds() const {
  std::unordered_set<SolverId> result;
  for (const auto& pair : solver_options_double_) { result.insert(pair.first); }
  for (const auto& pair : solver_options_int_) { result.insert(pair.first); }
  for (const auto& pair : solver_options_str_) { result.insert(pair.first); }
  return result;
}

template <typename T>
void MergeHelper(const MapMap<T>& other, MapMap<T>* self) {
  for (const auto& other_id_keyvals : other) {
    const SolverId& id = other_id_keyvals.first;
    std::unordered_map<std::string, T>& self_keyvals = (*self)[id];
    for (const auto& other_keyval : other_id_keyvals.second) {
      // This is a no-op when the key already exists.
      self_keyvals.insert(other_keyval);
    }
  }
}

void SolverOptions::Merge(const SolverOptions& other) {
  MergeHelper(other.solver_options_double_, &solver_options_double_);
  MergeHelper(other.solver_options_int_, &solver_options_int_);
  MergeHelper(other.solver_options_str_, &solver_options_str_);
}

bool SolverOptions::operator==(const SolverOptions& other) const {
  return solver_options_double_ == other.solver_options_double_ &&
         solver_options_int_ == other.solver_options_int_ &&
         solver_options_str_ == other.solver_options_str_;
}

bool SolverOptions::operator!=(const SolverOptions& other) const {
  return !(*this == other);
}

}  // namespace solvers
}  // namespace drake
