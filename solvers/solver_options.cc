#include "drake/solvers/solver_options.h"

#include <map>
#include <sstream>
#include <utility>

#include "fmt/format.h"
#include "fmt/ostream.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {

// A shorthand for our member field type, for options typed as T's, as in
// MapMap[SolverId][string] => T.
template <typename T>
using MapMap = std::unordered_map<SolverId, std::unordered_map<std::string, T>>;

// A shorthand for our member field type.
using CommonMap =
    std::unordered_map<CommonSolverOption, SolverOptions::OptionValue>;

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

void SolverOptions::SetOption(CommonSolverOption key, OptionValue value) {
  switch (key) {
    case CommonSolverOption::kPrintToConsole: {
      if (!std::holds_alternative<int>(value)) {
        throw std::runtime_error(fmt::format(
            "SolverOptions::SetOption support {} only with int value.", key));
      }
      const int int_value = std::get<int>(value);
      if (int_value != 0 && int_value != 1) {
        throw std::runtime_error(
            fmt::format("{} expects value either 0 or 1", key));
      }
      common_solver_options_[key] = std::move(value);
      return;
    }
    case CommonSolverOption::kPrintFileName: {
      if (!std::holds_alternative<std::string>(value)) {
        throw std::runtime_error(fmt::format(
            "SolverOptions::SetOption support {} only with std::string value.",
            key));
      }
      common_solver_options_[key] = std::move(value);
      return;
    }
  }
  DRAKE_UNREACHABLE();
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

std::string SolverOptions::get_print_file_name() const {
  // N.B. SetOption sanity checks the value; we don't need to re-check here.
  std::string result;
  auto iter = common_solver_options_.find(CommonSolverOption::kPrintFileName);
  if (iter != common_solver_options_.end()) {
    result = std::get<std::string>(iter->second);
  }
  return result;
}

bool SolverOptions::get_print_to_console() const {
  // N.B. SetOption sanity checks the value; we don't need to re-check here.
  bool result = false;
  auto iter = common_solver_options_.find(CommonSolverOption::kPrintToConsole);
  if (iter != common_solver_options_.end()) {
    const int value = std::get<int>(iter->second);
    result = static_cast<bool>(value);
  }
  return result;
}

std::unordered_set<SolverId> SolverOptions::GetSolverIds() const {
  std::unordered_set<SolverId> result;
  for (const auto& pair : solver_options_double_) { result.insert(pair.first); }
  for (const auto& pair : solver_options_int_) { result.insert(pair.first); }
  for (const auto& pair : solver_options_str_) { result.insert(pair.first); }
  return result;
}

namespace {
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

void MergeHelper(const CommonMap& other, CommonMap* self) {
  for (const auto& other_keyval : other) {
    // This is a no-op when the key already exists.
    self->insert(other_keyval);
  }
}
}  // namespace

void SolverOptions::Merge(const SolverOptions& other) {
  MergeHelper(other.solver_options_double_, &solver_options_double_);
  MergeHelper(other.solver_options_int_, &solver_options_int_);
  MergeHelper(other.solver_options_str_, &solver_options_str_);
  MergeHelper(
      other.common_solver_options_, &common_solver_options_);
}

bool SolverOptions::operator==(const SolverOptions& other) const {
  return solver_options_double_ == other.solver_options_double_ &&
         solver_options_int_ == other.solver_options_int_ &&
         solver_options_str_ == other.solver_options_str_ &&
         common_solver_options_ == other.common_solver_options_;
}

bool SolverOptions::operator!=(const SolverOptions& other) const {
  return !(*this == other);
}

namespace {
template <typename T>
void Summarize(const SolverId& id,
           const std::unordered_map<std::string, T>& keyvals,
           std::map<std::string, std::string>* pairs) {
  for (const auto& keyval : keyvals) {
    (*pairs)[fmt::format("{}:{}", id.name(), keyval.first)] =
        fmt::format("{}", keyval.second);
  }
}

}  // namespace

std::ostream& operator<<(std::ostream& os, const SolverOptions& x) {
  os << "{SolverOptions";
  const auto& ids = x.GetSolverIds();
  if (ids.empty()) {
    os << " empty";
  } else {
    // Map keyed on "solver_name:option_key" so our output is deterministic.
    std::map<std::string, std::string> pairs;
    for (const auto& id : ids) {
      Summarize(id, x.GetOptionsDouble(id), &pairs);
      Summarize(id, x.GetOptionsInt(id), &pairs);
      Summarize(id, x.GetOptionsStr(id), &pairs);
    }
    for (const auto& keyval : x.common_solver_options()) {
      const CommonSolverOption& key = keyval.first;
      const auto& val = keyval.second;
      std::visit(
          [key, &pairs](auto& val_x) {
            pairs[fmt::format("CommonSolverOption::{}", key)] =
                fmt::format("{}", val_x);
          },
          val);
    }
    for (const auto& pair : pairs) {
      os << ", " << pair.first << "=" << pair.second;
    }
  }
  os << "}";
  return os;
}

std::string to_string(const SolverOptions& x) {
  std::ostringstream result;
  result << x;
  return result.str();
}

namespace {
// Check if all the keys in key_value pair key_vals is a subset of
// allowable_keys, and throw an invalid argument if not.
template <typename T>
void CheckOptionKeysForSolverHelper(
    const std::unordered_map<std::string, T>& key_vals,
    const std::unordered_set<std::string>& allowable_keys,
    const std::string& solver_name) {
  for (const auto& key_val : key_vals) {
    if (allowable_keys.count(key_val.first) == 0) {
      throw std::invalid_argument(key_val.first +
                                  " is not allowed in the SolverOptions for " +
                                  solver_name + ".");
    }
  }
}
}  // namespace

void SolverOptions::CheckOptionKeysForSolver(
    const SolverId& solver_id,
    const std::unordered_set<std::string>& double_keys,
    const std::unordered_set<std::string>& int_keys,
    const std::unordered_set<std::string>& str_keys) const {
  CheckOptionKeysForSolverHelper(GetOptionsDouble(solver_id), double_keys,
                                 solver_id.name());
  CheckOptionKeysForSolverHelper(GetOptionsInt(solver_id), int_keys,
                                 solver_id.name());
  CheckOptionKeysForSolverHelper(GetOptionsStr(solver_id), str_keys,
                                 solver_id.name());
}

}  // namespace solvers
}  // namespace drake
