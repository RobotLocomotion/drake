#include "drake/solvers/solver_options.h"

#include <algorithm>
#include <utility>
#include <vector>

// Remove on 2025-05-01 upon completion of deprecation.
#include <sstream>

#include <fmt/format.h>

#include "drake/common/overloaded.h"

namespace drake {
namespace solvers {

using OptionValue = SolverOptions::OptionValue;

namespace {
constexpr const char kCommonSolverOptionKey[] = "Drake";
}  // namespace

namespace internal {
/* Returns the pythonic repr() for the given value. */
std::string Repr(const OptionValue& option_value) {
  return std::visit(
      [&](const auto& unwrapped_value) {
        // XXX needs proper python quoting or whatever
        return fmt::format("{}", unwrapped_value);
      },
      option_value);
}
}  // namespace internal

void SolverOptions::SetOption(const SolverId& solver_id,
                              std::string solver_option,
                              OptionValue option_value) {
  options[solver_id.name()][std::move(solver_option)] = std::move(option_value);
}

void SolverOptions::SetOption(CommonSolverOption key, OptionValue value) {
  switch (key) {
    case CommonSolverOption::kPrintToConsole: {
      if (std::holds_alternative<int>(value)) {
        const int int_value = std::get<int>(value);
        if (int_value == 0 || int_value == 1) {
          options[kCommonSolverOptionKey][fmt::to_string(key)] =
              std::move(value);
          return;
        }
      }
      throw std::runtime_error(fmt::format(
          "SolverOptions::SetOption({}) value must be 0 or 1, not {}.", key,
          internal::Repr(value)));
      return;
    }
    case CommonSolverOption::kPrintFileName: {
      if (std::holds_alternative<std::string>(value)) {
        options[kCommonSolverOptionKey][fmt::to_string(key)] = std::move(value);
        return;
      }
      throw std::runtime_error(fmt::format(
          "SolverOptions::SetOption({}) value must be a string, not {}.", key,
          internal::Repr(value)));
      return;
    }
    case CommonSolverOption::kStandaloneReproductionFileName: {
      if (!std::holds_alternative<std::string>(value)) {
        throw std::runtime_error(fmt::format(
            "SolverOptions::SetOption support {} only with std::string value.",
            key));
      }
      options[kCommonSolverOptionKey][fmt::to_string(key)] = std::move(value);
      return;
    }
    case CommonSolverOption::kMaxThreads: {
      if (!std::holds_alternative<int>(value)) {
        throw std::runtime_error(fmt::format(
            "SolverOptions::SetOption support {} only with int value.", key));
      }
      const int int_value = std::get<int>(value);
      if (int_value <= 0) {
        throw std::runtime_error(
            fmt::format("kMaxThreads must be > 0, got {}", int_value));
      }
      options[kCommonSolverOptionKey][fmt::to_string(key)] = std::move(value);
      return;
    }
  }
  DRAKE_UNREACHABLE();
}

void SolverOptions::Merge(const SolverOptions& other) {
  for (const auto& [other_solver, other_keyvals] : other.options) {
    string_unordered_map<OptionValue>& self_keyvals =
        this->options[other_solver];
    for (const auto& other_keyval : other_keyvals) {
      // This is a no-op when the key already exists.
      self_keyvals.insert(other_keyval);
    }
  }
}

bool SolverOptions::operator==(const SolverOptions& other) const {
  return options == other.options;
}

bool SolverOptions::operator!=(const SolverOptions& other) const {
  return !(*this == other);
}

namespace {
/* Returns keys of the given map in sorted order, paired up with a "bool comma"
marker that is set to false for the last item only. The keys are string_view
aliases into `map`, so the `map` object must outlive this result. */
template <typename Map>
std::vector<std::pair<std::string_view, bool>> SortedKeys(const Map& map) {
  std::vector<std::pair<std::string_view, bool>> result;
  if (!map.empty()) {
    for (const auto& [key, _] : map) {
      result.push_back({std::string_view{key}, true});
    }
    std::sort(result.begin(), result.end());
    result.back().second = false;
  }
  return result;
}
}  // namespace

std::string SolverOptions::to_string() const {
  std::ostringstream result;
  result << "SolverOptions(options={";
  for (const auto& [solver_name, comma1] : SortedKeys(options)) {
    result << fmt::format("\"{}\":{{", solver_name);
    const auto iter1 = options.find(solver_name);
    DRAKE_DEMAND(iter1 != options.end());
    const string_unordered_map<OptionValue>& solver_options = iter1->second;
    for (const auto& [key, comma2] : SortedKeys(solver_options)) {
      const auto iter2 = solver_options.find(key);
      DRAKE_DEMAND(iter2 != solver_options.end());
      const OptionValue& value = iter2->second;
      result << internal::Repr(value);
      if (comma2) {
        result << ", ";
      }
    }
    result << "}";
    if (comma1) {
      result << ", ";
    }
  }
  result << "})";
  return result.str();
}

// ========================================================================
// EVERYTHING AFTER THIS POINT IN THIS FILE IS DEPRECATION GOOP; IGNORE IT.
// Remove all of the below on 2025-05-01 upon completion of deprecation.
// ========================================================================

namespace {
// If options has an entry for the given solver_id, returns a copy of the mapped
// value, filered by the requested type `T`. Otherwise, returns an empty map.
template <typename T>
std::unordered_map<std::string, T> GetOptionsHelper(
    const SolverOptions& solver_options, const std::string_view solver_name) {
  std::unordered_map<std::string, T> result;
  const auto iter = solver_options.options.find(solver_name);
  if (iter != solver_options.options.end()) {
    const string_unordered_map<OptionValue>& items = iter->second;
    for (const auto& [key, value] : items) {
      if (std::holds_alternative<T>(value)) {
        result[key] = std::get<T>(value);
      }
    }
  }
  return result;
}
}  // namespace

std::unordered_map<std::string, double> SolverOptions::GetOptionsDouble(
    const SolverId& solver_id) const {
  return GetOptionsHelper<double>(*this, solver_id.name());
}

std::unordered_map<std::string, int> SolverOptions::GetOptionsInt(
    const SolverId& solver_id) const {
  return GetOptionsHelper<int>(*this, solver_id.name());
}

std::unordered_map<std::string, std::string> SolverOptions::GetOptionsStr(
    const SolverId& solver_id) const {
  return GetOptionsHelper<std::string>(*this, solver_id.name());
}

std::unordered_map<CommonSolverOption, OptionValue>
SolverOptions::common_solver_options() const {
  std::unordered_map<CommonSolverOption, OptionValue> result;
  const auto iter1 = options.find(kCommonSolverOptionKey);
  if (iter1 != options.end()) {
    const string_unordered_map<OptionValue>& common = iter1->second;
    for (auto key : {CommonSolverOption::kPrintToConsole,
                     CommonSolverOption::kPrintFileName}) {
      const auto iter2 = common.find(fmt::to_string(key));
      if (iter2 != common.end()) {
        const OptionValue& value = iter2->second;
        result[key] = value;
      }
    }
  }
  return result;
}

std::string SolverOptions::get_print_file_name() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unordered_map<CommonSolverOption, OptionValue> common =
      common_solver_options();
#pragma GCC diagnostic pop
  // N.B. SetOption sanity checks the value; we don't need to re-check here.
  std::string result;
  auto iter = common.find(CommonSolverOption::kPrintFileName);
  if (iter != common.end()) {
    result = std::get<std::string>(iter->second);
  }
  return result;
}

bool SolverOptions::get_print_to_console() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unordered_map<CommonSolverOption, OptionValue> common =
      common_solver_options();
#pragma GCC diagnostic pop
  // N.B. SetOption sanity checks the value; we don't need to re-check here.
  bool result = false;
  auto iter = common.find(CommonSolverOption::kPrintToConsole);
  if (iter != common.end()) {
    const int value = std::get<int>(iter->second);
    result = static_cast<bool>(value);
  }
  return result;
}

std::unordered_set<SolverId> SolverOptions::GetSolverIds() const {
  return solver_ids;
}

void SolverOptions::CheckOptionKeysForSolver(
    const SolverId& solver_id,
    const std::unordered_set<std::string>& double_keys,
    const std::unordered_set<std::string>& int_keys,
    const std::unordered_set<std::string>& str_keys) const {
  auto iter = options.find(solver_id.name());
  if (iter == options.end()) {
    return;
  }
  for (const auto& [key, value] : iter->second) {
    const bool allowed =
        std::visit(overloaded{[&key, &double_keys](const double&) {
                                return double_keys.count(key);
                              },
                              [&key, &int_keys](const int&) {
                                return int_keys.count(key);
                              },
                              [&key, &str_keys](const std::string&) {
                                return str_keys.count(key);
                              }},
                   value);
    if (!allowed) {
      throw std::invalid_argument(
          fmt::format("{} is not allowed in the SolverOptions for {}", key,
                      solver_id.name()));
    }
  }
}

std::string to_string(const SolverOptions& x) {
  return x.to_string();
}

std::ostream& operator<<(std::ostream& os, const SolverOptions& x) {
  os << x.to_string();
  return os;
}

}  // namespace solvers
}  // namespace drake
