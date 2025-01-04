#include "drake/solvers/solver_options.h"

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/overloaded.h"

namespace drake {
namespace solvers {

using OptionValue = SolverOptions::OptionValue;

namespace {
/* The 'options' map uses this "solver name" for the common options. */
constexpr const char kCommonKey[] = "Drake";
}  // namespace

namespace internal {

/* Returns a suitable repr-like string for the given value. */
std::string OptionValueToString(const OptionValue& option_value) {
  return std::visit(overloaded{[](const double& unboxed_value) {
                                 return fmt_floating_point(unboxed_value);
                               },
                               [](const int& unboxed_value) {
                                 return fmt::to_string(unboxed_value);
                               },
                               [](const std::string& unboxed_value) {
                                 return fmt_debug_string(unboxed_value);
                               }},
                    option_value);
}

}  // namespace internal

void SolverOptions::SetOption(const SolverId& solver_id, std::string key,
                              OptionValue option_value) {
  options[solver_id.name()][std::move(key)] = std::move(option_value);

  // Remove on 2025-09-01 upon completion of deprecation.
  solver_ids.insert(solver_id);
}

void SolverOptions::SetOption(CommonSolverOption key, OptionValue value) {
  switch (key) {
    case CommonSolverOption::kPrintToConsole: {
      if (std::holds_alternative<int>(value)) {
        const int int_value = std::get<int>(value);
        if (int_value == 0 || int_value == 1) {
          options[kCommonKey][fmt::to_string(key)] = std::move(value);
          return;
        }
      }
      throw std::runtime_error(fmt::format(
          "SolverOptions::SetOption({}) value must be 0 or 1, not {}.", key,
          internal::OptionValueToString(value)));
      return;
    }
    case CommonSolverOption::kPrintFileName: {
      if (std::holds_alternative<std::string>(value)) {
        options[kCommonKey][fmt::to_string(key)] = std::move(value);
        return;
      }
      throw std::runtime_error(fmt::format(
          "SolverOptions::SetOption({}) value must be a string, not {}.", key,
          internal::OptionValueToString(value)));
      return;
    }
    case CommonSolverOption::kStandaloneReproductionFileName: {
      if (std::holds_alternative<std::string>(value)) {
        options[kCommonKey][fmt::to_string(key)] = std::move(value);
        return;
      }
      throw std::runtime_error(fmt::format(
          "SolverOptions::SetOption({}) value must be a string, not {}.", key,
          internal::OptionValueToString(value)));
    }
    case CommonSolverOption::kMaxThreads: {
      if (std::holds_alternative<int>(value)) {
        const int int_value = std::get<int>(value);
        if (int_value > 0) {
          options[kCommonKey][fmt::to_string(key)] = std::move(value);
          return;
        }
      }
      throw std::runtime_error(fmt::format(
          "SolverOptions::SetOption({}) value must be an int > 0, not {}.", key,
          internal::OptionValueToString(value)));
    }
  }
  DRAKE_UNREACHABLE();
}

namespace {
// If options has an entry for the given solver_id, returns a copy of the mapped
// value, filered by the requested type `T`. Otherwise, returns an empty map.
// This function should be removed when the deprecations 2025-09-01 expire.
template <typename T>
std::unordered_map<std::string, T> GetOptionsHelper(
    const SolverId& solver_id, const SolverOptions& solver_options) {
  std::unordered_map<std::string, T> result;
  const auto iter = solver_options.options.find(solver_id.name());
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

// Deprecated 2025-09-01.
std::unordered_map<std::string, double> SolverOptions::GetOptionsDouble(
    const SolverId& solver_id) const {
  return GetOptionsHelper<double>(solver_id, *this);
}

// Deprecated 2025-09-01.
std::unordered_map<std::string, int> SolverOptions::GetOptionsInt(
    const SolverId& solver_id) const {
  return GetOptionsHelper<int>(solver_id, *this);
}

// Deprecated 2025-09-01.
std::unordered_map<std::string, std::string> SolverOptions::GetOptionsStr(
    const SolverId& solver_id) const {
  return GetOptionsHelper<std::string>(solver_id, *this);
}

// Deprecated 2025-09-01.
std::unordered_map<CommonSolverOption, OptionValue>
SolverOptions::common_solver_options() const {
  std::unordered_map<CommonSolverOption, OptionValue> result;
  const auto iter1 = options.find(kCommonKey);
  if (iter1 != options.end()) {
    const string_unordered_map<OptionValue>& common = iter1->second;
    for (auto key : {CommonSolverOption::kPrintFileName,
                     CommonSolverOption::kPrintToConsole,
                     CommonSolverOption::kStandaloneReproductionFileName,
                     CommonSolverOption::kMaxThreads}) {
      const auto iter2 = common.find(fmt::to_string(key));
      if (iter2 != common.end()) {
        const OptionValue& value = iter2->second;
        result[key] = value;
      }
    }
  }
  return result;
}

// Deprecated 2025-09-01.
std::string SolverOptions::get_print_file_name() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::unordered_map<CommonSolverOption, OptionValue> common =
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

// Deprecated 2025-09-01.
bool SolverOptions::get_print_to_console() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::unordered_map<CommonSolverOption, OptionValue> common =
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

// Deprecated 2025-09-01.
std::string SolverOptions::get_standalone_reproduction_file_name() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::unordered_map<CommonSolverOption, OptionValue> common =
      common_solver_options();
#pragma GCC diagnostic pop
  // N.B. SetOption sanity checks the value; we don't need to re-check here.
  std::string result;
  auto iter = common.find(CommonSolverOption::kStandaloneReproductionFileName);
  if (iter != common.end()) {
    result = std::get<std::string>(iter->second);
  }
  return result;
}

// Deprecated 2025-09-01.
std::optional<int> SolverOptions::get_max_threads() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::unordered_map<CommonSolverOption, OptionValue> common =
      common_solver_options();
#pragma GCC diagnostic pop
  // N.B. SetOption sanity checks the value; we don't need to re-check here.
  auto iter = common.find(CommonSolverOption::kMaxThreads);
  if (iter != common.end()) {
    return std::get<int>(iter->second);
  }
  return std::nullopt;
}

// Deprecated 2025-09-01.
std::unordered_set<SolverId> SolverOptions::GetSolverIds() const {
  return solver_ids;
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
      result << fmt_debug_string(key);
      result << ":";
      result << internal::OptionValueToString(value);
      if (comma2) {
        result << ",";
      }
    }
    result << "}";
    if (comma1) {
      result << ",";
    }
  }
  result << "})";
  return result.str();
}

// Deprecated 2025-09-01.
std::ostream& operator<<(std::ostream& os, const SolverOptions& x) {
  os << x.to_string();
  return os;
}

// Deprecated 2025-09-01.
std::string to_string(const SolverOptions& x) {
  return x.to_string();
}

// Deprecated 2025-09-01.
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

}  // namespace solvers
}  // namespace drake
