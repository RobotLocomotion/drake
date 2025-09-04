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
}

void SolverOptions::SetOption(CommonSolverOption key, OptionValue value) {
  switch (key) {
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

}  // namespace solvers
}  // namespace drake
