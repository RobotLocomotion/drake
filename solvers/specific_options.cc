#include "drake/solvers/specific_options.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include <fmt/ranges.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/overloaded.h"

namespace drake {
namespace solvers {
namespace internal {

using OptionValue = SolverOptions::OptionValue;

namespace {

/* Utility function for our constructor. Extracts one common (Drake) option
from `common`. Does nothing if not set, or if set to the wrong type. */
template <typename T, typename U>
void GetOneCommonOption(const string_unordered_map<OptionValue>& common,
                        CommonSolverOption key, U* out) {
  if (auto iter = common.find(to_string(key)); iter != common.end()) {
    const OptionValue& value = iter->second;
    if (std::holds_alternative<T>(value)) {
      *out = std::get<T>(value);
    }
  }
}

/* Utility function for our constructor. Extracts and returns common (Drake)
options from `all_options`. */
CommonSolverOptionValues GetCommonOptions(const SolverOptions* all_options) {
  // Check the pointer on behalf of our constructor.
  DRAKE_THROW_UNLESS(all_options != nullptr);
  CommonSolverOptionValues result;
  if (auto iter = all_options->options.find("Drake");
      iter != all_options->options.end()) {
    const string_unordered_map<OptionValue>& common = iter->second;
    GetOneCommonOption<std::string>(common, CommonSolverOption::kPrintFileName,
                                    &result.print_file_name);
    GetOneCommonOption<int>(common, CommonSolverOption::kPrintToConsole,
                            &result.print_to_console);
    GetOneCommonOption<std::string>(
        common, CommonSolverOption::kStandaloneReproductionFileName,
        &result.standalone_reproduction_file_name);
    GetOneCommonOption<int>(common, CommonSolverOption::kMaxThreads,
                            &result.max_threads);
  }
  return result;
}

/* Utility function for our constructor. Returns a reference to the solver
specific options for given solver `id` contained in `all_options`, or an empty
map in case there are none. */
const string_unordered_map<SolverOptions::OptionValue>& GetDirectOptions(
    const SolverId* id, const SolverOptions* all_options) {
  // Check the pointer on behalf of our constructor.
  DRAKE_THROW_UNLESS(id != nullptr);
  DRAKE_DEMAND(all_options != nullptr);  // Already checked by GetCommonOptions.
  auto iter = all_options->options.find(id->name());
  if (iter != all_options->options.end()) {
    return iter->second;
  }
  static const never_destroyed<string_unordered_map<SolverOptions::OptionValue>>
      empty;
  return empty.access();
}

}  // namespace

SpecificOptions::SpecificOptions(const SolverId* id,
                                 const SolverOptions* all_options)
    : common_options_{GetCommonOptions(all_options)},
      direct_options_{GetDirectOptions(id, all_options)},
      solver_name_{id->name()} {}

SpecificOptions::~SpecificOptions() = default;

void SpecificOptions::Respell(
    const std::function<void(const CommonSolverOptionValues&,
                             string_unordered_map<OptionValue>*)>& respell) {
  DRAKE_DEMAND(respell != nullptr);
  DRAKE_DEMAND(respelled_.empty());
  respell(common_options_, &respelled_);
}

template <typename Result>
std::optional<Result> SpecificOptions::Pop(std::string_view key) {
  if (auto iter = direct_options_.find(key); iter != direct_options_.end()) {
    if (popped_.contains(key)) {
      return std::nullopt;
    }
    const OptionValue& boxed_value = iter->second;
    if (std::holds_alternative<Result>(boxed_value)) {
      popped_.emplace(key);
      return std::get<Result>(boxed_value);
    }
    return std::nullopt;
  }
  if (auto iter = respelled_.find(key); iter != respelled_.end()) {
    OptionValue& boxed_value = iter->second;
    if (std::holds_alternative<Result>(boxed_value)) {
      std::optional<Result> result = std::move(std::get<Result>(boxed_value));
      respelled_.erase(iter);
      return result;
    }
    throw std::logic_error(fmt::format(
        "{}: internal error: option {} was respelled to the wrong type",
        solver_name_, key));
  }
  return std::nullopt;
}

template std::optional<double> SpecificOptions::Pop(std::string_view);
template std::optional<int> SpecificOptions::Pop(std::string_view);
template std::optional<std::string> SpecificOptions::Pop(std::string_view);

void SpecificOptions::CopyToCallbacks(
    const std::function<void(const std::string& key, double)>& set_double,
    const std::function<void(const std::string& key, int)>& set_int,
    const std::function<void(const std::string& key, const std::string&)>&
        set_string) const {
  // Wrap the solver's set_{type} callbacks with error-reporting sugar, and
  // logic to promote integers to doubles.
  auto on_double = [this, &set_double](const std::string& key, double value) {
    if (set_double != nullptr) {
      set_double(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: floating-point options are not supported; the option {}={} is "
        "invalid",
        solver_name_, key, value));
  };
  auto on_int = [this, &set_int, &set_double](const std::string& key,
                                              int value) {
    if (set_int != nullptr) {
      set_int(key, value);
      return;
    }
    if (set_double != nullptr) {
      set_double(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: integer and floating-point options are not supported; the option "
        "{}={} is invalid",
        solver_name_, key, value));
  };
  auto on_string = [this, &set_string](const std::string& key,
                                       const std::string& value) {
    if (set_string != nullptr) {
      set_string(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: string options are not supported; the option {}={} is invalid",
        solver_name_, key, fmt_debug_string(value)));
  };

  // Handle solver-specific options.
  for (const auto& [key, boxed_value] : direct_options_) {
    if (popped_.contains(key)) {
      continue;
    }
    const std::string& key_str = key;  // Allow lambda capture.
    std::visit(overloaded{[&key_str, &on_double](double value) {
                            on_double(key_str, value);
                          },
                          [&key_str, &on_int](int value) {
                            on_int(key_str, value);
                          },
                          [&key_str, &on_string](const std::string& value) {
                            on_string(key_str, value);
                          }},
               boxed_value);
  }

  // Handle any respelled options, being careful not to set anything that has
  // already been set.
  for (const auto& [key, boxed_value] : respelled_) {
    if (direct_options_.contains(key) && !popped_.contains(key)) {
      continue;
    }
    const std::string& key_str = key;  // Allow lambda capture.
    std::visit(overloaded{[&key_str, &on_double](double value) {
                            on_double(key_str, value);
                          },
                          [&key_str, &on_int](int value) {
                            on_int(key_str, value);
                          },
                          [&key_str, &on_string](const std::string& value) {
                            on_string(key_str, value);
                          }},
               boxed_value);
  }
}

void SpecificOptions::InitializePending() {
  pending_keys_.clear();
  pending_keys_.reserve(direct_options_.size());
  for (const auto& [key, _] : direct_options_) {
    if (popped_.contains(key)) {
      continue;
    }
    pending_keys_.insert(key);
  }
}

void SpecificOptions::CheckNoPending() const {
  // Identify any unsupported names (i.e., leftovers in `pending_`).
  if (!pending_keys_.empty()) {
    std::vector<std::string_view> unknown_names;
    for (const std::string_view& name : pending_keys_) {
      unknown_names.push_back(name);
    }
    std::sort(unknown_names.begin(), unknown_names.end());
    throw std::logic_error(fmt::format(
        "{}: the following solver option names were not recognized: {}",
        solver_name_, fmt::join(unknown_names, ", ")));
  }
}

const OptionValue* SpecificOptions::PrepareToCopy(const char* name) {
  DRAKE_DEMAND(name != nullptr);
  if (popped_.contains(name)) {
    return nullptr;
  }
  if (auto iter = direct_options_.find(name); iter != direct_options_.end()) {
    pending_keys_.erase(iter->first);
    const OptionValue& boxed_value = iter->second;
    return &boxed_value;
  }
  if (auto iter = respelled_.find(name); iter != respelled_.end()) {
    pending_keys_.erase(iter->first);
    const OptionValue& boxed_value = iter->second;
    return &boxed_value;
  }
  return nullptr;
}

template <typename T>
void SpecificOptions::CopyFloatingPointOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (const OptionValue* boxed_value = PrepareToCopy(name)) {
    if (std::holds_alternative<double>(*boxed_value)) {
      *output = std::get<double>(*boxed_value);
      return;
    }
    if (std::holds_alternative<int>(*boxed_value)) {
      *output = std::get<int>(*boxed_value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: Expected a floating-point value for option {}={}", solver_name_,
        name, internal::OptionValueToString(*boxed_value)));
  }
}
template void SpecificOptions::CopyFloatingPointOption(const char*, double*);
template void SpecificOptions::CopyFloatingPointOption(const char*, float*);

template <typename T>
void SpecificOptions::CopyIntegralOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (const OptionValue* boxed_value = PrepareToCopy(name)) {
    if (std::holds_alternative<int>(*boxed_value)) {
      const int value = std::get<int>(*boxed_value);
      if constexpr (std::is_same_v<T, int>) {
        *output = value;
      } else if constexpr (std::is_same_v<T, bool>) {
        if (!(value == 0 || value == 1)) {
          throw std::logic_error(fmt::format(
              "{}: Expected a boolean value (0 or 1) for int option {}={}",
              solver_name_, name, value));
        }
        *output = value;
      } else {
        static_assert(std::is_same_v<T, uint32_t>);
        if (value < 0) {
          throw std::logic_error(fmt::format(
              "{}: Expected a non-negative value for unsigned int option {}={}",
              solver_name_, name, value));
        }
        // In practice it's unlikely that sizeof(int) > 4, but better safe than
        // sorry. This also serves as a reminder to sanity check other casts if
        // we add more template instantiations than just uint32_t.
        if (static_cast<int64_t>(value) >
            static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
          throw std::logic_error(
              fmt::format("{}: Too-large value for uint32 option {}={}",
                          solver_name_, name, value));
        }
        *output = value;
      }
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: Expected an integer value for option {}={}", solver_name_, name,
        internal::OptionValueToString(*boxed_value)));
  }
}
template void SpecificOptions::CopyIntegralOption(const char*, int*);
template void SpecificOptions::CopyIntegralOption(const char*, bool*);
template void SpecificOptions::CopyIntegralOption(const char*, uint32_t*);

void SpecificOptions::CopyStringOption(const char* name, std::string* output) {
  DRAKE_DEMAND(output != nullptr);
  if (const OptionValue* boxed_value = PrepareToCopy(name)) {
    if (std::holds_alternative<std::string>(*boxed_value)) {
      *output = std::get<std::string>(*boxed_value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: Expected a string value for option {}={}", solver_name_, name,
        internal::OptionValueToString(*boxed_value)));
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
