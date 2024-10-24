#include "drake/solvers/specific_options.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <fmt/ranges.h>

#include "drake/common/overloaded.h"

namespace drake {
namespace solvers {
namespace internal {

using OptionValue = SolverOptions::OptionValue;

SpecificOptions::SpecificOptions(const SolverId* id,
                                 const SolverOptions* all_options)
    : solver_name_{id->name()}, all_options_{*all_options} {
  auto iter = all_options_.options.find(solver_name_);
  if (iter != all_options_.options.end()) {
    specific_options_ = &(iter->second);
  }
}

SpecificOptions::~SpecificOptions() = default;

CommonSolverOptionValues SpecificOptions::ExtractCommon(
    const SolverOptions& solver_options) {
  CommonSolverOptionValues result;
  if (const auto outer = solver_options.options.find("Drake");
      outer != solver_options.options.end()) {
    const string_unordered_map<OptionValue>& common = outer->second;
    // N.B. SetOption sanity checks the value; we don't need to re-check here.
    if (auto iter = common.find(to_string(CommonSolverOption::kPrintFileName));
        iter != common.end()) {
      result.print_file_name = std::get<std::string>(iter->second);
    }
    if (auto iter = common.find(to_string(CommonSolverOption::kPrintToConsole));
        iter != common.end()) {
      result.print_to_console = std::get<int>(iter->second);
    }
    if (auto iter = common.find(
            to_string(CommonSolverOption::kStandaloneReproductionFileName));
        iter != common.end()) {
      result.standalone_reproduction_file_name =
          std::get<std::string>(iter->second);
    }
    if (auto iter = common.find(to_string(CommonSolverOption::kMaxThreads));
        iter != common.end()) {
      result.max_threads = std::get<int>(iter->second);
    }
  }
  return result;
}

void SpecificOptions::Respell(
    const std::function<void(const CommonSolverOptionValues&,
                             string_unordered_map<OptionValue>*)>& respell) {
  DRAKE_DEMAND(respell != nullptr);
  DRAKE_DEMAND(respelled_.empty());
  respell(ExtractCommon(all_options_), &respelled_);
}

template <typename Result>
std::optional<Result> SpecificOptions::Pop(std::string_view key) {
  if (popped_.contains(key)) {
    return std::nullopt;
  }
  if (specific_options_ != nullptr) {
    if (auto iter = specific_options_->find(key);
        iter != specific_options_->end()) {
      const auto& boxed_value = iter->second;
      if (std::holds_alternative<Result>(boxed_value)) {
        popped_.emplace(key);
        return std::get<Result>(boxed_value);
      }
      return {};
    }
  }
  if (auto iter = respelled_.find(key); iter != respelled_.end()) {
    const OptionValue& value = iter->second;
    if (std::holds_alternative<Result>(value)) {
      popped_.emplace(key);
      return std::get<Result>(value);
    }
    throw std::logic_error(fmt::format(
        "{}: internal error: option {} was respelled to the wrong type",
        solver_name_, key));
  }
  return {};
}

template std::optional<double> SpecificOptions::Pop(std::string_view);
template std::optional<int> SpecificOptions::Pop(std::string_view);
template std::optional<std::string> SpecificOptions::Pop(std::string_view);

void SpecificOptions::CopyToCallbacks(
    const std::function<void(const std::string& key, double)>& set_double,
    const std::function<void(const std::string& key, int)>& set_int,
    const std::function<void(const std::string& key, const std::string&)>&
        set_string) const {
  // Wrap the solver's set_{type} callbacks with error-reporting sugar.
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
        "{}: string options are not supported; the option {}='{}' is invalid",
        solver_name_, key, value));
  };

  // Handle solver-specific options.
  if (specific_options_ != nullptr) {
    for (const auto& [key, boxed_value] : *specific_options_) {
      if (popped_.contains(key)) {
        continue;
      }
      std::visit(overloaded{[&key, &on_double](double value) {
                              on_double(key, value);
                            },
                            [&key, &on_int](int value) {
                              on_int(key, value);
                            },
                            [&key, &on_string](const std::string& value) {
                              on_string(key, value);
                            }},
                 boxed_value);
    }
  }

  // Handle any respelled options, being careful not to set anything that has
  // already been set.
  for (const auto& [respelled_key, boxed_value] : respelled_) {
    // Pedantically, lambdas cannot capture a structured binding so we need to
    // make a local variable that we can capture.
    const auto& key = respelled_key;
    if (popped_.contains(key)) {
      continue;
    }
    if (specific_options_ != nullptr && specific_options_->contains(key)) {
      continue;
    }
    std::visit(overloaded{[&key, &on_double](double value) {
                            on_double(key, value);
                          },
                          [&key, &on_int](int value) {
                            on_int(key, value);
                          },
                          [&key, &on_string](const std::string& value) {
                            on_string(key, value);
                          }},
               boxed_value);
  }
}

void SpecificOptions::InitializePending() {
  pending_keys_.clear();
  if (specific_options_ == nullptr) {
    return;
  }
  pending_keys_.reserve(specific_options_->size());
  for (const auto& [key, _] : *specific_options_) {
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
    for (const auto& name : pending_keys_) {
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
  if (specific_options_ != nullptr) {
    if (auto iter = specific_options_->find(name);
        iter != specific_options_->end()) {
      pending_keys_.erase(iter->first);
      const OptionValue& value = iter->second;
      return &value;
    }
  }
  if (auto iter = respelled_.find(name); iter != respelled_.end()) {
    pending_keys_.erase(iter->first);
    const OptionValue& value = iter->second;
    return &value;
  }
  return nullptr;
}

template <typename T>
void SpecificOptions::CopyFloatingPointOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto* boxed_value = PrepareToCopy(name)) {
    if (std::holds_alternative<double>(*boxed_value)) {
      *output = std::get<double>(*boxed_value);
      return;
    }
    if (std::holds_alternative<int>(*boxed_value)) {
      *output = std::get<int>(*boxed_value);
      return;
    }
    throw std::logic_error(
        fmt::format("{}: Expected a floating-point value for option {}={}",
                    solver_name_, name, internal::Repr(*boxed_value)));
  }
}
template void SpecificOptions::CopyFloatingPointOption(const char*, double*);
template void SpecificOptions::CopyFloatingPointOption(const char*, float*);

template <typename T>
void SpecificOptions::CopyIntegralOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto* boxed_value = PrepareToCopy(name)) {
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
    throw std::logic_error(
        fmt::format("{}: Expected an integer value for option {}={}",
                    solver_name_, name, internal::Repr(*boxed_value)));
  }
}
template void SpecificOptions::CopyIntegralOption(const char*, int*);
template void SpecificOptions::CopyIntegralOption(const char*, bool*);
template void SpecificOptions::CopyIntegralOption(const char*, uint32_t*);

void SpecificOptions::CopyStringOption(const char* name, std::string* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto* boxed_value = PrepareToCopy(name)) {
    if (std::holds_alternative<std::string>(*boxed_value)) {
      *output = std::get<std::string>(*boxed_value);
      return;
    }
    throw std::logic_error(
        fmt::format("{}: Expected a string value for option {}={}",
                    solver_name_, name, internal::Repr(*boxed_value)));
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
