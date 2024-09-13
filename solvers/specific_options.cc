#include "drake/solvers/specific_options.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "drake/common/overloaded.h"

namespace drake {
namespace solvers {
namespace internal {

using OptionValue = SolverOptions::OptionValue;

SpecificOptions::SpecificOptions(const SolverId* id,
                                 const SolverOptions* all_options)
    : id_{id}, all_options_{all_options} {
  DRAKE_DEMAND(id != nullptr);
  DRAKE_DEMAND(all_options != nullptr);
}

SpecificOptions::~SpecificOptions() = default;

void SpecificOptions::Respell(
    const std::function<string_unordered_map<OptionValue>(
        const CommonSolverOptionValues&)>& respell) {
  DRAKE_DEMAND(respell != nullptr);
  respelled_ = respell(CommonSolverOptionValues{
      .print_file_name = all_options_->get_print_file_name(),
      .print_to_console = all_options_->get_print_to_console(),
      .standalone_reproduction_file_name =
          all_options_->get_standalone_reproduction_file_name(),
      .max_threads = all_options_->get_max_threads(),
  });
}

template <typename Result>
std::optional<Result> SpecificOptions::Pop(std::string_view key) {
  if (popped_.contains(key)) {
    return std::nullopt;
  }
  popped_.emplace(key);
  const auto& typed_options = all_options_->template GetOptions<Result>(*id_);
  if (auto iter = typed_options.find(std::string{key});
      iter != typed_options.end()) {
    return iter->second;
  }
  if (auto iter = respelled_.find(key); iter != respelled_.end()) {
    const OptionValue& value = iter->second;
    if (std::holds_alternative<Result>(value)) {
      return std::get<Result>(value);
    }
    throw std::logic_error(fmt::format(
        "{}: internal error: option {} was respelled to the wrong type",
        id_->name(), key));
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
  // Bail out early when we have no options at all for this solver.
  const std::unordered_map<std::string, double>& options_double =
      all_options_->GetOptionsDouble(*id_);
  const std::unordered_map<std::string, int>& options_int =
      all_options_->GetOptionsInt(*id_);
  const std::unordered_map<std::string, std::string>& options_str =
      all_options_->GetOptionsStr(*id_);
  if (options_double.empty() && options_int.empty() && options_str.empty() &&
      respelled_.empty()) {
    return;
  }

  // Wrap the solver's set_{type} callbacks with error-reporting sugar, and
  // logic to promote integers to doubles.
  auto on_double = [this, &set_double](const std::string& key, double value) {
    if (popped_.contains(key)) {
      return;
    }
    if (set_double != nullptr) {
      set_double(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: floating-point options are not supported; the option {}={} is "
        "invalid",
        id_->name(), key, value));
  };
  auto on_int = [this, &set_int, &set_double](const std::string& key,
                                              int value) {
    if (popped_.contains(key)) {
      return;
    }
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
        id_->name(), key, value));
  };
  auto on_string = [this, &set_string](const std::string& key,
                                       const std::string& value) {
    if (popped_.contains(key)) {
      return;
    }
    if (set_string != nullptr) {
      set_string(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: string options are not supported; the option {}='{}' is invalid",
        id_->name(), key, value));
  };

  // Handle solver-specific options.
  for (const auto& [key, value] : options_double) {
    on_double(key, value);
  }
  for (const auto& [key, value] : options_int) {
    on_int(key, value);
  }
  for (const auto& [key, value] : options_str) {
    on_string(key, value);
  }

  // Handle any respelled options, being careful not to set anything that has
  // already been set.
  for (const auto& [respelled_key, boxed_value] : respelled_) {
    // Pedantially, lambdas cannot capture a structured binding so we need to
    // make a local variable that we can capture.
    const auto& key = respelled_key;
    std::visit(
        overloaded{[&key, &on_double, &options_double](double value) {
                     if (!options_double.contains(key)) {
                       on_double(key, value);
                     }
                   },
                   [&key, &on_int, &options_int](int value) {
                     if (!options_int.contains(key)) {
                       on_int(key, value);
                     }
                   },
                   [&key, &on_string, &options_str](const std::string& value) {
                     if (!options_str.contains(key)) {
                       on_string(key, value);
                     }
                   }},
        boxed_value);
  }
}

void SpecificOptions::InitializePending() {
  pending_keys_.clear();
  for (const auto& [key, _] : all_options_->GetOptionsDouble(*id_)) {
    pending_keys_.insert(key);
  }
  for (const auto& [key, _] : all_options_->GetOptionsInt(*id_)) {
    pending_keys_.insert(key);
  }
  for (const auto& [key, _] : all_options_->GetOptionsStr(*id_)) {
    pending_keys_.insert(key);
  }
  for (const auto& [key, _] : respelled_) {
    pending_keys_.insert(key);
  }
  for (const auto& key : popped_) {
    pending_keys_.erase(key);
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
        id_->name(), fmt::join(unknown_names, ", ")));
  }
}

std::optional<OptionValue> SpecificOptions::PrepareToCopy(const char* name) {
  DRAKE_DEMAND(name != nullptr);
  const std::unordered_map<std::string, double>& options_double =
      all_options_->GetOptionsDouble(*id_);
  if (auto iter = options_double.find(std::string{name});
      iter != options_double.end()) {
    pending_keys_.erase(iter->first);
    return iter->second;
  }
  const std::unordered_map<std::string, int>& options_int =
      all_options_->GetOptionsInt(*id_);
  if (auto iter = options_int.find(std::string{name});
      iter != options_int.end()) {
    pending_keys_.erase(iter->first);
    return iter->second;
  }
  const std::unordered_map<std::string, std::string>& options_str =
      all_options_->GetOptionsStr(*id_);
  if (auto iter = options_str.find(std::string{name});
      iter != options_str.end()) {
    pending_keys_.erase(iter->first);
    return iter->second;
  }
  if (auto iter = respelled_.find(name); iter != respelled_.end()) {
    pending_keys_.erase(iter->first);
    return iter->second;
  }
  return {};
}

template <typename T>
void SpecificOptions::CopyFloatingPointOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto value = PrepareToCopy(name)) {
    if (std::holds_alternative<double>(*value)) {
      *output = std::get<double>(*value);
      return;
    }
    if (std::holds_alternative<int>(*value)) {
      *output = std::get<int>(*value);
      return;
    }
    throw std::logic_error(
        fmt::format("{}: Expected a floating-point value for option {}",
                    id_->name(), name));
  }
}
template void SpecificOptions::CopyFloatingPointOption(const char*, double*);
template void SpecificOptions::CopyFloatingPointOption(const char*, float*);

template <typename T>
void SpecificOptions::CopyIntegralOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto value = PrepareToCopy(name)) {
    if (std::holds_alternative<int>(*value)) {
      // XXX bool validation / unsigned / etc.
      *output = std::get<int>(*value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: Expected an integer value for option {}", id_->name(), name));
  }
}
template void SpecificOptions::CopyIntegralOption(const char*, int*);
template void SpecificOptions::CopyIntegralOption(const char*, bool*);
template void SpecificOptions::CopyIntegralOption(const char*, uint32_t*);

void SpecificOptions::CopyStringOption(const char* name, std::string* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto value = PrepareToCopy(name)) {
    if (std::holds_alternative<std::string>(*value)) {
      *output = std::get<std::string>(*value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: Expected a string value for option {}", id_->name(), name));
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
