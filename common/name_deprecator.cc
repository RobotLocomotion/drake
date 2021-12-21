#include "drake/common/name_deprecator.h"

#include <regex>
#include <set>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace internal {

namespace {
// Keep a process-wide collection of offered messages; return true if the
// message has been seen before.
bool TrackIssuedWarnings(const std::string& message) {
  static drake::never_destroyed<std::mutex> mutex;
  static drake::never_destroyed<std::set<std::string>> issued_messages;
  std::lock_guard<std::mutex> lock(mutex.access());

  const auto result = issued_messages.access().insert(message);
  const bool inserted = result.second;
  return !inserted;
}
}  // namespace

NameDeprecator::NameDeprecator() {}

bool NameDeprecator::HasScope() const {
  return !scope_name_.empty();
}

void NameDeprecator::DeclareScope(
    const std::string& name,
    std::function<bool(const std::string&)> predicate) {
  DRAKE_DEMAND(!HasScope());
  scope_name_ = name;
  scope_predicate_ = predicate;
}

void NameDeprecator::DeclareDeprecatedName(
    const std::string& removal_date,
    const std::string& deprecated_name,
    const std::string& correct_name) {
  // Do lots of error checking here so deprecations don't get casually
  // mis-typed, and so the translation/warn step can be simple and relatively
  // fast.
  DRAKE_DEMAND(HasScope());
  DRAKE_DEMAND(std::regex_match(removal_date,
                                std::regex(R"""(\d{4}-\d{2}-\d{2})""")));
  DRAKE_DEMAND(deprecations_.count(deprecated_name) == 0);
  // N.B. We scope-exclude the deprecated name *before* activating it, since
  // the client may well be using a predicate that also searches deprecated
  // names.
  DRAKE_DEMAND(!scope_predicate_(deprecated_name));
  DRAKE_DEMAND(scope_predicate_(correct_name));

  NameRecord record{removal_date, correct_name};
  deprecations_[deprecated_name] = record;
}

const std::string& NameDeprecator::MaybeTranslate(
    const std::string& name) const {
  const auto it = deprecations_.find(name);
  if (it == deprecations_.end()) {
    return name;
  }
  auto& record = it->second;
  if (!record.is_warning_issued) {
    const auto message = fmt::format(
        "Name '{}' is deprecated within scope '{}'"
        " and will be removed after {}. Use '{}' instead.",
        name, scope_name_, record.removal_date, record.correct_name);
    record.is_warning_issued = TrackIssuedWarnings(message);
    if (!record.is_warning_issued) {
      log()->warn(message);
    }
  }
  return record.correct_name;
}

}  // namespace internal
}  // namespace drake
