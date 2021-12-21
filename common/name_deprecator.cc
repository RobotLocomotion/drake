#include "drake/common/name_deprecator.h"

#include <regex>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace internal {

NameDeprecator::NameDeprecator() {}

bool NameDeprecator::HasScope() const {
  return !scope_name_.empty();
}

void NameDeprecator::DeclareScope(
    const std::string& name,
    std::function<bool(const std::string&)> predicate) {
  DRAKE_ASSERT(!HasScope());
  scope_name_ = name;
  scope_predicate_ = predicate;
}

void NameDeprecator::DeclareDeprecatedName(
    const std::string& removal_date,
    const std::string& deprecated_name,
    const std::string& correct_name) {
  // Do lots of (debug-only) error checking here so deprecations don't get
  // casually mis-typed, and so the translation/warn step can be simple and
  // relatively fast.
  DRAKE_ASSERT(HasScope());
  DRAKE_ASSERT(std::regex_match(removal_date,
                                std::regex(R"""(\d{4}-\d{2}-\d{2})""")));
  DRAKE_ASSERT(!scope_predicate_(deprecated_name));
  DRAKE_ASSERT(scope_predicate_(correct_name));
  DRAKE_ASSERT(deprecations_.count(deprecated_name) == 0);
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
    record.is_warning_issued = true;
    log()->warn("Name '{}' is deprecated within scope '{}'"
                " and will be removed after {}. Use '{}' instead.",
                name, scope_name_, record.removal_date, record.correct_name);
  }
  return record.correct_name;
}

}  // namespace internal
}  // namespace drake
