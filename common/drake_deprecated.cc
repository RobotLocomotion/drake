#include "drake/common/drake_deprecated.h"

#include <cstdlib>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace internal {
namespace {

// TODO(jwnimmer-tri) Teach pybind11 to map this to Python's DeprecationWarning.
class DeprecationWarning final : public std::runtime_error {
 public:
  using runtime_error::runtime_error;
};

// Note that the same literal string name for the environment variable is used
// for both C++ code and Python code, so keep the two in sync.
constexpr char kEnvSeverity[] = "DRAKE_DEPRECATION_RUNTIME_SEVERITY";

}  // namespace

WarnDeprecated::WarnDeprecated(std::string_view removal_date,
                               std::string_view message) {
  const char* const severity_env = std::getenv(kEnvSeverity);
  std::string_view severity = (severity_env != nullptr) ? severity_env : "";

  if (!severity.empty() && severity != "error" && severity != "ignore") {
    static const logging::Warn log_once(
        "{} is set to an unrecognized value {:?}. Deprecation messages will"
        " be emitted as warnings.",
        kEnvSeverity, severity);
  }

  if (severity == "ignore") {
    return;
  }

  const bool missing_period = message.empty() || message.back() != '.';
  const std::string full_message = fmt::format(
      "DRAKE DEPRECATED: {}{} "
      "The deprecated code will be removed from Drake on or after {}.",
      message, missing_period ? "." : "", removal_date);

  if (severity == "error") {
    throw DeprecationWarning(full_message);
  } else {
    log()->warn(full_message);
  }

  // If a Drake developer made a mistake, we need to fail-fast but it's better
  // to have at least printed the warning first.
  DRAKE_THROW_UNLESS(removal_date.size() == 10);
  DRAKE_THROW_UNLESS(!message.empty());
}

}  // namespace internal
}  // namespace drake
