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

// These are used for controlling the severity of
// deprecation messages during runtime.
// Note that the same literal string name for the environment variable is
// used for both C++ code and Python code, so keep the two in sync.
constexpr char kEnvDrakeDeprecationIsError[] = "DRAKE_DEPRECATION_IS_ERROR";
constexpr char kEnvDrakeIgnoreDeprecated[] = "DRAKE_DEPRECATION_IS_SILENT";

}  // namespace

WarnDeprecated::WarnDeprecated(std::string_view removal_date,
                               std::string_view message) {
  const char* const is_error_env = std::getenv(kEnvDrakeDeprecationIsError);
  const char* const ignore_env = std::getenv(kEnvDrakeIgnoreDeprecated);

  const bool is_error =
      (is_error_env != nullptr && std::string_view(is_error_env) == "1");
  const bool ignore =
      (ignore_env != nullptr && std::string_view(ignore_env) == "1");

  if (is_error && ignore) {
    std::string msg{
        "DRAKE_DEPRECATION_IS_ERROR and DRAKE_DEPRECATION_IS_SILENT cannot "
        "both be set to \"1\""};
    throw std::runtime_error(msg);
  }

  if (ignore) {
    return;
  }

  const bool missing_period = message.empty() || message.back() != '.';
  const std::string full_message = fmt::format(
      "DRAKE DEPRECATED: {}{} "
      "The deprecated code will be removed from Drake on or after {}.",
      message, missing_period ? "." : "", removal_date);

  if (is_error) {
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
