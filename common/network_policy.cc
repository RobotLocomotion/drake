#include "drake/common/network_policy.h"

#include <algorithm>
#include <cstdlib>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace internal {
namespace {

bool IsAsciiLowercaseAlphaNumeric(std::string_view word) {
  return std::all_of(word.begin(), word.end(), [](char ch) {
    return ('a' <= ch && ch <= 'z') || ('0' <= ch && ch <= '9') || (ch == '_');
  });
}

}  // namespace

bool IsNetworkingAllowed(std::string_view component) {
  DRAKE_THROW_UNLESS(component.length() > 0);
  DRAKE_THROW_UNLESS(component != "none");
  DRAKE_THROW_UNLESS(IsAsciiLowercaseAlphaNumeric(component));

  // Unset or empty means allow-all.
  const char* const env_cstr = std::getenv("DRAKE_ALLOW_NETWORK");
  if (env_cstr == nullptr) {
    return true;
  }
  if (*env_cstr == '\0') {
    return true;
  }
  const std::string_view env_view{env_cstr};

  // Set to "none" means deny-all.
  if (env_view == "none") {
    return false;
  }

  // Iterate over the colon-delimited tokens.
  // N.B. We purposefully do not warn for unknown tokens because they may evolve
  // over time and we don't want to force users to churn their policy variables
  // to be congruent with their Drake version pin.
  // TODO(jwnimmer-tri) As of C++20, use std::ranges::lazy_split_view.
  bool match = false;
  std::string_view worklist = env_view;
  while (!worklist.empty()) {
    std::string_view token;
    auto delim = worklist.find(':');
    if (delim == std::string_view::npos) {
      token = worklist;
      worklist = {};
    } else {
      token = worklist.substr(0, delim);
      worklist.remove_prefix(delim + 1);
    }
    if (token == "none") {
      static const logging::Warn log_once(
          "Setting DRAKE_ALLOW_NETWORK={} combines 'none' with non-none "
          "values; this is probably not what you wanted! The effect is "
          "the same as just saying 'none' on its own; nothing is allowed!",
          env_view);
      return false;
    }
    if (token == component) {
      match = true;
    }
  }

  return match;
}

}  // namespace internal
}  // namespace drake
