#include "drake/common/network_policy.h"

#include <algorithm>
#include <cstdlib>

#include "drake/common/drake_throw.h"

namespace drake {
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

  // Unset or empty means all is allowed.
  const char* const env_cstr = std::getenv("DRAKE_ALLOW_NETWORK");
  if (env_cstr == nullptr) {
    return true;
  }
  if (*env_cstr == '\0') {
    return true;
  }

  // Iterate over the colon-delimited tokens.
  // TODO(jwnimmer-tri) As of C++20, use std::ranges::lazy_split_view.
  bool match = false;
  std::string_view worklist{env_cstr};
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
      return false;
    }
    if (token == component) {
      match = true;
    }
  }

  return match;
}

}  // namespace drake
