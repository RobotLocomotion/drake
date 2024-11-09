#include "drake/common/fmt.h"

namespace drake {

// We can simplify this after we drop support for Ubuntu 22.04 Jammy.
std::string fmt_debug_string(std::string_view x) {
#if FMT_VERSION >= 90000
  return fmt::format("{:?s}", x);
#else
  std::string result;
  result.reserve(x.size() + 2);
  result.push_back('"');
  for (const char ch : x) {
    // Check for characters with a custom escape sequence.
    if (ch == '\n') {
      result.push_back('\\');
      result.push_back('n');
      continue;
    }
    if (ch == '\r') {
      result.push_back('\\');
      result.push_back('r');
      continue;
    }
    if (ch == '\t') {
      result.push_back('\\');
      result.push_back('t');
      continue;
    }
    // Check for characters that require a leading backslash.
    if (ch == '"' || ch == '\'' || ch == '\\') {
      result.push_back('\\');
      result.push_back(ch);
      continue;
    }
    // Check for any other non-printable characters.
    if (ch < 0x20 || ch >= 0x7F) {
      result.append(fmt::format("\\x{:02x}", static_cast<int>(ch)));
      continue;
    }
    // Normal character.
    result.push_back(ch);
  }
  result.push_back('"');
  return result;
#endif
}

}  // namespace drake
