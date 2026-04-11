#include "drake/common/drake_assert.h"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "drake/common/drake_assertion_error.h"
#include "drake/common/fmt.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {
namespace {

// Singleton to manage assertion configuration.
struct AssertionConfig {
  static AssertionConfig& singleton() {
    static never_destroyed<AssertionConfig> global;
    return global.access();
  }

  std::atomic<bool> assertion_failures_are_exceptions;
};

// Stream into @p out the given failure details; only @p condition may be null.
void PrintFailureDetailTo(std::ostream& out, const char* condition,
                          const char* func, const char* file, int line) {
  out << "Failure at " << file << ":" << line << " in " << func << "()";
  if (condition) {
    out << ": condition '" << condition << "' failed.";
  } else {
    out << ".";
  }
}

// When calling DRAKE_THROW_UNLESS(condition, value1, value2, ...) some values
// need to be *prepared* for fmt::format by wrapping them as fmt_eigen(value1).
// We don't want those wrappers in the displayed string.
std::string_view StripFormatCruft(const std::string& key) {
  if (key.starts_with("fmt_eigen(")) {
    return std::string_view(key.begin() + 10, key.end() - 1);
  } else if (key.starts_with("drake::fmt_eigen(")) {
    return std::string_view(key.begin() + 17, key.end() - 1);
  }
  return std::string_view(key.begin(), key.end());
}

}  // namespace

// Declared in drake_assert.h.
void Abort(const char* condition, const char* func, const char* file,
           int line) {
  std::cerr << "abort: ";
  PrintFailureDetailTo(std::cerr, condition, func, file, line);
  std::cerr << std::endl;
  std::abort();
}

// Declared in drake_throw.h.
void Throw(const char* condition, const char* func, const char* file, int line,
           const ThrowValuesBuf& buffer) {
  std::ostringstream what;
  PrintFailureDetailTo(what, condition, func, file, line);
  if (buffer.values[0].first != nullptr) {
    std::vector<std::string> pairs;
    pairs.reserve(buffer.values.size());
    for (const auto& [key, value_str] : buffer.values) {
      if (key == nullptr) break;
      pairs.push_back(fmt::format("{} = {}", StripFormatCruft(key), value_str));
    }
    what << fmt::format(" {}.", fmt::join(pairs, ", "));
  }
  throw assertion_error(what.str().c_str());
}

// Declared in drake_assert.h.
void AssertionFailed(const char* condition, const char* func, const char* file,
                     int line) {
  if (AssertionConfig::singleton().assertion_failures_are_exceptions) {
    Throw(condition, func, file, line);
  } else {
    Abort(condition, func, file, line);
  }
}

template <typename T>
std::string StringifyErrorDetailValue(const T& value)
  requires(std::is_same_v<T, float> || std::is_same_v<T, double> ||
           std::is_same_v<T, int> || std::is_same_v<T, std::size_t> ||
           std::is_same_v<T, std::string> ||
           std::is_same_v<T, std::string_view> ||
           std::is_same_v<T, const char*>)
{
  if constexpr (std::is_floating_point_v<T>) {
    return fmt_floating_point(value);
  } else if constexpr (std::is_integral_v<T>) {
    return fmt::to_string(value);
  } else {
    return fmt_debug_string(value);
  }
}

template std::string StringifyErrorDetailValue<float>(const float&);
template std::string StringifyErrorDetailValue<double>(const double&);
template std::string StringifyErrorDetailValue<std::string>(const std::string&);
template std::string StringifyErrorDetailValue<std::string_view>(
    const std::string_view&);
template std::string StringifyErrorDetailValue<char const*>(char const* const&);
template std::string StringifyErrorDetailValue<int>(const int&);
template std::string StringifyErrorDetailValue<std::size_t>(const std::size_t&);

}  // namespace internal
}  // namespace drake

// Configures the DRAKE_ASSERT and DRAKE_DEMAND assertion failure handling
// behavior.
//
// By default, assertion failures will result in an ::abort().  If this method
// has ever been called, failures will result in a thrown exception instead.
//
// Assertion configuration has process-wide scope.  Changes here will affect
// all assertions within the current process.
//
// This method is intended ONLY for use by pydrake bindings, and thus is
// declared here in the cc file (not in any header file), to discourage
// anyone from using it.
extern "C" void drake_set_assertion_failure_to_throw_exception();

// Define the function (separate from declaration) to avoid compiler warnings.
void drake_set_assertion_failure_to_throw_exception() {
  drake::internal::AssertionConfig::singleton()
      .assertion_failures_are_exceptions = true;
}
