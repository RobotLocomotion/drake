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
#include "drake/common/fmt_eigen.h"
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

template <typename T> struct expect_supported_false : std::false_type {};

// Detect if T is a fmt_eigen_ref type. We'll key on the type's `matrix` member
// but exclude all other types that may have a matrix member.
template <typename T>
struct is_fmt_eigen_ref : std::false_type {};

template <typename T>
struct is_fmt_eigen_ref<fmt_eigen_ref<T>>: std::true_type {};

template <typename T>
std::string StringifyErrorDetailValue(const T& value) {
  // TODO(SeanCurtis-TRI) This version only supports floats. As we seek to pass
  // *other* types (strings, paths, eigen types, etc.), we'll need to extend
  // the supported types here and extend the declarations below.
  if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
    return fmt_floating_point(value);
  } else if constexpr (is_printable_string_v<T>) {
    return fmt::format("'{}'", value);
  } else if constexpr (is_fmt_eigen_ref<T>::value) {
    return fmt::to_string(value);
  } else {
    static_assert(
        expect_supported_false<T>::value,
        "DRAKE_THROW_UNLESS given an unsupported value expression type.");
  }
  DRAKE_UNREACHABLE();
}

template std::string StringifyErrorDetailValue<float>(const float&);
template std::string StringifyErrorDetailValue<double>(const double&);
template std::string StringifyErrorDetailValue<std::string>(const std::string&);
template std::string StringifyErrorDetailValue<std::string_view>(
    const std::string_view&);
template std::string StringifyErrorDetailValue<char const*>(char const* const&);
template std::string StringifyErrorDetailValue<internal::fmt_eigen_ref<double>>(
    const internal::fmt_eigen_ref<double>&);
// TODO(SeanCurtis-TRI): I need to enable this for AutoDiffXd. But there's a
// build dependency issue. AutoDiffXd is defined in `eigen_autodiff_types.h`
// which is defined in the `autodiff` build target. `autodiff` depends on the
// `essential` build target (which is what *this* is in). So, I end up with a
// circular dependency. I need to refactor the common denominator.
// template std::string StringifyErrorDetailValue<internal::fmt_eigen_ref<double>>(
//     const internal::fmt_eigen_ref<Eigen::AutoDiffScalar<Eigen::VectorXd>>&);

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
