// This file contains the implementation of both drake_assert and drake_throw.
// clang-format off
#include "drake/common/drake_assert.h"
#include "drake/common/drake_assert_toggle.h"
#include "drake/common/drake_throw.h"
// clang-format on

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/never_destroyed.h"

namespace drake {
namespace {

// This is what DRAKE_THROW_UNLESS throws and what DRAKE_ASSERT and
// DRAKE_DEMAND throw when our assertions are configured to throw.
class assertion_error : public std::runtime_error {
 public:
  explicit assertion_error(const std::string& what_arg)
      : std::runtime_error(what_arg) {}
};

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
}  // namespace

// Declared in drake_assert.h.
void detail::Abort(const char* condition, const char* func, const char* file,
                   int line) {
  std::cerr << "abort: ";
  PrintFailureDetailTo(std::cerr, condition, func, file, line);
  std::cerr << std::endl;
  std::abort();
}

// Declared in drake_throw.h.
void detail::Throw(const char* condition, const char* func, const char* file,
                   int line) {
  std::ostringstream what;
  PrintFailureDetailTo(what, condition, func, file, line);
  throw assertion_error(what.str());
}

// Declared in drake_assert.h.
void detail::AssertionFailed(const char* condition, const char* func,
                             const char* file, int line) {
  if (AssertionConfig::singleton().assertion_failures_are_exceptions) {
    detail::Throw(condition, func, file, line);
  } else {
    detail::Abort(condition, func, file, line);
  }
}

// Declared in drake_assert_toggle.h.
void assert::set_assertion_failure_to_throw_exception() {
  AssertionConfig::singleton().assertion_failures_are_exceptions = true;
}

}  // namespace drake
