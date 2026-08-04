#pragma once

#include <deque>
#include <string>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace test {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

/// A base class for test fixtures that involve DiagnosticPolicy output. It
/// sets error and warning actions on construction to collect all
/// diagnostics. On destruction, it expects its internal warning and error
/// collections to be empty. Test cases meet this requirement by calling
/// TakeError() and/or TakeWarning() to consume and examine expected
/// diagnostics.
class DiagnosticPolicyTestBase : public ::testing::Test {
 public:
  DiagnosticPolicyTestBase() {
    diagnostic_policy_.SetActionForErrors(
        [this](const DiagnosticDetail& detail) {
          error_records_.push_back(detail);
        });
    diagnostic_policy_.SetActionForWarnings(
        [this](const DiagnosticDetail& detail) {
          warning_records_.push_back(detail);
        });
  }

  ~DiagnosticPolicyTestBase() override;

  /// Remove an error from internal records and return its formatted string.
  std::string TakeError() {
    ScopedTrace trace;
    SCOPED_TRACE("in TakeError()");
    return Take(&error_records_).FormatError();
  }

  /// Remove a warning from internal records and return its formatted string.
  std::string TakeWarning() {
    ScopedTrace trace;
    SCOPED_TRACE("in TakeWarning()");
    return Take(&warning_records_).FormatWarning();
  }

  /// Return the current number of errors.
  int NumErrors() { return ssize(error_records_); }

  /// Return the current number of warnings.
  int NumWarnings() { return ssize(warning_records_); }

  // This resets the diagnostic collections so that lingering reports to not
  // pollute additional testing. All current reports are silently discarded.
  void ClearDiagnostics() {
    error_records_.clear();
    warning_records_.clear();
  }

  // This will trip on unexpected errors or warnings that remain after the
  // test logic has finished. It also resets the collections so lingering
  // reports to not pollute additional testing.
  void FlushDiagnostics() {
    ScopedTrace trace;
    EXPECT_TRUE(error_records_.empty()) << DumpErrors();
    EXPECT_TRUE(warning_records_.empty()) << DumpWarnings();
    ClearDiagnostics();
  }

 protected:
  std::string DumpErrors() {
    std::stringstream stream;
    for (const DiagnosticDetail& record : error_records_) {
      stream << record.FormatError() << '\n';
    }
    return stream.str();
  }

  std::string DumpWarnings() {
    std::stringstream stream;
    for (const DiagnosticDetail& record : warning_records_) {
      stream << record.FormatWarning() << '\n';
    }
    return stream.str();
  }

  void ThrowErrors() {
    diagnostic_policy_.SetActionForErrors(
        &DiagnosticPolicy::ErrorDefaultAction);
  }

  void RecordErrors() {
    diagnostic_policy_.SetActionForErrors(
        [this](const DiagnosticDetail& detail) {
          error_records_.push_back(detail);
        });
  }

  template <typename T>
  T Take(std::deque<T>* c) {
    if (c->empty()) {
      ADD_FAILURE() << "No messages to take!";
      return {};
    }
    T result = c->at(0);
    c->pop_front();
    return result;
  }

  // Extend the gtest scoped tracing feature to give more context to failures
  // found by the DiagnosticPolicyTestBase. When a failure is found in a scope
  // containing a ScopedTrace instance, an extra message will give the file
  // name, line number, and test name in the derived ("end user") unit test.
  //
  // Note that this differs from the SCOPED_TRACE() macro, which reports the
  // source code location of itself. The mechanism here reports the source code
  // location of the derived test case.
  //
  // For details of the gtest features used, see the following documentation:
  // https://google.github.io/googletest/advanced.html#getting-the-current-tests-name
  // https://google.github.io/googletest/advanced.html#adding-traces-to-assertions
  class ScopedTrace {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScopedTrace);
    ScopedTrace() = default;

   private:
    const testing::TestInfo* const test_info_{
        testing::UnitTest::GetInstance()->current_test_info()};
    ::testing::ScopedTrace trace_{
        test_info_->file(), test_info_->line(),
        fmt::format("{}.{}", test_info_->test_suite_name(),
                    test_info_->name())};
  };

  std::deque<DiagnosticDetail> error_records_;
  std::deque<DiagnosticDetail> warning_records_;

  DiagnosticPolicy diagnostic_policy_;
};

}  // namespace test
}  // namespace drake
