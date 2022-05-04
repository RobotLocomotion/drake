#pragma once

#include <deque>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
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

  ~DiagnosticPolicyTestBase() {
    FlushDiagnostics();
  }

  /// Remove an error from internal records and return its formatted string.
  std::string TakeError() {
    EXPECT_FALSE(error_records_.empty());
    return Take(&error_records_).FormatError();
  }

  /// Remove a warning from internal records and return its formatted string.
  std::string TakeWarning() {
    EXPECT_FALSE(warning_records_.empty());
    return Take(&warning_records_).FormatWarning();
  }

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

  template <typename T>
  T Take(std::deque<T>* c) {
    T result = c->at(0);
    c->pop_front();
    return result;
  }

  std::deque<DiagnosticDetail> error_records_;
  std::deque<DiagnosticDetail> warning_records_;

  DiagnosticPolicy diagnostic_policy_;
};

}  // namespace test
}  // namespace multibody
}  // namespace drake
