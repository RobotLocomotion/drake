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

  // Returns the first error as a string (or else fails the test case,
  // if there were no errors).
  std::string FormatFirstError() {
    if (error_records_.empty()) {
      for (const auto& warning : warning_records_) {
        drake::log()->warn(warning.FormatWarning());
      }
      EXPECT_GT(error_records_.size(), 0)
          << "FormatFirstError did not get any errors";
      return {};
    }
    return error_records_[0].FormatError();
  }

  // Returns the first warning as a string (or else fails the test case,
  // if there were no warnings). Also fails if there were any errors.
  std::string FormatFirstWarning() {
    for (const auto& error : error_records_) {
      drake::log()->error(error.FormatError());
    }
    EXPECT_TRUE(error_records_.empty());
    if (warning_records_.empty()) {
      EXPECT_TRUE(warning_records_.size() > 0)
          << "FormatFirstWarning did not get any warnings";
      return {};
    }
    return warning_records_[0].FormatWarning();
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
