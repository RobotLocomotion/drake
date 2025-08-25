#include "drake/common/diagnostic_policy.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace internal {
namespace {

GTEST_TEST(DiagnosticPolicyTest, DetailTest) {
  DiagnosticDetail detail;
  detail.message = "hey";

  EXPECT_EQ(detail.Format("meh"), "meh: hey");
  EXPECT_EQ(detail.FormatWarning(), "warning: hey");
  EXPECT_EQ(detail.FormatError(), "error: hey");

  detail.filename = "something.txt";

  EXPECT_EQ(detail.Format("meh"), "something.txt: meh: hey");
  EXPECT_EQ(detail.FormatWarning(), "something.txt: warning: hey");
  EXPECT_EQ(detail.FormatError(), "something.txt: error: hey");

  detail.line = 22;

  EXPECT_EQ(detail.Format("meh"), "something.txt:22: meh: hey");
  EXPECT_EQ(detail.FormatWarning(), "something.txt:22: warning: hey");
  EXPECT_EQ(detail.FormatError(), "something.txt:22: error: hey");
}

GTEST_TEST(DiagnosticPolicyTest, WarningDefaultAction) {
  DiagnosticPolicy dut;
  DiagnosticDetail detail{"a.txt", 22, "well"};

  // Warnings, by default, do not throw.
  // An untested side-effect is that a message is logged; verify manually.
  EXPECT_NO_THROW(dut.WarningDefaultAction(detail));
}

GTEST_TEST(DiagnosticPolicyTest, ErrorDefaultAction) {
  DiagnosticPolicy dut;
  DiagnosticDetail detail{"a.txt", 22, "well"};

  //  Errors, by default, throw.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.ErrorDefaultAction(detail),
                              "a.txt:22: error: well");
}

GTEST_TEST(DiagnosticPolicyTest, SetActionForWarnings) {
  DiagnosticPolicy dut;
  DiagnosticDetail detail{"a.txt", 22, "well"};

  std::vector<DiagnosticDetail> details;
  auto warning_action = [&details](const DiagnosticDetail& d) {
    details.push_back(d);
  };
  dut.SetActionForWarnings(warning_action);

  // Errors are unchanged; they still throw.
  EXPECT_THROW(dut.Error("ouch"), std::exception);
  EXPECT_THROW(dut.Error(detail), std::exception);

  // Warnings just increment the count.
  EXPECT_EQ(details.size(), 0);
  dut.Warning("ahem");
  EXPECT_EQ(details.size(), 1);
  EXPECT_FALSE(details.back().filename.has_value());
  EXPECT_FALSE(details.back().line.has_value());
  EXPECT_EQ(details.back().message, "ahem");
  dut.Warning(detail);
  EXPECT_EQ(details.size(), 2);
  EXPECT_EQ(*details.back().filename, "a.txt");
  EXPECT_EQ(*details.back().line, 22);
  EXPECT_EQ(details.back().message, "well");

  // Setting nullptr action restores default behavior.
  dut.SetActionForWarnings(nullptr);
  EXPECT_NO_THROW(dut.Warning("ahem"));
  EXPECT_EQ(details.size(), 2);
  EXPECT_NO_THROW(dut.Warning(detail));
  EXPECT_EQ(details.size(), 2);
}

GTEST_TEST(DiagnosticPolicyTest, SetActionForErrors) {
  DiagnosticPolicy dut;
  DiagnosticDetail detail{"a.txt", 22, "well"};

  std::vector<DiagnosticDetail> details;
  auto error_action = [&details](const DiagnosticDetail& d) {
    details.push_back(d);
  };
  dut.SetActionForErrors(error_action);

  // Warnings are unchanged; they do not throw, and log a message.
  // Verify the messages manually.
  EXPECT_NO_THROW(dut.Warning("ouch"));
  EXPECT_NO_THROW(dut.Warning(detail));

  // Errors just increment the count.
  EXPECT_EQ(details.size(), 0);
  dut.Error("ahem");
  EXPECT_EQ(details.size(), 1);
  EXPECT_FALSE(details.back().filename.has_value());
  EXPECT_FALSE(details.back().line.has_value());
  EXPECT_EQ(details.back().message, "ahem");
  dut.Error(detail);
  EXPECT_EQ(details.size(), 2);
  EXPECT_EQ(*details.back().filename, "a.txt");
  EXPECT_EQ(*details.back().line, 22);
  EXPECT_EQ(details.back().message, "well");

  // Setting nullptr action restores default behavior.
  dut.SetActionForErrors(nullptr);
  EXPECT_THROW(dut.Error("ahem"), std::exception);
  EXPECT_EQ(details.size(), 2);
  EXPECT_THROW(dut.Error(detail), std::exception);
  EXPECT_EQ(details.size(), 2);
}

}  // namespace
}  // namespace internal
}  // namespace drake
