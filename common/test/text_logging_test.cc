#include "drake/common/text_logging.h"

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// The BUILD.bazel rules must supply this flag.  This test code is compiled and
// run twice -- once with spdlog, and once without.
#ifndef TEXT_LOGGING_TEST_SPDLOG
#error Missing a required definition to compile this test case.
#endif

// Check for the expected HAVE_SPDLOG value.
// clang-format off
#if TEXT_LOGGING_TEST_SPDLOG
  #ifndef HAVE_SPDLOG
    #error Missing HAVE_SPDLOG.
  #endif
#else
  #ifdef HAVE_SPDLOG
    #error Unwanted HAVE_SPDLOG.
  #endif
#endif
// clang-format on

namespace {
class Formattable {
 public:
  std::string to_string() const { return "OK"; }
};
}  // namespace

DRAKE_FORMATTER_AS(, , Formattable, x, x.to_string())

namespace {

using drake::logging::kHaveSpdlog;

// Call each API function and macro to ensure that all of them compile.
// These should all compile and run both with and without spdlog.
GTEST_TEST(TextLoggingTest, SmokeTestFormattable) {
  Formattable obj;
  drake::log()->trace("drake::log()->trace test: {} {}", "OK", obj);
  drake::log()->debug("drake::log()->debug test: {} {}", "OK", obj);
  drake::log()->info("drake::log()->info test: {} {}", "OK", obj);
  drake::log()->warn("drake::log()->warn test: {} {}", "OK", obj);
  drake::log()->error("drake::log()->error test: {} {}", "OK", obj);
  drake::log()->critical("drake::log()->critical test: {} {}", "OK", obj);
  DRAKE_LOGGER_TRACE("DRAKE_LOGGER_TRACE macro test: {}, {}", "OK", obj);
  DRAKE_LOGGER_DEBUG("DRAKE_LOGGER_DEBUG macro test: {}, {}", "OK", obj);
}

// Check that floating point values format sensibly.  We'll just test fmt
// directly, since we know that spdlog uses it internally.
GTEST_TEST(TextLoggingTest, FloatingPoint) {
  EXPECT_EQ(fmt::format("{}", 1.1), "1.1");
  // This number is particularly challenging.
  EXPECT_EQ(fmt::format("{}", 0.009), "0.009");
}

// Check that the constexpr bool is set correctly.
GTEST_TEST(TextLoggingTest, ConstantTest) {
#if TEXT_LOGGING_TEST_SPDLOG
  EXPECT_TRUE(kHaveSpdlog);
#else
  EXPECT_FALSE(kHaveSpdlog);
#endif
}

// Check that the "warn once" idiom compiles and doesn't crash at runtime.
// We use a pattern substitution to cover both arguments of the Warn's ctor.
GTEST_TEST(TextLoggingTest, WarnOnceTest) {
  static const drake::logging::Warn log_once("The {} happened as expected.",
                                             "log_once");
}

// Abuse gtest internals to verify that logging actually prints when enabled,
// and that the default level is INFO.
GTEST_TEST(TextLoggingTest, CaptureOutputTest) {
  testing::internal::CaptureStderr();
  drake::log()->trace("bad sentinel");
  drake::log()->debug("bad sentinel");
  drake::log()->info("good sentinel");
  std::string output = testing::internal::GetCapturedStderr();
#if TEXT_LOGGING_TEST_SPDLOG
  EXPECT_TRUE(output.find("good sentinel") != std::string::npos);
  EXPECT_TRUE(output.find("bad sentinel") == std::string::npos);
#else
  EXPECT_EQ(output, "");
#endif
}

// Verify that DRAKE_LOGGER macros succeed in avoiding evaluation of their
// arguments.
GTEST_TEST(TextLoggingTest, DrakeMacrosDontEvaluateArguments) {
  int tracearg = 0, debugarg = 0;

// Shouldn't increment argument whether the macro expanded or not, since
// logging is off.
#if TEXT_LOGGING_TEST_SPDLOG
  drake::log()->set_level(spdlog::level::off);
#endif
  DRAKE_LOGGER_TRACE("tracearg={}", ++tracearg);
  DRAKE_LOGGER_DEBUG("debugarg={}", ++debugarg);
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
  tracearg = 0;
  debugarg = 0;

// Should increment arg only if the macro expanded.
#if TEXT_LOGGING_TEST_SPDLOG
  drake::log()->set_level(spdlog::level::trace);
#endif
  DRAKE_LOGGER_TRACE("tracearg={}", ++tracearg);
  DRAKE_LOGGER_DEBUG("debugarg={}", ++debugarg);
#ifndef NDEBUG
  EXPECT_EQ(tracearg, kHaveSpdlog ? 1 : 0);
  EXPECT_EQ(debugarg, kHaveSpdlog ? 1 : 0);
#else
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
#endif
  tracearg = 0;
  debugarg = 0;

// Only DEBUG should increment arg since trace is not enabled.
#if TEXT_LOGGING_TEST_SPDLOG
  drake::log()->set_level(spdlog::level::debug);
#endif
  DRAKE_LOGGER_TRACE("tracearg={}", ++tracearg);
  DRAKE_LOGGER_DEBUG("debugarg={}", ++debugarg);
#ifndef NDEBUG
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, kHaveSpdlog ? 1 : 0);
#else
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
#endif
  tracearg = 0;
  debugarg = 0;
}

GTEST_TEST(TextLoggingTest, SetLogLevel) {
  using drake::logging::set_log_level;

#if TEXT_LOGGING_TEST_SPDLOG
  EXPECT_THROW(set_log_level("bad"), std::runtime_error);
  const std::vector<std::string> levels = {"trace", "debug",    "info", "warn",
                                           "err",   "critical", "off"};
  const std::string first_level = set_log_level("unchanged");
  std::string prev_level = "off";
  set_log_level(prev_level);
  for (const std::string& level : levels) {
    EXPECT_EQ(set_log_level(level), prev_level);
    prev_level = level;
  }
  set_log_level(first_level);
#else
  ASSERT_EQ(drake::logging::set_log_level("anything really"), "");
#endif
}

GTEST_TEST(TextLoggingTest, SetLogPattern) {
  using drake::logging::set_log_pattern;

#if TEXT_LOGGING_TEST_SPDLOG
  set_log_pattern("%v");
  set_log_pattern("%+");
#else
  set_log_pattern("anything really");
#endif
}

}  // namespace

// To enable compiling without depending on @spdlog, we need to provide our own
// main routine.  The default drake_cc_googletest_main depends on @spdlog.
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
