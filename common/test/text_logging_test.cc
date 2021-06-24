#include "drake/common/text_logging.h"

#include <ostream>
#include <sstream>
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
#if TEXT_LOGGING_TEST_SPDLOG
  #ifndef HAVE_SPDLOG
    #error Missing HAVE_SPDLOG.
  #endif
#else
  #ifdef HAVE_SPDLOG
    #error Unwanted HAVE_SPDLOG.
  #endif
#endif

#ifdef HAVE_SPDLOG
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/ostream_sink.h>
#endif  // HAVE_SPDLOG

namespace {

using drake::logging::kHaveSpdlog;

class Streamable {
  template<typename ostream_like>
  friend ostream_like& operator<<(ostream_like& os, const Streamable& c) {
    return os << "OK";
  }
};

// Call each API function and macro to ensure that all of them compile.
// These should all compile and run both with and without spdlog.
GTEST_TEST(TextLoggingTest, SmokeTest) {
  Streamable obj;
  drake::log()->trace("drake::log()->trace test: {} {}", "OK", obj);
  drake::log()->debug("drake::log()->debug test: {} {}", "OK", obj);
  drake::log()->info("drake::log()->info test: {} {}", "OK", obj);
  drake::log()->warn("drake::log()->warn test: {} {}", "OK", obj);
  drake::log()->error("drake::log()->error test: {} {}", "OK", obj);
  drake::log()->critical("drake::log()->critical test: {} {}", "OK", obj);
  DRAKE_LOGGER_TRACE("DRAKE_LOGGER_TRACE macro test: {}, {}",
                     "OK", obj);
  DRAKE_LOGGER_DEBUG("DRAKE_LOGGER_DEBUG macro test: {}, {}",
                     "OK", obj);
}

// Check that floating point values format sensibly.  We'll just test fmt
// directly, since we know that spdlog uses it internally.
GTEST_TEST(TextLoggingTest, FloatingPoint) {
  EXPECT_EQ(fmt::format("{:#}", 1.0), "1.0");
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
    const std::vector<std::string> levels = {
        "trace", "debug", "info", "warn", "err", "critical", "off"};
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

// We must run this test last because it changes the default configuration.
GTEST_TEST(TextLoggingTest, ZZZ_ChangeDefaultSink) {
  // The getter should never return nullptr, even with spdlog disabled.
  drake::logging::sink* const sink_base = drake::logging::get_dist_sink();
  ASSERT_NE(sink_base, nullptr);

  // The remainder of the test case only makes sense when spdlog is enabled.
  #if TEXT_LOGGING_TEST_SPDLOG
    // Our API promises that the result always has this subtype.
    auto* const sink = dynamic_cast<spdlog::sinks::dist_sink_mt*>(sink_base);
    ASSERT_NE(sink, nullptr);

    // Redirect all logs to a memory stream.
    std::ostringstream messages;
    auto custom_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(
        messages, true /* flush */);
    sink->set_sinks({custom_sink});
    drake::log()->info("This is some good info!");
    EXPECT_THAT(messages.str(), testing::EndsWith(
        "[console] [info] This is some good info!\n"));
  #endif
}

}  // anon namespace

// To enable compiling without depending on @spdlog, we need to provide our own
// main routine.  The default drake_cc_googletest_main depends on @spdlog.
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
