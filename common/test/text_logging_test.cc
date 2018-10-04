#include "drake/common/text_logging.h"

#include <ostream>
#include <string>
#include <vector>

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
  SPDLOG_TRACE(drake::log(), "SPDLOG_TRACE macro test: {}, {}", "OK", obj);
  SPDLOG_DEBUG(drake::log(), "SPDLOG_DEBUG macro test: {}, {}", "OK", obj);
  DRAKE_SPDLOG_TRACE(drake::log(), "DRAKE_SPDLOG_TRACE macro test: {}, {}",
                     "OK", obj);
  DRAKE_SPDLOG_DEBUG(drake::log(), "DRAKE_SPDLOG_DEBUG macro test: {}, {}",
                     "OK", obj);
}

// Check that the constexpr bool is set correctly.
GTEST_TEST(TextLoggingTest, ConstantTest) {
  #if TEXT_LOGGING_TEST_SPDLOG
    EXPECT_TRUE(kHaveSpdlog);
  #else
    EXPECT_FALSE(kHaveSpdlog);
  #endif
}

// Abuse gtest internals to verify that logging actually prints, when enabled.
GTEST_TEST(TextLoggingTest, CaptureOutputTest) {
  testing::internal::CaptureStderr();
  drake::log()->info("sentinel string");
  std::string output = testing::internal::GetCapturedStderr();
  #if TEXT_LOGGING_TEST_SPDLOG
    EXPECT_TRUE(output.find("sentinel string") != std::string::npos);
  #else
    EXPECT_EQ(output, "");
  #endif
}

// Verify that DRAKE_SPDLOG macros succeed in avoiding evaluation of their
// arguments. (The plain SPDLOG ones currently evaluate unconditionally when
// expanded but we won't check that here since it could change.)
GTEST_TEST(TextLoggingTest, DrakeMacrosDontEvaluateArguments) {
  int tracearg = 0, debugarg = 0;

  // Shouldn't increment argument whether the macro expanded or not, since
  // logging is off.
  #if TEXT_LOGGING_TEST_SPDLOG
    drake::log()->set_level(spdlog::level::off);
  #endif
  DRAKE_SPDLOG_TRACE(drake::log(), "tracearg={}", ++tracearg);
  DRAKE_SPDLOG_DEBUG(drake::log(), "debugarg={}", ++debugarg);
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
  tracearg = 0;
  debugarg = 0;

  // Should increment arg only if the macro expanded.
  #if TEXT_LOGGING_TEST_SPDLOG
    drake::log()->set_level(spdlog::level::trace);
  #endif
  DRAKE_SPDLOG_TRACE(drake::log(), "tracearg={}", ++tracearg);
  DRAKE_SPDLOG_DEBUG(drake::log(), "debugarg={}", ++debugarg);
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
  DRAKE_SPDLOG_TRACE(drake::log(), "tracearg={}", ++tracearg);
  DRAKE_SPDLOG_DEBUG(drake::log(), "debugarg={}", ++debugarg);
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

}  // anon namespace

// To enable compiling without depending on @spdlog, we need to provide our own
// main routine.  The default drake_cc_googletest_main depends on @spdlog.
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
