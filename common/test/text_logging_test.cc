#include "drake/common/text_logging.h"

#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

class Streamable {
  template<typename ostream_like>
  friend ostream_like& operator<<(ostream_like& os, const Streamable& c) {
    return os << "OK";
  }
};

// Call each API function and macro to ensure that all of them compile.
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

// Abuse gtest internals to verify that logging actually prints.
#ifdef HAVE_SPDLOG
GTEST_TEST(TextLoggingTest, CaptureOutputTest) {
  testing::internal::CaptureStderr();
  drake::log()->info("sentinel string");
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.find("sentinel string") != std::string::npos);
}
#endif

// Verify that DRAKE_SPDLOG macros succeed in avoiding evaluation of
// their arguments. (The plain SPDLOG ones currently evaluate unconditionally
// when expanded but we won't check that here since it could change.)
#ifdef HAVE_SPDLOG
GTEST_TEST(TextLoggingTest, DrakeMacrosDontEvaluateArguments) {
  int tracearg = 0, debugarg = 0;
  drake::log()->set_level(spdlog::level::off);

  // Shouldn't increment argument whether the macro expanded or not, since
  // logging is off.
  DRAKE_SPDLOG_TRACE(drake::log(), "tracearg={}", ++tracearg);
  DRAKE_SPDLOG_DEBUG(drake::log(), "debugarg={}", ++debugarg);
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);

  drake::log()->set_level(spdlog::level::trace);
  // Should increment arg only if the macro expanded.
  DRAKE_SPDLOG_TRACE(drake::log(), "tracearg={}", ++tracearg);
  DRAKE_SPDLOG_DEBUG(drake::log(), "debugarg={}", ++debugarg);
  #ifndef NDEBUG
    EXPECT_EQ(tracearg, 1);
    EXPECT_EQ(debugarg, 1);
  #else
    EXPECT_EQ(tracearg, 0);
    EXPECT_EQ(debugarg, 0);
  #endif

  drake::log()->set_level(spdlog::level::debug);
  // Only DEBUG should increment arg since trace is not enabled.
  DRAKE_SPDLOG_TRACE(drake::log(), "tracearg={}", ++tracearg);
  DRAKE_SPDLOG_DEBUG(drake::log(), "debugarg={}", ++debugarg);
  #ifndef NDEBUG
    EXPECT_EQ(tracearg, 1);
    EXPECT_EQ(debugarg, 2);
  #else
    EXPECT_EQ(tracearg, 0);
    EXPECT_EQ(debugarg, 0);
  #endif
}
#endif

}  // anon namespace
