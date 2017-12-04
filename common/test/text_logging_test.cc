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

// Call each API function and macro to ensure that all of them compile; the
// cmake script will cause this to be compiled in both spdlog and non-spdlog
// modes.
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

}  // anon namespace
