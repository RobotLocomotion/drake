/* clang-format off to disable clang-format-includes */
#include "drake/common/text_logging.h"
/* clang-format on */

#include <ostream>

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

#include "drake/common/fmt_ostream.h"

namespace {

class Streamable {
  [[maybe_unused]]  // If we don't have spdlog, this function is dead code.
  friend std::ostream&
  operator<<(std::ostream& os, const Streamable& c) {
    return os << "OK";
  }
};

using drake::fmt_streamed;

// Call each API function and macro to ensure that all of them compile.
// These should all compile and run both with and without spdlog.
GTEST_TEST(TextLoggingTest, SmokeTestStreamable) {
  Streamable obj;
  drake::log()->trace("drake::log()->trace test: {} {}", "OK",
                      fmt_streamed(obj));
  drake::log()->debug("drake::log()->debug test: {} {}", "OK",
                      fmt_streamed(obj));
  drake::log()->info("drake::log()->info test: {} {}", "OK", fmt_streamed(obj));
  drake::log()->warn("drake::log()->warn test: {} {}", "OK", fmt_streamed(obj));
  drake::log()->error("drake::log()->error test: {} {}", "OK",
                      fmt_streamed(obj));
  drake::log()->critical("drake::log()->critical test: {} {}", "OK",
                         fmt_streamed(obj));
  DRAKE_LOGGER_TRACE("DRAKE_LOGGER_TRACE macro test: {}, {}", "OK",
                     fmt_streamed(obj));
  DRAKE_LOGGER_DEBUG("DRAKE_LOGGER_DEBUG macro test: {}, {}", "OK",
                     fmt_streamed(obj));
}

}  // namespace

// To enable compiling without depending on @spdlog, we need to provide our own
// main routine.  The default drake_cc_googletest_main depends on @spdlog.
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
