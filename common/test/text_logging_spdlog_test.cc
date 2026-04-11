#include "drake/common/text_logging_spdlog.h"

#include <memory>
#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include "drake/common/text_logging.h"

namespace {

GTEST_TEST(TextLoggingSpdlogTest, ChangeDefaultSink) {
  // The getter should never return nullptr.
  spdlog::sinks::dist_sink_mt* const sink = drake::logging::get_dist_sink();
  ASSERT_NE(sink, nullptr);

  // Redirect all logs to a memory stream.
  std::ostringstream messages;
  auto custom_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(
      messages, true /* flush */);
  sink->set_sinks({custom_sink});
  drake::log()->info("This is some good info!");
  EXPECT_THAT(messages.str(),
              testing::EndsWith("[console] [info] This is some good info!\n"));
}

}  // namespace
