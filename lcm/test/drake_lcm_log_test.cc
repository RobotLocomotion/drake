#include "drake/lcm/drake_lcm_log.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

// Generates a log file using the write-only interface, then plays it back
// and check message content with a subscriber.
GTEST_TEST(LcmLogTest, LcmLogTestSaveAndRead) {
  auto w_log = std::make_unique<DrakeLcmLog>("test.log", true);
  const std::string channel_name("test_channel");

  drake::lcmt_drake_signal msg{};
  msg.dim = 1;
  msg.val.push_back(0.1);
  msg.coord.push_back("test");
  msg.timestamp = 1234;

  const double log_time = 111;
  Publish(w_log.get(), channel_name, msg, log_time);
  // Finish writing.
  w_log.reset();

  auto r_log = std::make_unique<DrakeLcmLog>("test.log", false);

  // Add multiple subscribers to the same channel.
  std::vector<drake::lcmt_drake_signal> messages(3, drake::lcmt_drake_signal{});
  for (int i = 0; i < 3; i++) {
    Subscribe<drake::lcmt_drake_signal>(
        r_log.get(), channel_name, [i, &messages](const auto& message) {
          messages[i] = message;
        });
  }

  // Also subscribe via SubscribeAllChannels
  bool multichannel_received = false;
  r_log->SubscribeAllChannels(
      [&multichannel_received](
          std::string_view channel, const void*, int) {
        EXPECT_EQ(channel, "test_channel");
        EXPECT_FALSE(multichannel_received);
        multichannel_received = true;
      });

  double r_time = r_log->GetNextMessageTime();
  EXPECT_NEAR(r_time, log_time, 1e-12);
  r_log->DispatchMessageAndAdvanceLog(r_time);

  for (int i = 0; i < 3; i++) {
    const auto& decoded_msg = messages[i];
    EXPECT_EQ(msg.dim, decoded_msg.dim);
    EXPECT_EQ(msg.val.size(), decoded_msg.val.size());
    EXPECT_EQ(msg.val[0], decoded_msg.val[0]);
    EXPECT_EQ(msg.coord.size(), decoded_msg.coord.size());
    EXPECT_EQ(msg.coord[0], decoded_msg.coord[0]);
    EXPECT_EQ(msg.timestamp, decoded_msg.timestamp);
  }

  EXPECT_TRUE(multichannel_received);
}

}  // namespace
}  // namespace lcm
}  // namespace drake
