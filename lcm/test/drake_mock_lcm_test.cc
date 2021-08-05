#include "drake/lcm/drake_mock_lcm.h"

#include <stdlib.h>

#include <gtest/gtest.h>

#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

// Tests DrakeMockLcm's round-trip ability using DrakeLcmInterface's sugar,
// without any native LCM APIs.
GTEST_TEST(DrakeMockLcmTest, AcceptanceTest) {
  ASSERT_EQ(::unsetenv("LCM_DEFAULT_URL"), 0);
  DrakeMockLcm dut;
  EXPECT_EQ(dut.get_lcm_url(), "memq://");
  const std::string channel_name = "DrakeMockLcmTest.AcceptanceTest";
  lcmt_drake_signal message{};
  message.timestamp = 10;

  {
    Subscriber<lcmt_drake_signal> subscriber(&dut, channel_name);
    Publish(&dut, channel_name, message);
    dut.HandleSubscriptions(0);
    EXPECT_EQ(subscriber.count(), 1);
    EXPECT_EQ(subscriber.message().timestamp, 10);
  }

  {
    int received = 0;
    dut.SubscribeAllChannels(
        [&received, &channel_name](
            std::string_view channel, const void*, int) {
          EXPECT_EQ(channel, channel_name);
          received++;
        });
    Publish(&dut, channel_name, message);
    dut.HandleSubscriptions(0);
    EXPECT_EQ(received, 1);
  }
}

}  // namespace
}  // namespace lcm
}  // namespace drake
