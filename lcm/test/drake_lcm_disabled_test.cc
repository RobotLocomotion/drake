#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

GTEST_TEST(DrakeLcmDisabledTest, DrakeLcm) {
  EXPECT_FALSE(DrakeLcm::available());

  DrakeLcm dut;
  EXPECT_NO_THROW(std::make_unique<DrakeLcm>("memq://"));
  EXPECT_NO_THROW(std::make_unique<DrakeLcm>(DrakeLcmParams{"memq://"}));

  const std::string channel = "DRAKE_LCM_DISABLED_TEST";
  const lcmt_drake_signal signal{};
  DRAKE_EXPECT_THROWS_MESSAGE(Publish(&dut, channel, signal),
                              ".*DrakeLcm.*cannot be used.*disabled.*");

  EXPECT_NO_THROW(dut.get_lcm_url());

  auto ignore = [](auto&&...) {};
  EXPECT_NO_THROW(dut.Subscribe(channel, ignore));
  EXPECT_NO_THROW(dut.SubscribeMultichannel(channel, ignore));
  EXPECT_NO_THROW(dut.SubscribeAllChannels(ignore));

  DRAKE_EXPECT_THROWS_MESSAGE(dut.HandleSubscriptions(0),
                              ".*DrakeLcm.*cannot be used.*disabled.*");
}

GTEST_TEST(DrakeLcmDisabledTest, DrakeLcmLog) {
  EXPECT_FALSE(DrakeLcmLog::available());
  DRAKE_EXPECT_THROWS_MESSAGE(DrakeLcmLog("write.lcmlog", true),
                              ".*DrakeLcmLog.*cannot be used.*disabled.*");
}

}  // namespace
}  // namespace lcm
}  // namespace drake
