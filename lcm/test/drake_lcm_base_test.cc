#include "drake/lcm/drake_lcm_base.h"

#include <array>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace lcm {
namespace {

class SampleLcm : public DrakeLcmBase {};

GTEST_TEST(LcmBaseTest, All) {
  SampleLcm dut;
  std::array<char, 1> data;
  DrakeLcmInterface::HandlerFunction handler;
  DrakeLcmInterface::MultichannelHandlerFunction multi_handler;

  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_lcm_url(),
                              ".*SampleLcm.*get_lcm_url.*not implemented.*");

  DRAKE_EXPECT_THROWS_MESSAGE(dut.Publish("channel", data.data(), 1, {}),
                              ".*SampleLcm.*Publish.*not.*");

  DRAKE_EXPECT_THROWS_MESSAGE(dut.Subscribe("channel", handler),
                              ".*SampleLcm.*Subscribe.*not.*");

  DRAKE_EXPECT_THROWS_MESSAGE(dut.SubscribeMultichannel(".*", multi_handler),
                              ".*SampleLcm.*Multichannel.*not.*");

  DRAKE_EXPECT_THROWS_MESSAGE(dut.SubscribeAllChannels(multi_handler),
                              ".*SampleLcm.*AllChannels.*not.*");

  DRAKE_EXPECT_THROWS_MESSAGE(dut.HandleSubscriptions(0),
                              ".*SampleLcm.*HandleSubscriptions.*not.*");
}

}  // namespace
}  // namespace lcm
}  // namespace drake
