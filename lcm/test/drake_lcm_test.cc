#include "drake/lcm/drake_lcm.h"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <gtest/gtest.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
// Provides friend access to the underlying LCM object.
class DrakeLcmTester {
 public:
  DrakeLcmTester() = delete;
  static ::lcm::LCM get_native(DrakeLcm* dut) {
    DRAKE_DEMAND(dut != nullptr);
    return ::lcm::LCM(
        static_cast<::lcm_t*>(dut->get_native_lcm_handle_for_unit_testing()));
  }
};
namespace {

// A udpm URL that is not the default.  We'll transmit here in our tests.
constexpr const char* const kUdpmUrl = "udpm://239.255.76.67:7670";

// @file
// This file tests non-threaded use of DrakeLcm.
// See drake_lcm_thread_test.cc for threaded use.

using drake::lcmt_drake_signal;

// Test fixture.
class DrakeLcmTest : public ::testing::Test {
 protected:
  DrakeLcmTest() {
    message_.dim = 2;
    message_.val.push_back(0.3739558136);
    message_.val.push_back(0.2801694990);
    message_.coord.push_back("artin's constant");
    message_.coord.push_back("bernstein's constant");
    message_.timestamp = 142857;
  }

  // Call step() until the `received` matches our expected message.
  void LoopUntilDone(lcmt_drake_signal* received, int retries,
                     const std::function<void(void)>& step) {
    // Try until we're either done, or we reach the retry limit.
    bool message_was_received = false;
    int count = 0;
    while (!message_was_received && count++ < retries) {
      step();
      message_was_received =
          CompareLcmtDrakeSignalMessages(*received, message_);
    }
    EXPECT_TRUE(message_was_received);
  }

  // Returns a C++ interface wrapper around DrakeLcm's internal LCM object.
  ::lcm::LCM get_native() { return DrakeLcmTester::get_native(dut_.get()); }

  // The device under test.
  std::unique_ptr<DrakeLcm> dut_ = std::make_unique<DrakeLcm>();

  // A prototypical message with non-default values.
  lcmt_drake_signal message_{};
};

TEST_F(DrakeLcmTest, DefaultUrlTest) {
  EXPECT_TRUE(DrakeLcm::available());
  EXPECT_GT(dut_->get_lcm_url().size(), 0);
}

TEST_F(DrakeLcmTest, CustomUrlTest) {
  dut_ = std::make_unique<DrakeLcm>(kUdpmUrl);
  EXPECT_EQ(dut_->get_lcm_url(), kUdpmUrl);
}

TEST_F(DrakeLcmTest, BadUrlTest) {
  // At the moment, invalid URLs print to the console but do not throw.
  // We probably want to revisit this as some point (to fail-fast).
  EXPECT_NO_THROW(DrakeLcm("no-such-scheme://foo"));
}

TEST_F(DrakeLcmTest, DeferThreadTest) {
  DrakeLcmParams params;
  params.lcm_url = kUdpmUrl;
  params.defer_initialization = false;
  dut_ = std::make_unique<DrakeLcm>(params);
  EXPECT_EQ(dut_->get_lcm_url(), kUdpmUrl);
  // There's no easy way to check whether the thread is running.  For now,
  // we'll satisfy ourselves that the test compiles and preserves the URL.
}

TEST_F(DrakeLcmTest, EmptyChannelTest) {
  auto noop = [](const void*, int) {};
  DRAKE_EXPECT_THROWS_MESSAGE(dut_->Subscribe("", noop), ".*channel.empty.*");

  char data[1] = {};
  DRAKE_EXPECT_THROWS_MESSAGE(dut_->Publish("", data, 1, {}),
                              ".*channel.empty.*");
}

// Tests DrakeLcm's ability to publish an LCM message.
// We subscribe using the native LCM APIs.
TEST_F(DrakeLcmTest, PublishTest) {
  ::lcm::LCM native_lcm = get_native();
  const std::string channel_name = "DrakeLcmTest.PublishTest";

  lcmt_drake_signal received{};
  ::lcm::LCM::HandlerFunction<lcmt_drake_signal> handler =
      [&received](const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_drake_signal* new_value) {
        DRAKE_DEMAND(new_value != nullptr);
        received = *new_value;
      };
  native_lcm.subscribe(channel_name, std::move(handler));

  LoopUntilDone(&received, 20 /* retries */, [&]() {
    Publish(dut_.get(), channel_name, message_);
    native_lcm.handleTimeout(50 /* millis */);
  });
}

// Tests DrakeLcm's ability to subscribe to an LCM message.
// We publish using the native LCM APIs.
TEST_F(DrakeLcmTest, SubscribeTest) {
  ::lcm::LCM native_lcm = get_native();
  const std::string channel_name = "DrakeLcmTest.SubscribeTest";

  lcmt_drake_signal received{};
  auto subscription =
      dut_->Subscribe(channel_name, [&received](const void* data, int size) {
        received.decode(data, 0, size);
      });
  subscription.reset();  // Deleting the subscription should be a no-op.

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm.publish(channel_name, &message_);
    total += dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(total, 1);
}

// Repeats the above test, but with explicit opt-out of unsubscribe.
TEST_F(DrakeLcmTest, SubscribeTest2) {
  ::lcm::LCM native_lcm = get_native();
  const std::string channel_name = "DrakeLcmTest.SubscribeTest2";

  lcmt_drake_signal received{};
  auto subscription =
      dut_->Subscribe(channel_name, [&received](const void* data, int size) {
        received.decode(data, 0, size);
      });
  subscription->set_unsubscribe_on_delete(false);  // We shall be explicit.
  subscription.reset();

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm.publish(channel_name, &message_);
    total += dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(total, 1);
}

// Repeat SubscribeTest for SubscribeAllChannels.
TEST_F(DrakeLcmTest, SubscribeAllTest) {
  ::lcm::LCM native_lcm = get_native();
  const std::string channel_name = "DrakeLcmTest.SubscribeAllTest";

  lcmt_drake_signal received{};
  auto subscription = dut_->SubscribeAllChannels(
      [&received, &channel_name](std::string_view channel, const void* data,
                                 int size) {
        EXPECT_EQ(channel, channel_name);
        received.decode(data, 0, size);
      });
  subscription.reset();  // Deleting the subscription should be a no-op.

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm.publish(channel_name, &message_);
    total += dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(total, 1);
}

// Repeat SubscribeTest2 for SubscribeAllChannels.
TEST_F(DrakeLcmTest, SubscribeAllTest2) {
  ::lcm::LCM native_lcm = get_native();
  const std::string channel_name = "DrakeLcmTest.SubscribeAllTest2";

  lcmt_drake_signal received{};
  auto subscription = dut_->SubscribeAllChannels(
      [&received, &channel_name](std::string_view channel, const void* data,
                                 int size) {
        EXPECT_EQ(channel, channel_name);
        received.decode(data, 0, size);
      });
  subscription->set_unsubscribe_on_delete(false);  // We shall be explicit.
  subscription.reset();

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm.publish(channel_name, &message_);
    total += dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(total, 1);
}

// Tests DrakeLcm's round-trip ability using DrakeLcmInterface's sugar,
// without any native LCM APIs.
TEST_F(DrakeLcmTest, AcceptanceTest) {
  const std::string channel_name = "DrakeLcmTest.AcceptanceTest";
  Subscriber<lcmt_drake_signal> subscriber(dut_.get(), channel_name);
  LoopUntilDone(&subscriber.message(), 20 /* retries */, [&]() {
    Publish(dut_.get(), channel_name, message_);
    dut_->HandleSubscriptions(50 /* millis */);
  });
}

// Ensure that regex characters in the channel named are treated as literals,
// not regex directives.
TEST_F(DrakeLcmTest, SubscribeNotRegexTest) {
  const std::string channel1 = "DrakeLcmTest_SubscribeNotRegexTest";
  const std::string channel2 = "DrakeLcmTest_.*";

  Subscriber<lcmt_drake_signal> channel1_subscriber(dut_.get(), channel1);
  Subscriber<lcmt_drake_signal> channel2_subscriber(dut_.get(), channel2);

  // Publishing one channel is only ever received by its own subscriber.
  LoopUntilDone(&channel1_subscriber.message(), 20 /* retries */, [&]() {
    Publish(dut_.get(), channel1, message_);
    dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_GE(channel1_subscriber.count(), 0);
  EXPECT_EQ(channel2_subscriber.count(), 0);
  channel1_subscriber.clear();
  channel2_subscriber.clear();

  LoopUntilDone(&channel2_subscriber.message(), 20 /* retries */, [&]() {
    Publish(dut_.get(), channel2, message_);
    dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(channel1_subscriber.count(), 0);
  EXPECT_GE(channel2_subscriber.count(), 0);
}

// Tests DrakeLcm's unsubscribe.
TEST_F(DrakeLcmTest, UnsubscribeTest) {
  const std::string channel_name = "DrakeLcmTest.UnsubscribeTest";

  // First, confirm that the subscriber is active.
  lcmt_drake_signal received{};
  auto subscription =
      dut_->Subscribe(channel_name, [&](const void* data, int size) {
        received.decode(data, 0, size);
      });
  subscription->set_unsubscribe_on_delete(true);
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    Publish(dut_.get(), channel_name, message_);
    dut_->HandleSubscriptions(50 /* millis */);
  });

  // Then, unsubscribe and make sure no more messages are delivered.
  subscription.reset();
  received = {};
  Publish(dut_.get(), channel_name, message_);
  ASSERT_EQ(dut_->HandleSubscriptions(1000 /* millis */), 0);
  ASSERT_EQ(received.timestamp, 0);
}

// Tests DrakeLcm's unsubscribe after a SubscribeAll to ensure multi-channel
// subscriptions behave correctly.
TEST_F(DrakeLcmTest, UnsubscribeFromMultichannelTest) {
  const std::string channel_name =
      "DrakeLcmTest.UnsubscribeFromMultichannelTest";

  // First, confirm that the subscriber is active.
  lcmt_drake_signal received{};
  auto subscription = dut_->SubscribeAllChannels(
      [&](std::string_view channel, const void* data, int size) {
        received.decode(data, 0, size);
      });
  subscription->set_unsubscribe_on_delete(true);
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    Publish(dut_.get(), channel_name, message_);
    dut_->HandleSubscriptions(50 /* millis */);
  });

  // Then, unsubscribe and make sure no more messages are delivered.
  subscription.reset();
  received = {};
  Publish(dut_.get(), channel_name, message_);
  ASSERT_EQ(dut_->HandleSubscriptions(1000 /* millis */), 0);
  ASSERT_EQ(received.timestamp, 0);
}

// Tests DrakeLcm's queue-size controls.  These do not work with the memq URL,
// but only with real udpm URLs.
TEST_F(DrakeLcmTest, QueueCapacityTest) {
  dut_ = std::make_unique<DrakeLcm>(kUdpmUrl);
  const std::string channel_name = "DrakeLcmTest.QueueCapacityTest";

  int count = 0;
  auto subscription =
      dut_->Subscribe(channel_name, [&count](const void* data, int size) {
        ++count;
      });
  EXPECT_EQ(dut_->HandleSubscriptions(10 /* millis */), 0);

  // Send three messages, but only one comes out.
  for (int i = 0; i < 3; ++i) {
    Publish(dut_.get(), channel_name, message_);
    // Let lcm_udpm's receive thread get scheduled.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(dut_->HandleSubscriptions(1000 /* millis */), 1);
  ASSERT_EQ(dut_->HandleSubscriptions(1000 /* millis */), 0);
  EXPECT_EQ(count, 1);
  count = 0;

  // Send five messages, but only three come out.
  subscription->set_queue_capacity(3);
  for (int i = 0; i < 5; ++i) {
    Publish(dut_.get(), channel_name, message_);
    // Let lcm_udpm's receive thread get scheduled.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(dut_->HandleSubscriptions(1000 /* millis */), 3);
  ASSERT_EQ(dut_->HandleSubscriptions(1000 /* millis */), 0);
  EXPECT_EQ(count, 3);
  count = 0;
}

// Tests that upstream LCM actually obeys the IP address in the URL.
TEST_F(DrakeLcmTest, AddressFilterAcceptanceTest) {
  const std::string channel_name = "DrakeLcmTest.AddressFilterAcceptanceTest";

  // The device that should never receive any traffic.
  dut_ = std::make_unique<DrakeLcm>(kUdpmUrl);
  int count = 0;
  auto subscription = dut_->Subscribe(channel_name, [&count](const void*, int) {
    ++count;
  });

  // A crosstalker with the same port number but different address.
  std::string crosstalk_url{kUdpmUrl};
  const int port_number_sep = 20;
  DRAKE_DEMAND(crosstalk_url.at(port_number_sep) == ':');
  DRAKE_DEMAND(crosstalk_url.at(port_number_sep - 1) == '7');
  crosstalk_url[port_number_sep - 1] = '8';
  ASSERT_NE(crosstalk_url, kUdpmUrl);
  auto crosstalker = std::make_unique<DrakeLcm>(crosstalk_url);

  // Transmit on the crosstalker, should not receive on the dut.
  for (int i = 0; i < 10; ++i) {
    Publish(crosstalker.get(), channel_name, message_);
    dut_->HandleSubscriptions(100 /* millis */);
  }
  EXPECT_EQ(count, 0);
}

// Tests the channel name suffix feature.
TEST_F(DrakeLcmTest, Suffix) {
  // N.B. This test uses its own DrakeLcm object, it ignores the default `dut_`.
  DrakeLcmParams params;
  params.channel_suffix = "_SUFFIX";
  dut_ = std::make_unique<DrakeLcm>(params);
  ::lcm::LCM native_lcm = get_native();

  // Subscribe using native LCM (with the fully-qualified channel name).
  lcmt_drake_signal received_native{};
  ::lcm::LCM::HandlerFunction<lcmt_drake_signal> handler =
      [&received_native](const ::lcm::ReceiveBuffer*, const std::string&,
                         const lcmt_drake_signal* new_value) {
        DRAKE_DEMAND(new_value != nullptr);
        received_native = *new_value;
      };
  native_lcm.subscribe("SuffixDrakeLcmTest_SUFFIX", std::move(handler));

  // Subscribe using Drake LCM (with the abbreviated channel name).
  lcmt_drake_signal received_drake{};
  auto subscription = dut_->Subscribe(
      "SuffixDrakeLcmTest", [&received_drake](const void* data, int size) {
        received_drake.decode(data, 0, size);
      });

  // Publish using the abbreviated channel name.
  // Check that the native subscription gets it.
  LoopUntilDone(&received_native, 20 /* retries */, [&]() {
    Publish(dut_.get(), "SuffixDrakeLcmTest", message_);
    native_lcm.handleTimeout(50 /* millis */);
  });

  // Publish using the abbreviated channel name.
  // Check that the drake subscription gets it.
  LoopUntilDone(&received_drake, 20 /* retries */, [&]() {
    Publish(dut_.get(), "SuffixDrakeLcmTest", message_);
    native_lcm.handleTimeout(50 /* millis */);
  });
}

// Tests the channel name suffix feature.
TEST_F(DrakeLcmTest, SuffixInSubscribeAllChannels) {
  // The device under test will listen for messages.
  DrakeLcmParams params;
  params.channel_suffix = ".SUFFIX";
  params.lcm_url = kUdpmUrl;
  dut_ = std::make_unique<DrakeLcm>(params);

  // Here we use a suffix string that contains a regex character.
  // We want it to be treated as a string literal, not a regex.
  const std::string unadorned = "SuffixInSubscribeAllChannels";
  const std::string suffixed = unadorned + ".SUFFIX";
  const std::string mismatched = unadorned + "xSUFFIX";

  // Use a separate publisher, to have direct control over the channel name.
  auto publisher = std::make_unique<DrakeLcm>(kUdpmUrl);

  // Check SubscribeAll, expecting to see the unadorned channel name.
  lcmt_drake_signal received_drake{};
  auto subscription = dut_->SubscribeAllChannels(
      [&received_drake, unadorned](std::string_view channel_name,
                                   const void* data, int size) {
        EXPECT_EQ(channel_name, unadorned);
        received_drake.decode(data, 0, size);
      });
  subscription->set_queue_capacity(30);
  const lcmt_drake_signal empty_data{};
  LoopUntilDone(&received_drake, 200 /* retries */, [&]() {
    Publish(publisher.get(), mismatched, empty_data);
    Publish(publisher.get(), suffixed, message_);
    dut_->HandleSubscriptions(5 /* millis */);
  });
}

// Confirm that SubscribeMultichannel ignores mismatched channel names.
TEST_F(DrakeLcmTest, SubscribeMultiTest) {
  ::lcm::LCM native_lcm = get_native();
  const std::string channel_name = "DrakeLcmTest.SubscribeMultiTest";

  lcmt_drake_signal received{};
  auto subscription = dut_->SubscribeMultichannel(
      "Drake.*MultiTest",
      [&received, &channel_name](std::string_view channel, const void* data,
                                 int size) {
        EXPECT_EQ(channel, channel_name);
        received.decode(data, 0, size);
      });
  subscription.reset();  // Deleting the subscription should be a no-op.

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm.publish("WRONG_" + channel_name, &message_);
    native_lcm.publish(channel_name, &message_);
    total += dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(total, 1);
}

}  // namespace
}  // namespace lcm
}  // namespace drake
