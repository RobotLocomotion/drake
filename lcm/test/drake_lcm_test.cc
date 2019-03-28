#include "drake/lcm/drake_lcm.h"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>
#include "lcm/lcm-cpp.hpp"

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
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

  // The device under test.
  std::unique_ptr<DrakeLcm> dut_ = std::make_unique<DrakeLcm>();

  // A prototypical message with non-default values.
  lcmt_drake_signal message_{};
};

TEST_F(DrakeLcmTest, DefaultUrlTest) {
  EXPECT_EQ(dut_->get_requested_lcm_url(), "");
  EXPECT_GT(dut_->get_lcm_url().size(), 0);
}

TEST_F(DrakeLcmTest, CustomUrlTest) {
  dut_ = std::make_unique<DrakeLcm>(kUdpmUrl);
  EXPECT_EQ(dut_->get_requested_lcm_url(), kUdpmUrl);
  EXPECT_EQ(dut_->get_lcm_url(), kUdpmUrl);
}

TEST_F(DrakeLcmTest, EmptyChannelTest) {
  auto noop = [](const void*, int) {};
  DRAKE_EXPECT_THROWS_MESSAGE(dut_->Subscribe("", noop), std::exception,
      ".*channel.empty.*");

  char data[1] = {};
  DRAKE_EXPECT_THROWS_MESSAGE(dut_->Publish("", data, 1, {}), std::exception,
      ".*channel.empty.*");
}

// Tests DrakeLcm's ability to publish an LCM message.
// We subscribe using the native LCM APIs.
TEST_F(DrakeLcmTest, PublishTest) {
  ::lcm::LCM* const native_lcm = dut_->get_lcm_instance();
  const std::string channel_name = "DrakeLcmTest.PublishTest";

  lcmt_drake_signal received{};
  ::lcm::LCM::HandlerFunction<lcmt_drake_signal> handler = [&received](
      const ::lcm::ReceiveBuffer*, const std::string&,
      const lcmt_drake_signal* new_value) {
    DRAKE_DEMAND(new_value != nullptr);
    received = *new_value;
  };
  native_lcm->subscribe(channel_name, std::move(handler));

  LoopUntilDone(&received, 20 /* retries */, [&]() {
    Publish(dut_.get(), channel_name, message_);
    native_lcm->handleTimeout(50 /* millis */);
  });
}

// Tests DrakeLcm's ability to subscribe to an LCM message.
// We publish using the native LCM APIs.
TEST_F(DrakeLcmTest, SubscribeTest) {
  ::lcm::LCM* const native_lcm = dut_->get_lcm_instance();
  const std::string channel_name = "DrakeLcmTest.SubscribeTest";

  lcmt_drake_signal received{};
  auto subscription = dut_->Subscribe(channel_name, [&received](
      const void* data, int size) {
    received.decode(data, 0, size);
  });
  subscription.reset();  // Deleting the subscription should be a no-op.

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm->publish(channel_name, &message_);
    total += dut_->HandleSubscriptions(50 /* millis */);
  });
  EXPECT_EQ(total, 1);
}

// Repeats the above test, but with explicit opt-out of unsubscribe.
TEST_F(DrakeLcmTest, SubscribeTest2) {
  ::lcm::LCM* const native_lcm = dut_->get_lcm_instance();
  const std::string channel_name = "DrakeLcmTest.SubscribeTest2";

  lcmt_drake_signal received{};
  auto subscription = dut_->Subscribe(channel_name, [&received](
      const void* data, int size) {
    received.decode(data, 0, size);
  });
  subscription->set_unsubscribe_on_delete(false);  // We shall be explicit.
  subscription.reset();

  int total = 0;
  LoopUntilDone(&received, 20 /* retries */, [&]() {
    native_lcm->publish(channel_name, &message_);
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

// Tests DrakeLcm's unsubscribe.
TEST_F(DrakeLcmTest, UnsubscribeTest) {
  const std::string channel_name = "DrakeLcmTest.UnsubscribeTest";

  // First, confirm that the subscriber is active.
  lcmt_drake_signal received{};
  auto subscription = dut_->Subscribe(channel_name, [&](
      const void* data, int size) {
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
  auto subscription = dut_->Subscribe(channel_name, [&count](
      const void* data, int size) {
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

}  // namespace
}  // namespace lcm
}  // namespace drake
