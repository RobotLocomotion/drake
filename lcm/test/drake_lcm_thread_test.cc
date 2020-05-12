#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "lcm/lcm-cpp.hpp"
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

// @file
// This file tests threaded use of DrakeLcm.
// See drake_lcm_test.cc for non-threaded use.

namespace drake {
namespace lcm {
namespace {

using drake::lcmt_drake_signal;

// Stores a message, guarded by a mutex.
class MessageMailbox {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MessageMailbox)

  lcmt_drake_signal GetMessage() const {
    lcmt_drake_signal result{};
    std::lock_guard<std::mutex> lock(mutex_);
    result = message_;
    return result;
  }

 protected:
  MessageMailbox() = default;

  void SetMessage(const lcmt_drake_signal& new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    message_ = new_value;
  }

 private:
  mutable std::mutex mutex_;
  lcmt_drake_signal message_{};
};

// Subscribes to LCM without any DrakeLcmInterface sugar or mocks.
class NativeMailbox final : public MessageMailbox {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NativeMailbox)

  // A constructor that adds the LCM message subscription.
  NativeMailbox(const std::string& channel_name, ::lcm::LCM* lcm) {
    lcm->subscribe(channel_name, &NativeMailbox::Handle, this);
  }

 private:
  void Handle(const ::lcm::ReceiveBuffer*, const std::string&,
              const lcmt_drake_signal* message) {
    DRAKE_DEMAND(message != nullptr);
    SetMessage(*message);
  }
};

// Subscribes to LCM using DrakeLcm::Subscribe.
class DutMailbox final : public MessageMailbox {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DutMailbox)

  DutMailbox(const std::string& channel, DrakeLcm* dut) {
    auto subscription = dut->Subscribe(
        channel, [this](const void* data, int size) {
          this->Handle(data, size);
        });
    // By default, deleting the subscription should not unsubscribe.
    subscription.reset();
  }

 private:
  void Handle(const void* data, int size) {
    lcmt_drake_signal decoded{};
    decoded.decode(data, 0, size);
    SetMessage(decoded);
  }
};

// Test fixture.
class DrakeLcmThreadTest : public ::testing::Test {
 protected:
  DrakeLcmThreadTest() {
    message_.dim = 2;
    message_.val.push_back(0.3739558136);
    message_.val.push_back(0.2801694990);
    message_.coord.push_back("artin's constant");
    message_.coord.push_back("bernstein's constant");
    message_.timestamp = 142857;
  }

  // Call publish() until the mailbox matches our expected message.
  void LoopUntilDone(const MessageMailbox& mailbox,
                     const std::function<void(void)>& publish) {
    // Launch a received thread until message_was_recived is done.
    std::atomic_bool message_was_received{false};
    DrakeLcm* const dut = &dut_;
    std::thread receive_thread([dut, &message_was_received]() {
      while (!message_was_received) {
        dut->HandleSubscriptions(300);
      }
    });

    // Try until we're either done, or we timeout (5 seconds).
    const std::chrono::milliseconds kDelay(50);
    const int kMaxCount = 100;
    int count = 0;
    while (!message_was_received && count++ < kMaxCount) {
      publish();
      std::this_thread::sleep_for(kDelay);
      message_was_received =
          CompareLcmtDrakeSignalMessages(mailbox.GetMessage(), message_);
    }
    EXPECT_TRUE(message_was_received);

    // Stop the thread.
    message_was_received = true;
    receive_thread.join();
  }

  // The device under test.
  DrakeLcm dut_;

  // A prototypical message with non-default values.
  lcmt_drake_signal message_{};
};

// Tests DrakeLcm's ability to publish an LCM message.
// We subscribe using the native LCM APIs.
TEST_F(DrakeLcmThreadTest, PublishTest) {
  ::lcm::LCM* const native_lcm = dut_.get_lcm_instance();
  const std::string channel_name = "DrakeLcmThreadTest.PublishTest";
  NativeMailbox mailbox(channel_name, native_lcm);
  LoopUntilDone(mailbox, [&]() {
    Publish(&dut_, channel_name, message_);
  });
}

// Tests DrakeLcm's ability to subscribe to an LCM message.
// We publish using the native LCM APIs.
TEST_F(DrakeLcmThreadTest, SubscribeTest) {
  ::lcm::LCM* const native_lcm = dut_.get_lcm_instance();
  const std::string channel_name = "DrakeLcmThreadTest.SubscribeTest";
  DutMailbox mailbox(channel_name, &dut_);
  LoopUntilDone(mailbox, [&]() {
    native_lcm->publish(channel_name, &message_);
  });
}

}  // namespace
}  // namespace lcm
}  // namespace drake
