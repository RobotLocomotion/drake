#include "drake/lcm/drake_lcm_interface.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

class DrakeLcmInterfaceTest : public ::testing::Test {
 public:
  using Message = lcmt_drake_signal;

  DrakeLcmInterfaceTest() {
    lcm_.EnableLoopBack();

    sample_.timestamp = 123;
    sample_.dim = 1;
    sample_.coord.emplace_back("x");
    sample_.val.emplace_back(1.0);

    const int num_bytes = sample_.getEncodedSize();
    DRAKE_DEMAND(num_bytes >= 0);
    sample_bytes_.resize(static_cast<size_t>(num_bytes));
    sample_.encode(sample_bytes_.data(), 0, num_bytes);
  }

 protected:
  DrakeMockLcm lcm_;
  const std::string channel_ = "NAME";

  // A convenient, populated sample message and its encoded form.
  Message sample_{};
  std::vector<uint8_t> sample_bytes_;
};

TEST_F(DrakeLcmInterfaceTest, FreeFunctionTest) {
  // Subscribe using the helper free-function.
  Message received{};
  Subscribe<Message>(&lcm_, channel_, [&](const Message& message) {
    received = message;
  });

  // Publish using the helper free-function.
  Publish(&lcm_, channel_, sample_);
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, sample_));
}

TEST_F(DrakeLcmInterfaceTest, DefaultErrorHandlingTest) {
  // Subscribe using the helper free-function, using default error-handling.
  Message received{};
  Subscribe<Message>(&lcm_, channel_, [&](const Message& message) {
    received = message;
  });

  // Publish successfully.
  lcm_.Publish(channel_, sample_bytes_.data(), sample_bytes_.size(), {});
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, sample_));
  received = {};

  // Corrupt the message.
  std::vector<uint8_t> corrupt_bytes = sample_bytes_;
  corrupt_bytes.at(0) = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      lcm_.Publish(channel_, corrupt_bytes.data(), corrupt_bytes.size(), {}),
      std::runtime_error,
      "Error decoding message on NAME");
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, Message{}));
}

TEST_F(DrakeLcmInterfaceTest, CustomErrorHandlingTest) {
  // Subscribe using the helper free-function, using default error-handling.
  Message received{};
  bool error = false;
  Subscribe<Message>(&lcm_, channel_, [&](const Message& message) {
    received = message;
  }, [&]() {
    error = true;
  });

  // Publish successfully.
  lcm_.Publish(channel_, sample_bytes_.data(), sample_bytes_.size(), {});
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, sample_));
  EXPECT_FALSE(error);
  received = {};

  // Corrupt the message.
  std::vector<uint8_t> corrupt_bytes = sample_bytes_;
  corrupt_bytes.at(0) = 0;
  lcm_.Publish(channel_, corrupt_bytes.data(), corrupt_bytes.size(), {});
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(received, Message{}));
  EXPECT_TRUE(error);
}

}  // namespace
}  // namespace lcm
}  // namespace drake
