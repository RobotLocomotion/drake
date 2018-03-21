#include "drake/lcm/drake_lcm_log.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

template <typename MsgType>
class TestHandler : public DrakeLcmMessageHandlerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestHandler)

  explicit TestHandler(const std::string& name) : name_(name) {}

  void HandleMessage(const std::string& channel, const void* buffer,
                     int size) override {
    EXPECT_EQ(channel, name_);
    msg_.decode(buffer, 0, size);
  }

  const MsgType& get_msg() const { return msg_; }

 private:
  const std::string name_;
  MsgType msg_;
};

// Generates a log file using the write-only interface, then plays it back
// and check message content with a subscriber.
GTEST_TEST(LcmLogTest, LcmLogTestSaveAndRead) {
  auto w_log = std::make_unique<DrakeLcmLog>("test.log", true);
  const std::string channel_name("test_channel");

  drake::lcmt_drake_signal msg;
  msg.dim = 1;
  msg.val.push_back(0.1);
  msg.coord.push_back("test");
  msg.timestamp = 1234;

  std::vector<uint8_t> buffer(msg.getEncodedSize());
  EXPECT_EQ(msg.encode(&buffer[0], 0, msg.getEncodedSize()),
            msg.getEncodedSize());

  const double log_time = 111;
  w_log->Publish(channel_name, buffer.data(), buffer.size(), log_time);
  // Finish writing.
  w_log.reset();

  auto r_log = std::make_unique<DrakeLcmLog>("test.log", false);
  // Add multiple subscribers to the same channel.
  std::vector<std::unique_ptr<TestHandler<drake::lcmt_drake_signal>>> handlers;
  for (int i = 0; i < 3; i++) {
    handlers.emplace_back(
        std::make_unique<TestHandler<drake::lcmt_drake_signal>>(channel_name));
    r_log->Subscribe(channel_name, handlers.back().get());
  }

  double r_time = r_log->GetNextMessageTime();
  EXPECT_NEAR(r_time, log_time, 1e-12);
  r_log->DispatchMessageAndAdvanceLog(r_time);

  for (const auto& handler : handlers) {
    const drake::lcmt_drake_signal& decoded_msg = handler->get_msg();
    EXPECT_EQ(msg.dim, decoded_msg.dim);
    EXPECT_EQ(msg.val.size(), decoded_msg.val.size());
    EXPECT_EQ(msg.val[0], decoded_msg.val[0]);
    EXPECT_EQ(msg.coord.size(), decoded_msg.coord.size());
    EXPECT_EQ(msg.coord[0], decoded_msg.coord[0]);
    EXPECT_EQ(msg.timestamp, decoded_msg.timestamp);
  }
}

}  // namespace
}  // namespace lcm
}  // namespace drake
