#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "lcm/lcm-cpp.hpp"
#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

class DrakeLcmLog : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmLog);

  explicit DrakeLcmLog(const std::string& file_name, bool is_write);

  void StartReceiveThread() override {}
  void StopReceiveThread() override {}

  // This actually blocks and saves to disk. so could be expensive.
  void Publish(const std::string& channel, const void* data,
               int data_size, uint64_t timestamp) override;
  void Subscribe(const std::string& channel,
                 DrakeLcmMessageHandlerInterface* handler) override;

  // Only valid in log playing mode.
  // Dispatch NextMessage to every DrakeLcmMessageHandlerInterface that's
  // listening to NextMessage's channel.
  void DispatchMessageToAllSubscribers() const;

  // Returns the next msg's time, or infinity if we run out of msg.
  double GetNextMessageTime() const;

  // Advance the log to the next msg.
  void AdvanceLog();

  bool is_write_only() const;

 private:
  const bool is_write_;

  mutable std::mutex mutex_;
  std::unique_ptr<::lcm::LogFile> log_;

  std::map<std::string, std::vector<DrakeLcmMessageHandlerInterface*>> subscriptions_;

  // The address is managed by log_
  const ::lcm::LogEvent* next_event_{nullptr};
};

}  // namespace lcm
}  // namespace drake
