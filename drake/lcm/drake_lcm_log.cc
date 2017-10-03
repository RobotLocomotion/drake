#include "drake/lcm/drake_lcm_log.h"
#include "drake/common/drake_assert.h"
#include <iostream>

namespace drake {
namespace lcm {

DrakeLcmLog::DrakeLcmLog(const std::string& file_name, bool is_write)
    : is_write_(is_write) {
  if (is_write)
    log_ = std::make_unique<::lcm::LogFile>(file_name, "w");
  else {
    log_ = std::make_unique<::lcm::LogFile>(file_name, "r");
    next_event_ = log_->readNextEvent();
  }
}

bool DrakeLcmLog::is_write_only() const {
  return is_write_;
}

void DrakeLcmLog::Publish(const std::string& channel, const void* data, int data_size, uint64_t timestamp) {
  if (!is_write_) {
    throw std::logic_error("Should not call Publish on a read-only log.");
  }

  std::lock_guard<std::mutex> lock(mutex_);

  ::lcm::LogEvent log_event;
  log_event.timestamp = timestamp;
  log_event.channel = channel;
  log_event.datalen = data_size;
  log_event.data = const_cast<void*>(data);

  if (log_->writeEvent(&log_event) != 0) {
    throw std::runtime_error("Faield to write to log file.");
  }
}

void DrakeLcmLog::Subscribe(const std::string& channel,
    DrakeLcmMessageHandlerInterface* handler) {
  auto it = subscriptions_.find(channel);
  // No channels yet.
  if (it == subscriptions_.end()) {
    subscriptions_[channel] = {handler};
  } else {
    it->second.push_back(handler);
  }
}

double DrakeLcmLog::GetNextMessageTime() const {
  if (is_write_) {
    throw std::logic_error("GetNextMessageTime is only available for log playback.");
  }
  if (next_event_ == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  return timestamp_to_second(next_event_->timestamp);
}

void DrakeLcmLog::DispatchMessageAndAdvanceLog(double current_time) {
  if (is_write_) {
    throw std::logic_error("DispatchMessageAndAdvanceLog is only available for log playback.");
  }

  // End of log, do nothing.
  if (next_event_ == nullptr)
    return;

  // Has already called this function on the same tick, do nothing.
  if (current_time != timestamp_to_second(next_event_->timestamp)) {
    std::cout << current_time << " already called\n";
    return;
  }

  // Dispatch message if necessary.
  auto it = subscriptions_.find(next_event_->channel);
  if (it != subscriptions_.end()) {
    for (DrakeLcmMessageHandlerInterface* handler : it->second) {
      handler->HandleMessage(next_event_->channel,
          next_event_->data,
          next_event_->datalen);
    }
  }

  // Advance log.
  next_event_ = log_->readNextEvent();
}

}  // namespace lcm
}  // namespace drake
