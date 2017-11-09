#include "drake/lcm/drake_lcm_log.h"

#include <chrono>
#include <iostream>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace lcm {

DrakeLcmLog::DrakeLcmLog(const std::string& file_name, bool is_write,
                         bool overwrite_publish_time_with_system_clock)
    : is_write_(is_write),
      overwrite_publish_time_with_system_clock_(
          overwrite_publish_time_with_system_clock) {
  if (is_write) {
    log_ = std::make_unique<::lcm::LogFile>(file_name, "w");
  } else {
    log_ = std::make_unique<::lcm::LogFile>(file_name, "r");
    next_event_ = log_->readNextEvent();
  }
  if (!log_->good()) {
    throw std::runtime_error("Failed to open log file: " + file_name);
  }
}

void DrakeLcmLog::Publish(const std::string& channel, const void* data,
                          int data_size, double second) {
  if (!is_write_) {
    throw std::logic_error("Publish is only available for log saving.");
  }

  std::lock_guard<std::mutex> lock(mutex_);

  ::lcm::LogEvent log_event;
  if (!overwrite_publish_time_with_system_clock_) {
    log_event.timestamp = second_to_timestamp(second);
  } else {
    log_event.timestamp = std::chrono::steady_clock::now().time_since_epoch() /
                          std::chrono::microseconds(1);
  }
  log_event.channel = channel;
  log_event.datalen = data_size;
  log_event.data = const_cast<void*>(data);

  // TODO(siyuan): should make cache this or thread write this.
  if (log_->writeEvent(&log_event) != 0) {
    throw std::runtime_error("Publish failed to write to log file.");
  }
}

void DrakeLcmLog::Subscribe(const std::string& channel,
                            DrakeLcmMessageHandlerInterface* handler) {
  if (is_write_) {
    throw std::logic_error("Subscribe is only available for log playback.");
  }

  std::lock_guard<std::mutex> lock(mutex_);

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
    throw std::logic_error(
        "GetNextMessageTime is only available for log playback.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (next_event_ == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  return timestamp_to_second(next_event_->timestamp);
}

void DrakeLcmLog::DispatchMessageAndAdvanceLog(double current_time) {
  if (is_write_) {
    throw std::logic_error(
        "DispatchMessageAndAdvanceLog is only available for log playback.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // End of log, do nothing.
  if (next_event_ == nullptr) return;

  // Do nothing if the call time does not match the event's time.
  if (current_time != timestamp_to_second(next_event_->timestamp)) {
    return;
  }

  // Dispatch message if necessary.
  auto it = subscriptions_.find(next_event_->channel);
  if (it != subscriptions_.end()) {
    for (DrakeLcmMessageHandlerInterface* handler : it->second) {
      handler->HandleMessage(next_event_->channel, next_event_->data,
                             next_event_->datalen);
    }
  }

  // Advance log.
  next_event_ = log_->readNextEvent();
}

}  // namespace lcm
}  // namespace drake
