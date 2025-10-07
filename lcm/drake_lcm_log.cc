#include "drake/lcm/drake_lcm_log.h"

#include <chrono>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include <lcm/lcm.h>

#include "drake/common/drake_assert.h"
#include "drake/common/string_map.h"

namespace drake {
namespace lcm {

using HandlerFunction = DrakeLcmInterface::HandlerFunction;
using MultichannelHandlerFunction =
    DrakeLcmInterface::MultichannelHandlerFunction;

class DrakeLcmLog::Impl {
 public:
  string_multimap<HandlerFunction> subscriptions_;
  std::vector<MultichannelHandlerFunction> multichannel_subscriptions_;
  std::unique_ptr<::lcm_eventlog_t, decltype(&::lcm_eventlog_destroy)>  // BR
      log_{nullptr, &::lcm_eventlog_destroy};
  std::unique_ptr<::lcm_eventlog_event_t, decltype(&::lcm_eventlog_free_event)>
      next_event_{nullptr, &::lcm_eventlog_free_event};
};

DrakeLcmLog::DrakeLcmLog(const std::string& file_name, bool is_write,
                         bool overwrite_publish_time_with_system_clock)
    : is_write_(is_write),
      overwrite_publish_time_with_system_clock_(
          overwrite_publish_time_with_system_clock),
      url_("lcmlog://" + file_name),
      impl_(new Impl) {
  if (is_write_) {
    impl_->log_.reset(lcm_eventlog_create(file_name.c_str(), "w"));
  } else {
    impl_->log_.reset(lcm_eventlog_create(file_name.c_str(), "r"));
    impl_->next_event_.reset(lcm_eventlog_read_next_event(impl_->log_.get()));
  }
  if (impl_->log_ == nullptr) {
    throw std::runtime_error("Failed to open log file: " + file_name);
  }
}

DrakeLcmLog::~DrakeLcmLog() = default;

std::string DrakeLcmLog::get_lcm_url() const {
  return url_;
}

void DrakeLcmLog::Publish(const std::string& channel, const void* data,
                          int data_size, std::optional<double> time_sec) {
  if (!is_write_) {
    throw std::logic_error("Publish is only available for log saving.");
  }

  ::lcm_eventlog_event_t log_event{};
  if (!overwrite_publish_time_with_system_clock_) {
    log_event.timestamp = second_to_timestamp(time_sec.value_or(0.0));
  } else {
    log_event.timestamp = std::chrono::steady_clock::now().time_since_epoch() /
                          std::chrono::microseconds(1);
  }
  log_event.channellen = channel.size();
  log_event.channel = const_cast<char*>(channel.c_str());
  log_event.datalen = data_size;
  log_event.data = const_cast<void*>(data);
  std::lock_guard<std::mutex> lock(mutex_);
  // TODO(siyuan): should make cache this or thread write this.
  if (::lcm_eventlog_write_event(impl_->log_.get(), &log_event) != 0) {
    throw std::runtime_error("Publish failed to write to log file.");
  }
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmLog::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  if (is_write_) {
    throw std::logic_error("Subscribe is only available for log playback.");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  impl_->subscriptions_.emplace(channel, std::move(handler));
  return nullptr;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmLog::SubscribeMultichannel(
    std::string_view /* regex */, MultichannelHandlerFunction /* handler */) {
  throw std::logic_error(
      "DrakeLcmLog::SubscribeMultichannel is not implemented.");
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmLog::SubscribeAllChannels(
    MultichannelHandlerFunction handler) {
  if (is_write_) {
    throw std::logic_error("Subscribe is only available for log playback.");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  impl_->multichannel_subscriptions_.push_back(std::move(handler));
  return nullptr;
}

int DrakeLcmLog::HandleSubscriptions(int) {
  if (is_write_) {
    throw std::logic_error(
        "HandleSubscriptions is only available for log playback.");
  }
  return 0;
}

double DrakeLcmLog::GetNextMessageTime() const {
  if (is_write_) {
    throw std::logic_error(
        "GetNextMessageTime is only available for log playback.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->next_event_ == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  return timestamp_to_second(impl_->next_event_->timestamp);
}

void DrakeLcmLog::DispatchMessageAndAdvanceLog(double current_time) {
  if (is_write_) {
    throw std::logic_error(
        "DispatchMessageAndAdvanceLog is only available for log playback.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // End of log, do nothing.
  if (impl_->next_event_ == nullptr) {
    return;
  }
  const ::lcm_eventlog_event_t& next_event = *impl_->next_event_;

  // Do nothing if the call time does not match the event's time.
  if (current_time != timestamp_to_second(next_event.timestamp)) {
    return;
  }

  // Dispatch message if necessary.
  const std::string_view channel(next_event.channel, next_event.channellen);
  const auto& range = impl_->subscriptions_.equal_range(channel);
  for (auto iter = range.first; iter != range.second; ++iter) {
    const HandlerFunction& handler = iter->second;
    handler(next_event.data, next_event.datalen);
  }
  for (const auto& multi_handler : impl_->multichannel_subscriptions_) {
    multi_handler(channel, next_event.data, next_event.datalen);
  }

  // Advance log.
  impl_->next_event_.reset(lcm_eventlog_read_next_event(impl_->log_.get()));
}

void DrakeLcmLog::OnHandleSubscriptionsError(const std::string& error_message) {
  // We are not called via LCM C code, so it's safe to throw there.
  throw std::runtime_error(error_message);
}

bool DrakeLcmLog::available() {
  return true;
}

}  // namespace lcm
}  // namespace drake
