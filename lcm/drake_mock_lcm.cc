#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace lcm {

DrakeMockLcm::DrakeMockLcm() {}

void DrakeMockLcm::Publish(const std::string& channel, const void* data,
                           int data_size, optional<double> time_sec) {
  DRAKE_THROW_UNLESS(!channel.empty());
  LastPublishedMessage* per_channel_data{};
  auto iter = last_published_messages_.find(channel);
  if (iter != last_published_messages_.end()) {
    per_channel_data = &iter->second;
  } else {
    per_channel_data = &last_published_messages_[channel];
  }

  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  per_channel_data->data = std::vector<uint8_t>(&bytes[0], &bytes[data_size]);
  per_channel_data->time_sec = time_sec;
  per_channel_data->handled = false;

  if (enable_loop_back_) {
    InduceSubscriberCallback(channel, data, data_size);
  }
}

const std::vector<uint8_t>& DrakeMockLcm::get_last_published_message(
    const std::string& channel) const {
  if (last_published_messages_.find(channel) ==
      last_published_messages_.end()) {
    throw std::runtime_error(
        "DrakeMockLcm::get_last_published_message: ERROR: "
        "No message was previous published on channel \"" +
        channel + "\".");
  }

  const LastPublishedMessage* message = &last_published_messages_.at(channel);
  DRAKE_DEMAND(message);

  return message->data;
}

optional<double> DrakeMockLcm::get_last_publication_time(
    const std::string& channel) const {
  auto iter = last_published_messages_.find(channel);
  if (iter == last_published_messages_.end()) {
    return nullopt;
  }
  return iter->second.time_sec;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeMockLcm::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  subscriptions_.emplace(channel, std::move(handler));
  // TODO(jwnimmer-tri) Handle unsubscribe.
  return nullptr;
}

void DrakeMockLcm::InduceSubscriberCallback(const std::string& channel,
                                            const void* data, int data_size) {
  const auto& range = subscriptions_.equal_range(channel);
  if (range.first == range.second) {
    throw std::runtime_error(
        "DrakeMockLcm::InduceSubscriberCallback: No subscription to channel "
        "\"" + channel + "\".");
  }
  for (auto iter = range.first; iter != range.second; ++iter) {
    const HandlerFunction& handler = iter->second;
    handler(data, data_size);
  }
}

int DrakeMockLcm::HandleSubscriptions(int) {
  int result = 0;
  for (auto& pub_iter : last_published_messages_) {
    const std::string& channel = pub_iter.first;
    LastPublishedMessage& per_channel_data = pub_iter.second;
    if (!per_channel_data.handled) {
      const auto& sub_range = subscriptions_.equal_range(channel);
      for (auto iter = sub_range.first; iter != sub_range.second; ++iter) {
        const HandlerFunction& handler = iter->second;
        handler(per_channel_data.data.data(), per_channel_data.data.size());
      }
      ++result;
      per_channel_data.handled = true;
    }
  }
  return result;
}

}  // namespace lcm
}  // namespace drake
