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
  if (last_published_messages_.find(channel) ==
      last_published_messages_.end()) {
    last_published_messages_[channel] = LastPublishedMessage();
  }
  LastPublishedMessage* saved_message{nullptr};
  saved_message = &last_published_messages_[channel];

  DRAKE_DEMAND(saved_message);

  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  saved_message->data = std::vector<uint8_t>(&bytes[0], &bytes[data_size]);
  saved_message->time_sec = time_sec;

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

void DrakeMockLcm::Subscribe(const std::string& channel,
                             HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  subscriptions_.emplace(channel, std::move(handler));
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

}  // namespace lcm
}  // namespace drake
