#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>
#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace lcm {

DrakeMockLcm::DrakeMockLcm() {}

void DrakeMockLcm::Publish(const std::string& channel, const void* data,
                           int data_size, optional<double>) {
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
  per_channel_data->handled = false;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeMockLcm::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  subscriptions_.emplace(channel, std::move(handler));
  // TODO(jwnimmer-tri) Handle unsubscribe.
  return nullptr;
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
