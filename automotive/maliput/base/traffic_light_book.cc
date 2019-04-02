#include "drake/automotive/maliput/base/traffic_light_book.h"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {

using api::rules::TrafficLight;

class TrafficLightBook::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() {}
  ~Impl() {}

  void AddTrafficLight(const TrafficLight& traffic_light) {
    auto result = book_.emplace(traffic_light.id(), traffic_light);
    if (!result.second) {
      throw std::logic_error(
          "Attempted to add multiple TrafficLight instances "
          "with ID " +
          traffic_light.id().string());
    }
  }

  optional<TrafficLight> DoGetTrafficLight(const TrafficLight::Id& id) const {
    auto it = book_.find(id);
    if (it == book_.end()) {
      return nullopt;
    }
    return it->second;
  }

 private:
  std::unordered_map<TrafficLight::Id, const TrafficLight> book_;
};

TrafficLightBook::TrafficLightBook() : impl_(std::make_unique<Impl>()) {}

TrafficLightBook::~TrafficLightBook() = default;

void TrafficLightBook::AddTrafficLight(const TrafficLight& traffic_light) {
  impl_->AddTrafficLight(traffic_light);
}

optional<TrafficLight> TrafficLightBook::DoGetTrafficLight(
    const TrafficLight::Id& id) const {
  return impl_->DoGetTrafficLight(id);
}

}  // namespace maliput
}  // namespace drake
