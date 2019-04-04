#pragma once

#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from TrafficLight::Id to
/// TrafficLight.
class TrafficLightBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLightBook);

  virtual ~TrafficLightBook() = default;

  /// Gets the specified TrafficLight. Returns nullopt if @p id is unrecognized.
  optional<TrafficLight> GetTrafficLight(const TrafficLight::Id& id) const {
    return DoGetTrafficLight(id);
  }

 protected:
  TrafficLightBook() = default;

 private:
  virtual optional<TrafficLight> DoGetTrafficLight(
      const TrafficLight::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
