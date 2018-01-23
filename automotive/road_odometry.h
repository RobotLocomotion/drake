#pragma once

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/systems/rendering/frame_velocity.h"

namespace drake {
namespace automotive {

/// RoadOdometry contains the position of the vehicle with respect to a lane in
/// a road, along with its velocity vector in the world frame.
template <typename T>
struct RoadOdometry {
  /// Default constructor.
  RoadOdometry() = default;
  /// Fully-parameterized constructor.
  RoadOdometry(const maliput::api::RoadPosition& road_position,
               const systems::rendering::FrameVelocity<double>& frame_velocity)
      : lane(road_position.lane), pos(road_position.pos), vel(frame_velocity) {}
  /// Fully-parameterized constructor that is T-supported.
  RoadOdometry(const maliput::api::Lane* lane_in,
               const maliput::api::LanePositionT<T>& lane_position,
               const systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(lane_in), pos(lane_position), vel(frame_velocity) {}

  const maliput::api::Lane* lane{};
  maliput::api::LanePositionT<T> pos{};
  systems::rendering::FrameVelocity<T> vel{};
};

}  // namespace automotive
}  // namespace drake
