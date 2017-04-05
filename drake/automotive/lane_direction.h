#pragma once

#include <memory>

#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace automotive {

/// LaneDirection holds the lane that a MaliputRailcar is traversing and the
/// direction in which it is moving. A MaliputRailcar can either travel in the
/// increasing-`s` direction or in the decreasing-`s` direction.
struct LaneDirection {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneDirection)

  /// Default constructor.
  LaneDirection() {}

  /// A constructor that sets `with_s` to be `true`.
  explicit LaneDirection(const maliput::api::Lane* lane_input)
      : LaneDirection(lane_input, true) {}

  /// Fully parameterized constructor.
  LaneDirection(const maliput::api::Lane* lane_input, bool with_s_input)
      : lane(lane_input), with_s(with_s_input) {}

  const maliput::api::Lane* lane{nullptr};

  /// True means that the MaliputRailcar's `s` coordinate increases when the
  /// vehicle has positive speed. False means the opposite.
  bool with_s{true};
};

}  // namespace automotive
}  // namespace drake
