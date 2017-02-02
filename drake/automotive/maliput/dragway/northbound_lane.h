#pragma once

#include <memory>

// #include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/dragway/lane.h"

namespace drake {
namespace maliput {
namespace dragway {

/**
  Dragway's implementation of a northbound lane. The following lane is
  implemented:

  <pre>
                          driveable_bounds
                |<------------------------------->|
                            lane_bounds
            |<--------------------------------------->|

            -------------------------------------------  ———  s = length()
            |                    :                    |   ^
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
            |                    :                    |   |
  world     |                    :                    |   |
  frame:    |                    :                    |   |
            |                    :                    |   |
       X    |                    :                    |   |
       ^    |                    :                    |   |
       |    |                    :                    |   |
       |    |                    :                    |   v
  Y <--o    ---------------------o---------------------  ———  s = 0

                    r_max                r_min
            |<------------------>|<------------------>|

                y_offset
       |<----------------------->|
  </pre>

  The lane is flat with a height of zero. The origin of the lane's frame is
  defined by the `o` along the above-shown `s = 0` line.

  The lane's frame is shifted relative to the world's frame by `y_offset` along
  the world frame's Y axis. Specifically, let `(x_l, y_l)` be a point in the
  lane's frame and `(x_w, y_w)` be the same point in the world's frame. The
  relationship between these two points is as follows:

  <pre>
      (x_w, y_w) = (x_l, y_l + y_offset)
  </pre>

  Note that the `z` coordinate is always zero since the lane is flat.
**/
class NorthboundLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NorthboundLane)

  /// Constructs a NorthboundLane.
  ///
  /// @param id the lane ID. This can be any user-defined value.
  ///
  /// @param index The index of the lane with its Segment.
  ///
  /// @param length The total length of the lane.
  ///
  /// @param y_offset The conversion of a lane's y-axis coordinate into
  /// the world frame's y-axis coordinate.
  ///
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which must be a subset of @p driveable_bounds.
  ///
  /// @param driveable_bounds The driveable bounds of the lane, uniform along
  ///        the entire reference path.
  ///
  NorthboundLane(const Segment* segment, const api::LaneId& id, int index,
      double length, double y_offset, const api::RBounds& lane_bounds,
      const api::RBounds& driveable_bounds);

  ~NorthboundLane() final = default;

 private:
  api::GeoPosition DoToGeoPosition(const api::LanePosition& lane_pos) const
      final;

  api::Rotation DoGetOrientation(const api::LanePosition& lane_pos) const
      final;

  api::LanePosition DoToLanePosition(const api::GeoPosition& geo_pos) const
      final;
};


}  // namespace dragway
}  // namespace maliput
}  // namespace drake
