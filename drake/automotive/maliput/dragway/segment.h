#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace dragway {

class Junction;

/// Dragway's implementation of api::Segment. It contains multiple straight
/// lanes. The lanes are oriented northbound or southbound. For the semantics of
/// northbound and southbound, see the class descriptions of NorthboundLane and
/// SouthboundLane.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)
  static const char* const kDragwaySegmentId;

  /// Constructs a new Segment for a dragway.
  ///
  /// @param[in] junction The junction to which this Segment belongs.
  ///
  /// @param[in] num_southbound_lanes The number of northbound lanes.
  ///
  /// @param[in] num_northbound_lanes The number of southbound lanes.
  ///
  /// @param[in] length The length of the segment.
  ///
  /// @param[in] lane_bounds The bounds of each lane.
  ///
  /// @param[in] drivable_bounds The drivable bounds of each lane.
  Segment(Junction* junction,
      int num_southbound_lanes, int num_northbound_lanes, double length,
      const api::RBounds& lane_bounds, const api::RBounds& driveable_bounds);

  ~Segment() override = default;

  /// Returns the number of lanes in this segment.
  int num_lanes() const { return static_cast<int>(lanes_.size()); }
  int num_southbound_lanes() const { return num_southbound_lanes_; }
  int num_northbound_lanes() const { return num_northbound_lanes_; }

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return static_cast<int>(lanes_.size()); }

  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  Junction* junction_{};
  const int num_southbound_lanes_{};
  const int num_northbound_lanes_{};
  std::vector<std::unique_ptr<api::Lane>> lanes_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
