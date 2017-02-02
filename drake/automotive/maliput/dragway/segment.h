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
/// lanes. For the lane semantics, see the class descriptions of Lane.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new dragway Segment.
  ///
  /// @param[in] junction The junction to which this Segment belongs.
  ///
  /// @param[in] num_lanes The number of lanes in the segment.
  ///
  /// @param[in] length The length of the dragway.
  ///
  /// @param[in] lane_width The width of each lane.
  ///
  /// @param[in] shoulder_width The width of the shoulders on each side of the
  /// road.
  Segment(Junction* junction,
      int num_lanes,
      double length,
      double lane_width,
      double shoulder_width);

  ~Segment() override = default;

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return static_cast<int>(lanes_.size()); }

  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  Junction* junction_{};
  std::vector<std::unique_ptr<api::Lane>> lanes_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
