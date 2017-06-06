#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

class SplineLane;

/// An api::Segment implementation for RNDF.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new Segment.
  /// @param id to name this segment
  /// @param junction must remain valid for the lifetime of this class.
  Segment(const api::SegmentId& id, api::Junction* junction)
      : id_(id), junction_(junction) {}

  /// Gives the segment a newly constructed SplineLane.
  ///
  /// @param id is the id of the lane.
  /// @param width is the width specified by the RNDF lane_width
  /// parameter, or the default assigned value by this code. Later, this value
  /// will be used to construct the api::Lane::lane_bounds() and the
  /// api::Lane::driveable_bounds() result.
  /// @return a pointer to a valid SplineLane.
  SplineLane* NewSplineLane(const api::LaneId& id, double width);

  ~Segment() override = default;

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override { return junction_; }

  int do_num_lanes() const override { return lanes_.size(); }

  // Throws an exception if the index is not in between 0 and the size of
  // lanes_ minus one.
  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  api::Junction* junction_{};
  std::vector<std::unique_ptr<Lane>> lanes_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
