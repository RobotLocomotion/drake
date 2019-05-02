#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "ignition/math/Vector3.hh"

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
  /// @param id This segment's ID.
  /// @param junction The api::Junction that contains this Segment. It must
  /// remain valid for the lifetime of this object.
  Segment(const api::SegmentId& id, api::Junction* junction)
      : id_(id), junction_(junction) {}

  /// Gives the segment a newly constructed SplineLane.
  ///
  /// @param id The lane's ID.
  /// @param control_points A vector of tuples that hold the point (first
  /// element) and the tangent (second element) at that point to construct the
  /// spline based lane. The size should be at least two pairs.
  /// @param width The width specified by the RNDF lane_width
  /// parameter. Later, this value will be used to construct the
  /// api::Lane::lane_bounds() and the api::Lane::driveable_bounds() result.
  /// @return a pointer to a valid SplineLane that was added to this Segment.
  /// @throws std::runtime_error When @p control_points' size is less than 2.
  SplineLane* NewSplineLane(
      const api::LaneId& id,
      const std::vector<std::tuple<ignition::math::Vector3d,
                                   ignition::math::Vector3d>>& control_points,
      double width);

  ~Segment() override = default;

 private:
  api::SegmentId do_id() const override { return id_; }

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
