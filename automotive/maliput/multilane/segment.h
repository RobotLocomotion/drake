#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

class ArcLane;
class LineLane;

/// An api::Segment implementation.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new Segment.
  ///
  /// The Segment is not fully initialized until NewLane() is called at least
  /// once. `junction` must remain valid for the lifetime of this class.
  /// @param id Segment's ID.
  /// @param junction Parent junction.
  /// @param road_curve A curve that defines the reference trajectory over the
  /// segment. A child Lane object will be constructed from an offset of the
  /// road_curve's reference curve.
  /// @param r_min Lateral distance to the minimum extent of road_curve's curve
  /// from where Segment's surface starts. It must be smaller or equal than
  /// `r_max`.
  /// @param r_max Lateral distance to the maximum extent of road_curve's curve
  /// from where Segment's surface ends. It should be greater or equal than
  /// `r_min`.
  /// @param elevation_bounds The height bounds over the segment' surface.
  Segment(const api::SegmentId& id, api::Junction* junction,
          std::unique_ptr<RoadCurve> road_curve, double r_min, double r_max,
          const api::HBounds& elevation_bounds)
      : id_(id),
        junction_(junction),
        road_curve_(std::move(road_curve)),
        r_min_(r_min),
        r_max_(r_max),
        elevation_bounds_(elevation_bounds) {
    DRAKE_DEMAND(road_curve_.get() != nullptr);
    DRAKE_DEMAND(r_min <= r_max);
    DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, elevation_bounds_));
    DRAKE_DEMAND(junction_->road_geometry()->linear_tolerance() ==
                 road_curve_->linear_tolerance());
  }

  /// Creates a new Lane object.
  ///
  /// Driveable bounds of the lane will be derived based on the lateral offset
  /// of it so as to reach `r_min` and `r_max` distances (see class constructor
  /// for more details).
  /// @param id Lane's ID.
  /// @param r0 Lateral displacement of the Lane with respect to segment
  /// RoadCurve's reference curve. It must be greater than `r_min` and smaller
  /// than `r_max`, and be greater than the last lane's r0 displacement (if
  /// any).
  /// @param lane_bounds Nominal bounds of the lane, uniform along the entire
  /// reference path. It must fit inside segments bounds when those are
  /// translated to `r0` offset distance.
  /// @return A Lane object.
  Lane* NewLane(api::LaneId id, double r0, const api::RBounds& lane_bounds);

  ~Segment() override = default;

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return lanes_.size(); }

  const api::Lane* do_lane(int index) const override;

  // Segment's ID.
  api::SegmentId id_;
  // Parent junction.
  api::Junction* junction_{};
  // Child Lane vector.
  std::vector<std::unique_ptr<Lane>> lanes_;
  // Reference trajectory over the Segment's surface.
  std::unique_ptr<RoadCurve> road_curve_;
  // Lateral distance to the minimum extent of road_curve_'s curve from where
  // Segment's surface starts.
  const double r_min_{};
  // Lateral distance to the maximum extent of road_curve_'s curve from where
  // Segment's surface ends.
  const double r_max_{};
  // Elevation bounds over the Segment's surface.
  const api::HBounds elevation_bounds_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
