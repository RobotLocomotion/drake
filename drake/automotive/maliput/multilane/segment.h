#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
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
  /// The Segment is not fully initialized until NewLane() is called exactly
  /// @p num_lanes. @p junction must remain valid for the lifetime of this
  /// class.
  /// @param id Segment's ID.
  /// @param junction Parent junction.
  /// @param road_curve A curve that defines the reference trajectory over the
  /// segment. A child Lane object will be constructed from an offset of the
  /// road_curve's reference curve.
  /// @param num_lanes The number of Lane objects it is allowed to hold. It
  /// must be positive an non zero.
  /// @param r0 Lateral distance from the @p road_curve reference curve where
  /// the first Lane centerline lies. It must be in the range of
  /// [@p r_min, @p r_max].
  /// @param r_spacing Lateral and constant distance between centerlines of two
  /// consecutive lanes. It must be positive and make up to the last lane
  /// lateral offset fit inside the bounds.
  /// @param r_min Lateral distance to the minimum extent of road_curve's curve
  /// from where Segment's surface starts. It must be smaller or equal than
  /// @p r_max.
  /// @param r_max Lateral distance to the maximum extent of road_curve's curve
  /// from where Segment's surface ends. It should be greater or equal than
  /// @p r_min.
  /// @param elevation_bounds The height bounds over the segment' surface.
  Segment(const api::SegmentId& id, api::Junction* junction,
          std::unique_ptr<RoadCurve> road_curve, int num_lanes, double r0,
          double r_spacing, double r_min, double r_max,
          const api::HBounds& elevation_bounds);

  /// Creates a new Lane object.
  ///
  /// This method should be called no more than num_lanes times
  /// (see class constructor for more details) in the lifespan of the object.
  ///
  /// Driveable bounds of the lane will be derived based on the lateral offset
  /// of it so as to reach r_min and r_max distances (see class constructor
  /// for more details).
  /// @param id Lane's ID.
  /// @param lane_bounds
  /// @return A Lane object.
  Lane* NewLane(api::LaneId id, const api::RBounds& lane_bounds);

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
  // Number of lanes this Segment is able to hold.
  const int num_lanes_{};
  // Lateral distance from the road_curve_ reference curve where the first Lane
  // centerline lies.
  const double r0_{};
  // Lateral and constant distance between centerlines of two consecutive lanes.
  const double r_spacing_{};
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
