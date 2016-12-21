#pragma once

#include <memory>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/monolane/lane.h"

namespace drake {
namespace maliput {
namespace monolane {

class Junction;
class ArcLane;
class LineLane;

/// An api::Segment implementation.
class Segment : public api::Segment {
 public:
  /// Constructs a new Segment.
  ///
  /// The Segment is not fully initialized until one of NewLineLane()
  /// or NewArcLane() is called exactly once.  @p junction must remain
  /// valid for the lifetime of this class.
  Segment(const api::SegmentId& id, Junction* junction)
      : id_(id), junction_(junction) {}

  /// Gives the segment a newly constructed LineLane.
  LineLane* NewLineLane(api::LaneId id,
                        const V2& xy0, const V2& dxy,
                        const api::RBounds& lane_bounds,
                        const api::RBounds& driveable_bounds,
                        const CubicPolynomial& elevation,
                        const CubicPolynomial& superelevation);

  /// Gives the segment a newly constructed ArcLane.
  ArcLane* NewArcLane(api::LaneId id,
                      const V2& center, const double radius,
                      const double theta0, const double d_theta,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds,
                      const CubicPolynomial& elevation,
                      const CubicPolynomial& superelevation);

  virtual ~Segment() {}

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return 1; }

  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  Junction* junction_{};
  std::unique_ptr<Lane> lane_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
