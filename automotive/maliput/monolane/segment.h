#pragma once

#include <functional>
#include <memory>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace monolane {

class ArcLane;
class LineLane;

/// An api::Segment implementation.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment);

  /// Constructs a new Segment.
  ///
  /// The Segment is not fully initialized until one of NewLineLane()
  /// or NewArcLane() is called exactly once.  @p junction must remain
  /// valid for the lifetime of this class.
  ///
  /// @p register_lane will be called on any new Lane instance created as
  /// a child of the Segment.
  Segment(const api::SegmentId& id, const api::Junction* junction,
          const std::function<void(const api::Lane*)>& register_lane)
      : id_(id), junction_(junction), register_lane_(register_lane) {}

  /// Gives the segment a newly constructed LineLane.
  LineLane* NewLineLane(api::LaneId id,
                        const V2& xy0, const V2& dxy,
                        const api::RBounds& lane_bounds,
                        const api::RBounds& driveable_bounds,
                        const api::HBounds& elevation_bounds,
                        const CubicPolynomial& elevation,
                        const CubicPolynomial& superelevation);

  /// Gives the segment a newly constructed ArcLane.
  ArcLane* NewArcLane(api::LaneId id,
                      const V2& center, const double radius,
                      const double theta0, const double d_theta,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds,
                      const api::HBounds& elevation_bounds,
                      const CubicPolynomial& elevation,
                      const CubicPolynomial& superelevation);

  ~Segment() override = default;

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return 1; }

  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  const api::Junction* junction_{};
  std::function<void(const api::Lane*)> register_lane_;
  std::unique_ptr<Lane> lane_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
