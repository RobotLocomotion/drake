#pragma once

#include <memory>

#include "drake/automotive/maliput/monolane/lane.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"

#include "drake/automotive/maliput/monolane/mathiness.h"

namespace drake {
namespace maliput {
namespace monolane {

class Junction;
class ArcLane;
class LineLane;

class Segment : public api::Segment {
 public:
  Segment(const api::SegmentId& id, Junction* j)
      : id_(id), junction_(j) {}

  LineLane* NewLineLane(api::LaneId id,
                        const V2& xy0, const V2& dxy,
                        const api::RBounds& lane_bounds,
                        const api::RBounds& driveable_bounds,
                        const CubicPolynomial& elevation,
                        const CubicPolynomial& superelevation);

  ArcLane* NewArcLane(api::LaneId id,
                      const V2& center, const double radius,
                      const double theta0, const double d_theta,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds,
                      const CubicPolynomial& elevation,
                      const CubicPolynomial& superelevation);

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
