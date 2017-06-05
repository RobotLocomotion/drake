#pragma once

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

// TODO(@agalbachicar) Implement me.
/// This class is a stub and will be completed in a subsequent PR.
class SplineLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SplineLane)

  SplineLane(const api::LaneId& id, const api::Segment* segment, double width,
             int index);

  ~SplineLane() override = default;

 private:
  double do_length() const override { return 0.0; }
  api::RBounds do_lane_bounds(double) const override {
    return api::RBounds(0., 0.);
  }
  api::RBounds do_driveable_bounds(double) const override {
    return api::RBounds(0., 0.);
  }
  api::GeoPosition DoToGeoPosition(const api::LanePosition&) const override {
    return api::GeoPosition();
  }
  api::LanePosition DoToLanePosition(const api::GeoPosition&, api::GeoPosition*,
                                     double*) const override {
    return api::LanePosition();
  }
  api::Rotation DoGetOrientation(const api::LanePosition&) const override {
    return api::Rotation();
  }
  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition&, const api::IsoLaneVelocity&) const override {
    return api::LanePosition();
  }
  api::HBounds do_elevation_bounds(double, double) const override {
    return api::HBounds(0., 0.);
  }
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
