#pragma once

#include <cmath>

#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

/// Specialization of Lane with a circular arc as its reference curve
/// in the xy-plane.
class ArcLane : public Lane {
 public:
  /// Constructs an ArcLane, specified by a circular arc defined in the
  /// xy-plane (the ground plane) of the World frame.
  ///
  /// @param center center of the reference arc
  /// @param radius radius of the reference arc (must be positive)
  /// @param theta0 angle of the start point of the reference arc with respect
  ///               to @p center (0 == parallel to x-axis)
  /// @param d_theta central angle of the arc, i.e., angular displacement
  ///                from start to end.  d_theta > 0 is counter-clockwise.
  ///
  /// @param id,segment,lane_bounds,driveable_bounds,elevation,superelevation
  ///        See documentation for the Lane base class.
  ArcLane(const api::LaneId& id, Segment* segment,
          const V2& center, const double radius,
          const double theta0, const double d_theta,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const CubicPolynomial& elevation,
          const CubicPolynomial& superelevation)
      : Lane(id, segment,
             lane_bounds, driveable_bounds,
             radius * std::abs(d_theta),
             elevation, superelevation),
        r_(radius), cx_(center.x()), cy_(center.y()),
        theta0_(theta0), d_theta_(d_theta) {
    DRAKE_DEMAND(r_ > 0.);
  }

  virtual ~ArcLane() {}

 private:
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos) const override;

  V2 xy_of_p(const double p) const override;
  V2 xy_dot_of_p(const double p) const override;
  double heading_of_p(const double p) const override;
  double heading_dot_of_p(const double p) const override;

  double theta_of_p(double p) const;

  double r_{};
  double cx_{};
  double cy_{};
  double theta0_{};
  double d_theta_{};
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
