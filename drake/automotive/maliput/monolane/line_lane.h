#pragma once

#include "drake/automotive/maliput/monolane/lane.h"

namespace drake {
namespace maliput {
namespace monolane {

/// Specialization of Lane with a line segment as its reference curve
/// in the xy-plane (the ground plane) of the world frame.
class LineLane : public Lane {
 public:
  /// Constructs a LineLane, a lane specified by a line segment defined in the
  /// xy-plane (the ground plane) of the world frame.
  ///
  /// @param xy0 start point of the reference line segment
  /// @param dxy displacement to the end point of the reference line segment
  ///
  /// @param id,segment,lane_bounds,driveable_bounds,elevation,superelevation
  ///        See documentation for the Lane base class.
  LineLane(const api::LaneId& id, const Segment* segment,
           const V2& xy0, const V2& dxy,
           const api::RBounds& lane_bounds,
           const api::RBounds& driveable_bounds,
           const CubicPolynomial& elevation,
           const CubicPolynomial& superelevation)
      : Lane(id, segment,
             lane_bounds, driveable_bounds,
             dxy.norm(),
             elevation, superelevation),
        x0_(xy0.x()),
        y0_(xy0.y()),
        dx_(dxy.x()),
        dy_(dxy.y()),
        heading_(std::atan2(dy_, dx_)) {}

  virtual ~LineLane() {}

 private:
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos) const override;

  V2 xy_of_p(const double p) const override;
  V2 xy_dot_of_p(const double p) const override;
  double heading_of_p(const double p) const override;
  double heading_dot_of_p(const double p) const override;

  const double x0_{};
  const double y0_{};
  const double dx_{};
  const double dy_{};
  const double heading_{};  // Memoized; derived from dy_, dx_.
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
