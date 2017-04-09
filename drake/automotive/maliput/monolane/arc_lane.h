#pragma once

#include <cmath>

#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace monolane {

/// Specialization of Lane with a circular arc as its reference curve
/// in the xy-plane (the ground plane) of the world frame.
class ArcLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcLane)

  /// Constructs an ArcLane, specified by a circular arc defined in the
  /// xy-plane (the ground plane) of the world frame.
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
  ///
  /// N.B. The override ArcLane::ToLanePosition() is currently restricted to
  /// lanes in which superelevation and elevation change are both zero.
  ArcLane(const api::LaneId& id, const api::Segment* segment,
          const V2& center, double radius,
          double theta0, double d_theta,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const CubicPolynomial& elevation,
          const CubicPolynomial& superelevation);

  ~ArcLane() override = default;

 private:
  // Computes the LanePosition from a given GeoPosition.  This function is valid
  // under the assumption that the road is flat (superelevation is everywhere
  // zero and the elevation has zero gradient).
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos,
      api::GeoPosition* nearest_point,
      double* distance) const override;

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
