#pragma once

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// RoadCurve specification for a reference curve that describes a line.
class LineRoadCurve : public RoadCurve {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LineRoadCurve)

  /// Constructor. Computes a line from @p xy0 as the initial point of the line
  /// and @p dxy as the difference vector that connects the @p xy0 with the end
  /// point of the reference curve.
  /// @param xy0 A 2D vector that represents the first point of the lane.
  /// @param dxy A 2D difference vector between the last point and @p xy0.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. See RoadCurve class constructor for more details.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. See RoadCurve class constructor for more
  /// details.
  explicit LineRoadCurve(const Vector2<double>& xy0, const Vector2<double>& dxy,
                         const CubicPolynomial& elevation,
                         const CubicPolynomial& superelevation)
      : RoadCurve(elevation, superelevation),
        p0_(xy0),
        dp_(dxy),
        heading_(std::atan2(dxy.y(), dxy.x())) {
    DRAKE_DEMAND(dxy.norm() > kMinimumNorm);
  }

  ~LineRoadCurve() override = default;

  double p_from_s(double s, double r) const override;

  double s_from_p(double p, double r) const override;

  Vector2<double> xy_of_p(double p) const override { return p0_ + p * dp_; }

  Vector2<double> xy_dot_of_p(double p) const override {
    unused(p);
    return dp_;
  }

  double heading_of_p(double p) const override {
    unused(p);
    return heading_;
  }

  double heading_dot_of_p(double p) const override {
    unused(p);
    return 0.;
  }

  double p_scale() const override { return dp_.norm(); }

  Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      double r_min, double r_max,
      const api::HBounds& height_bounds) const override;

  bool IsValid(
      double r_min, double r_max,
      const api::HBounds& height_bounds) const override {
    unused(r_min);
    unused(r_max);
    unused(height_bounds);
    return true;
  }

 private:
  // The first point in world coordinates over the z=0 plane of the reference
  // curve.
  const Vector2<double> p0_{};
  // The difference vector that joins the end point of the reference curve with
  // the first one, p0_.
  const Vector2<double> dp_{};
  // The constant angle deviation of dp_ with respect to the x axis of the world
  // frame.
  const double heading_{};
  // The minimum dp_ norm to avoid having issues when computing heading_.
  static const double kMinimumNorm;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
