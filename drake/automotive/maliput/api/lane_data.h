#pragma once

#include <functional>
#include <string>

namespace drake {
namespace maliput {
namespace api {


class Lane;


/// A specific endpoint of a specific Lane.
struct LaneEnd {
  /// Labels for the endpoints of a Lane.
  /// kStart is the "s == 0" end, and kFinish is the other end.
  enum Which { kStart, kFinish, };

  /// An arbitrary strict complete ordering, useful for, e.g., std::map.
  struct StrictOrder {
    bool operator()(const LaneEnd& lhs, const LaneEnd& rhs) const {
      auto as_tuple = [](const LaneEnd& le) {
        return std::tie(le.lane, le.end);
      };
      return as_tuple(lhs) < as_tuple(rhs);
    }
  };

  /// Default constructor.
  LaneEnd() {}

  /// Construct a LaneEnd specifying the @p end of @p lane.
  LaneEnd(const Lane* _lane, Which _end) : lane(_lane), end(_end) {}

  const Lane* lane{};
  Which end{};
};


/// A 3-dimensional rotation, expressed as a roll around X, followed
/// by pitch around Y, followed by yaw around Z.
struct Rotation {
  /// Default constructor.
  Rotation() {}

  /// Fully parameterized constructor.
  Rotation(double _roll, double _pitch, double _yaw)
      : roll(_roll), pitch(_pitch), yaw(_yaw) {}

  double roll{};
  double pitch{};
  double yaw{};
};


/// A position in 3-dimensional geographical Cartesian space.
struct GeoPosition {
  /// Default constructor.
  GeoPosition() {}

  /// Fully parameterized constructor.
  GeoPosition(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

  double x{};
  double y{};
  double z{};
};


/// A 3-dimensional position in LANE-space, consisting of three components:
///  * s is longitudinal position, as arc-length along a Lane's reference line.
///  * r is lateral position, perpendicular to the reference line at s.
///  * h is height above the road surface.
struct LanePosition {
  /// Default constructor.
  LanePosition() {}

  /// Fully parameterized constructor.
  LanePosition(double _s, double _r, double _h) : s(_s), r(_r), h(_h) {}

  double s{};
  double r{};
  double h{};
};


/// Isometric velocity vector in LANE-space.
///
/// sigma_v, rho_v, and eta_v are the components of velocity in a
/// (sigma, rho, eta) coordinate system.  (sigma, rho, eta) have the same
/// orientation as the (s, r, h) at any given point in space, however they
/// form an isometric system with a Cartesian distance metric.  Hence,
/// IsoLaneVelocity represents a "real" physical velocity vector (albeit
/// with an orientation relative to the road surface).
struct IsoLaneVelocity {
  /// Default constructor.
  IsoLaneVelocity() {}

  /// Fully parameterized constructor.
  IsoLaneVelocity(double _sigma_v, double _rho_v, double _eta_v)
      : sigma_v(_sigma_v), rho_v(_rho_v), eta_v(_eta_v) {}

  double sigma_v{};
  double rho_v{};
  double eta_v{};
};


/// A position in the road network, consisting of a pointer to a specific
/// Lane and a LANE-space position on that Lane.
struct RoadPosition {
  /// Default constructor.
  RoadPosition() {}

  /// Fully parameterized constructor.
  RoadPosition(const Lane* _lane, const LanePosition& _pos)
      : lane(_lane), pos(_pos) {}

  const Lane* lane{};
  LanePosition pos;
};


/// Bounds in the lateral dimension (r component) of LANE-space, consisting
/// of a pair of minimum and maximum r value.
struct RBounds {
  /// Default constructor.
  RBounds() {}

  /// Fully parameterized constructor.
  RBounds(double rmin, double rmax) : r_min(rmin), r_max(rmax) {}

  double r_min{};
  double r_max{};
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
