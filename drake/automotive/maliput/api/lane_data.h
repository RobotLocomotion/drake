#pragma once

#include <functional>
#include <ostream>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"

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
  LaneEnd() = default;

  /// Construct a LaneEnd specifying the @p end of @p lane.
  LaneEnd(const Lane* _lane, Which _end) : lane(_lane), end(_end) {}

  const Lane* lane{};
  Which end{};
};

/// Streams a string representation of @p which_end into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LaneEnd::Which& which_end);

/// A 3-dimensional rotation.
class Rotation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rotation)

  /// Default constructor, creating an identity Rotation.
  Rotation() : quaternion_(Quaternion<double>::Identity()) {}

  /// Constructs a Rotation from a quaternion @p quaternion (which will be
  /// normalized).
  static Rotation FromQuat(const Quaternion<double>& quaternion) {
    return Rotation(quaternion.normalized());
  }

  /// Constructs a Rotation from @p rpy, a vector of `[roll, pitch, yaw]`,
  /// expressing a roll around X, followed by pitch around Y,
  /// followed by yaw around Z (with all angles in radians).
  static Rotation FromRpy(const Vector3<double>& rpy) {
    return Rotation(math::RollPitchYawToQuaternion(rpy));
  }

  /// Constructs a Rotation expressing a @p roll around X, followed by
  /// @p pitch around Y, followed by @p yaw around Z (with all angles
  /// in radians).
  static Rotation FromRpy(double roll, double pitch, double yaw) {
    return FromRpy(Vector3<double>(roll, pitch, yaw));
  }

  /// Provides a quaternion representation of the rotation.
  const Quaternion<double>& quat() const { return quaternion_; }

  /// Sets value from a Quaternion @p quaternion (which will be normalized).
  void set_quat(const Quaternion<double>& quaternion) {
    quaternion_ = quaternion.normalized();
  }

  /// Provides a rotation matrix representation of the rotation.
  Matrix3<double> matrix() const { return quaternion_.toRotationMatrix(); }

  /// Provides a representation of rotation as a vector of angles
  /// `[roll, pitch, yaw]` (in radians).
  Vector3<double> rpy() const {
    return math::QuaternionToSpaceXYZ(to_drake(quaternion_));
  }

  // TODO(maddog@tri.global)  Deprecate and/or remove roll()/pitch()/yaw(),
  //                          since they hide the call to rpy(), and since
  //                          most call-sites should probably be using something
  //                          else (e.g., quaternion) anyway.
  /// Returns the roll component of the Rotation (in radians).
  double roll() const { return rpy().x(); }

  /// Returns the pitch component of the Rotation (in radians).
  double pitch() const { return rpy().y(); }

  /// Returns the yaw component of the Rotation (in radians).
  double yaw() const { return rpy().z(); }

 private:
  explicit Rotation(const Quaternion<double>& quat) : quaternion_(quat) {}

  // Converts Eigen (x,y,z,w) quaternion to drake's temporary(?) (w,x,y,z).
  static Vector4<double> to_drake(const Quaternion<double>& quat) {
    return Vector4<double>(quat.w(), quat.x(), quat.y(), quat.z());
  }

  Quaternion<double> quaternion_;
};

/// Streams a string representation of @p rotation into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const Rotation& rotation);

/// A position in 3-dimensional geographical Cartesian space, i.e.,
/// in the world frame, consisting of three components x, y, and z.
class GeoPosition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeoPosition)

  /// Default constructor, initializing all components to zero.
  GeoPosition() : xyz_(0., 0., 0.) {}

  /// Fully parameterized constructor.
  GeoPosition(double x, double y, double z) : xyz_(x, y, z) {}

  /// Fully parameterized constructor from a 3-vector @p xyz of the form
  /// `[x, y, z]`.
  explicit GeoPosition(const Vector3<double>& xyz) : xyz_(xyz) {}

  /// Constructs a GeoPosition from a 3-vector @p xyz of the form `[x, y, z]`.
  static GeoPosition FromXyz(const Vector3<double>& xyz) {
    return GeoPosition(xyz);
  }

  /// Returns all components as 3-vector `[x, y, z]`.
  const Vector3<double>& xyz() const { return xyz_; }
  /// Sets all components from 3-vector `[x, y, z]`.
  void set_xyz(const Vector3<double>& xyz) { xyz_ = xyz; }

  /// @name Getters and Setters
  //@{
  /// Gets `x` value.
  double x() const { return xyz_.x(); }
  /// Sets `x` value.
  void set_x(double x) { xyz_.x() = x; }
  /// Gets `y` value.
  double y() const { return xyz_.y(); }
  /// Sets `y` value.
  void set_y(double y) { xyz_.y() = y; }
  /// Gets `z` vaue.
  double z() const { return xyz_.z(); }
  /// Sets `z` value.
  void set_z(double z) { xyz_.z() = z; }
  //@}

 private:
  Vector3<double> xyz_;
};

/// Streams a string representation of @p geo_position into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const GeoPosition& geo_position);

/// GeoPosition overload for the equality operator.
bool operator==(const GeoPosition& lhs, const GeoPosition& rhs);

/// GeoPosition overload for the inequality operator.
bool operator!=(const GeoPosition& lhs, const GeoPosition& rhs);

/// A 3-dimensional position in a `Lane`-frame, consisting of three components:
///  * s is longitudinal position, as arc-length along a Lane's reference line.
///  * r is lateral position, perpendicular to the reference line at s.
///  * h is height above the road surface.
class LanePosition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LanePosition)

  /// Default constructor, initializing all components to zero.
  LanePosition() : srh_(0., 0., 0.) {}

  /// Fully parameterized constructor.
  LanePosition(double s, double r, double h) : srh_(s, r, h) {}

  /// Constructs a LanePosition from a 3-vector @p srh of the form `[s, r, h]`.
  static LanePosition FromSrh(const Vector3<double>& srh) {
    return LanePosition(srh);
  }

  /// Returns all components as 3-vector `[s, r, h]`.
  const Vector3<double>& srh() const { return srh_; }
  /// Sets all components from 3-vector `[s, r, h]`.
  void set_srh(const Vector3<double>& srh) { srh_ = srh; }

  /// @name Getters and Setters
  //@{
  /// Gets `s` value.
  double s() const { return srh_.x(); }
  /// Sets `s` value.
  void set_s(double s) { srh_.x() = s; }
  /// Gets `r` value.
  double r() const { return srh_.y(); }
  /// Sets `r` value.
  void set_r(double r) { srh_.y() = r; }
  /// Gets `h` value.
  double h() const { return srh_.z(); }
  /// Sets `h` value.
  void set_h(double h) { srh_.z() = h; }
  //@}

 private:
  Vector3<double> srh_;

  explicit LanePosition(const Vector3<double>& srh) : srh_(srh) {}
};

/// Streams a string representation of @p lane_position into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LanePosition& lane_position);

/// Isometric velocity vector in a `Lane`-frame.
///
/// sigma_v, rho_v, and eta_v are the components of velocity in a
/// (sigma, rho, eta) coordinate system.  (sigma, rho, eta) have the same
/// orientation as the (s, r, h) at any given point in space, however they
/// form an isometric system with a Cartesian distance metric.  Hence,
/// IsoLaneVelocity represents a "real" physical velocity vector (albeit
/// with an orientation relative to the road surface).
struct IsoLaneVelocity {
  /// Default constructor.
  IsoLaneVelocity() = default;

  /// Fully parameterized constructor.
  IsoLaneVelocity(double _sigma_v, double _rho_v, double _eta_v)
      : sigma_v(_sigma_v), rho_v(_rho_v), eta_v(_eta_v) {}

  double sigma_v{};
  double rho_v{};
  double eta_v{};
};


/// A position in the road network, consisting of a pointer to a specific
/// Lane and a `Lane`-frame position in that Lane.
struct RoadPosition {
  /// Default constructor.
  RoadPosition() = default;

  /// Fully parameterized constructor.
  RoadPosition(const Lane* _lane, const LanePosition& _pos)
      : lane(_lane), pos(_pos) {}

  const Lane* lane{};
  LanePosition pos;
};


/// Bounds in the lateral dimension (r component) of a `Lane`-frame, consisting
/// of a pair of minimum and maximum r value.  The bounds must straddle r = 0,
/// i.e., the minimum must be <= 0 and the maximum must be >= 0.
struct RBounds {
  /// Default constructor.
  RBounds() = default;

  /// Fully parameterized constructor.
  RBounds(double rmin, double rmax) : r_min(rmin), r_max(rmax) {
    DRAKE_DEMAND(r_min <= 0.);
    DRAKE_DEMAND(r_max >= 0.);
  }

  double r_min{};
  double r_max{};
};


/// Bounds in the elevation dimension (`h` component) of a `Lane`-frame,
/// consisting of a pair of minimum and maximum `h` value.  The bounds
/// must straddle `h = 0`, i.e., the minimum must be `<= 0` and the
/// maximum must be `>= 0`.
class HBounds {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HBounds)

  /// Default constructor.
  HBounds() = default;

  /// Fully parameterized constructor.
  HBounds(double min, double max) : min_(min), max_(max) {
    DRAKE_DEMAND(min <= 0.);
    DRAKE_DEMAND(max >= 0.);
  }

  /// @name Getters and Setters
  //@{
  /// Gets minimum bound.
  double min() const { return min_; }
  /// Sets minimum bound.
  void set_min(double min) { min_ = min; }
  /// Gets maximum bound.
  double max() const { return max_; }
  /// Sets maximum bound.
  void set_max(double max) { max_ = max; }
  //@}

 private:
  double min_{};
  double max_{};
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
