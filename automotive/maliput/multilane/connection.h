#pragma once

#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

/// XY-plane-only parameters for an endpoint of a connection, specified in
/// the world frame.
///
/// The three components are:
///  - x: x position
///  - y: y position
///  - heading: counter-clockwise around z=cross(x,y). It is the heading of
///             reference path (radians, zero == x-direction).
class EndpointXy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointXy)

  // Constructs an EndpointXy with all zero parameters.
  EndpointXy() = default;

  EndpointXy(double x, double y, double heading)
      : x_(x), y_(y), heading_(heading) {}

  /// Returns an EndpointXy with reversed direction.
  EndpointXy reverse() const {
    return EndpointXy(x_, y_,
                      std::atan2(-std::sin(heading_), -std::cos(heading_)));
  }

  double x() const { return x_; }

  double y() const { return y_; }

  double heading() const { return heading_; }

 private:
  double x_{};
  double y_{};
  double heading_{};
};

/// Streams a string representation of `endpoint_xy` into `out`. Returns
/// `out`. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const EndpointXy& endpoint_xy);

/// Out-of-plane parameters for an endpoint of a connection, specified in
/// the world frame.
///
/// The four components are:
///  - z: elevation
///  - z_dot: grade (rate of change of elevation with respect to
///           arc length of the reference path)
///  - theta: superelevation (rotation of road surface around r = 0 centerline;
///           when theta > 0, elevation at r > 0 is above elevation at r < 0)
///  - theta_dot: rate of change of superelevation with respect to arc length
///               of the reference path
class EndpointZ {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointZ)
  // Constructs an EndpointZ with all zero parameters.
  EndpointZ() = default;

  EndpointZ(double z, double z_dot, double theta, double theta_dot)
      : z_(z), z_dot_(z_dot), theta_(theta), theta_dot_(theta_dot) {}

  /// Returns an EndpointZ with reversed direction.
  ///
  /// Reversing direction is equivalent to rotating s (and along with it, r)
  /// around the h-axis by 180 degrees, thus flipping the signs of z_dot
  /// and theta.
  EndpointZ reverse() const {
    return EndpointZ(z_, -z_dot_, -theta_, theta_dot_);
  }

  double z() const { return z_; }

  double z_dot() const { return z_dot_; }

  double theta() const { return theta_; }

  double theta_dot() const { return theta_dot_; }

 private:
  double z_{};
  double z_dot_{};

  double theta_{};
  double theta_dot_{};
};

/// Streams a string representation of `endpoint_z` into `out`. Returns
/// `out`. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const EndpointZ& endpoint_z);

/// Complete set of parameters for an endpoint of a connection,
/// specified in the world frame.  It comprises two subsets of parameters:
/// those pertaining only to the xy ground-plane, and those pertaining to
/// out-of-plane aspects of an endpoint.
class Endpoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Endpoint)

  // Constructs an Endpoint with all zero parameters.
  Endpoint() = default;

  Endpoint(const EndpointXy& xy, const EndpointZ& z) : xy_(xy), z_(z) {}

  /// Returns an Endpoint with reversed direction.
  Endpoint reverse() const { return Endpoint(xy_.reverse(), z_.reverse()); }

  /// Returns the subset of parameters pertaining to the xy ground-plane.
  const EndpointXy& xy() const { return xy_; }

  /// Returns the subset of parameters pertaining to out-of-ground-plane
  /// aspects.
  const EndpointZ& z() const { return z_; }

 private:
  EndpointXy xy_;
  EndpointZ z_;
};

/// Streams a string representation of `endpoint` into `out`. Returns
/// `out`. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const Endpoint& endpoint);

/// Specification for path offset along a line.
///  * length: length of the line, which must be nonnegative.
class LineOffset {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LineOffset)

  LineOffset() = default;

  explicit LineOffset(double length) : length_(length) {
    DRAKE_DEMAND(length_ >= 0.);
  }

  double length() const { return length_; }

 private:
  double length_{};
};

/// Streams a string representation of `line_offset` into `out`. Returns `out`.
/// This method is provided for the purposes of debugging or text-logging.
/// It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LineOffset& line_offset);

/// Specification for path offset along a circular arc.
///  * radius: radius of the arc, which must be positive
///  * d_theta:  angle of arc segment (Δθ)
///    * d_theta > 0 is counterclockwise ('veer to left')
///    * d_theta < 0 is clockwise ('veer to right')
class ArcOffset {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArcOffset)

  /// Constructs an ArcOffset with all zero parameters.
  ArcOffset() = default;

  ArcOffset(double radius, double d_theta)
      : radius_(radius), d_theta_(d_theta) {
    DRAKE_DEMAND(radius_ > 0.);
  }

  double radius() const { return radius_; }

  double d_theta() const { return d_theta_; }

 private:
  double radius_{};
  double d_theta_{};
};

/// Streams a string representation of `arc_offset` into `out`. Returns `out`.
/// This method is provided for the purposes of debugging or text-logging.
/// It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const ArcOffset& arc_offset);

/// Representation of a reference path connecting two endpoints.
///
/// Upon building the RoadGeometry, a Connection yields a Segment
/// bearing multiple Lanes with offsets from the reference path. The
/// Segment will belong to its own Junction, unless the Connection was
/// grouped with other Connections into a Group.
///
/// Two connection geometries are supported: line and arc.  These
/// primitives determine the projection of the reference path onto the
/// (locally-flat) plane of the earth.  The out-of-plane shape of
/// the path will be determined by the EndpointZ (elevation) parameters
/// of the endpoints.
class Connection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Connection)

  /// Possible connection geometries:  line- or arc-segment.
  enum Type { kLine, kArc };

  /// Constructs a line-segment connection.
  ///
  /// Segment's reference curve begins at `start` and extends on the plane
  /// z=0 `line_length` distance with `start.xy().heading()` heading angle.
  /// `end_z` will be used to build elevation and superelevation information of
  /// the road.
  ///
  /// `line_length` must be non negative.
  ///
  /// Segments will contain `num_lanes` lanes, which must be greater than zero.
  /// First Lane centerline will be placed at `r0` distance from the reference
  /// curve. Each lane's width will be `lane_width`, which should be greater or
  /// equal to zero.
  ///
  /// `left_shoulder` and `right_shoulder` are extra spaces added to the right
  /// and left side of the first and last lanes of the Segment. They will be
  /// added to Segment's bounds and must be greater or equal to zero.
  ///
  /// `scale_length` constrains the maximum level of detail captured by the
  /// underlying RoadCurve.
  ///
  /// `linear_tolerance` and `computation_policy` set the efficiency vs. speed
  /// balance for computations in the underlying RoadCurve.
  Connection(const std::string& id, const Endpoint& start,
             const EndpointZ& end_z, int num_lanes, double r0,
             double lane_width, double left_shoulder, double right_shoulder,
             double line_length, double linear_tolerance, double scale_length,
             ComputationPolicy computation_policy);

  /// Constructs an arc-segment connection.
  ///
  /// Segment's reference curve begins at `start` and extends on the plane z=0
  /// with `arc_offset.radius()` and angle span of `arc_offset.d_theta()`.
  /// `end_z` will be used to build elevation and superelevation information of
  /// the road.
  ///
  /// `arc_offset.radius()` must be positive. `arc_offset.d_theta()` > 0
  /// indicates a counterclockwise arc from `start` with initial heading angle
  /// `start.heading()`.
  ///
  /// Segments will contain `num_lanes` lanes, which must be greater than zero.
  /// First Lane centerline will be placed at `r0` distance from the reference
  /// curve.  Each lane's width will be `lane_width`, which should be greater or
  /// equal to zero.
  ///
  /// `left_shoulder` and `right_shoulder` are extra spaces added to the right
  /// and left side of the first and last lanes of the Segment. They will be
  /// added to Segment's bounds and must be greater or equal to zero.
  ///
  /// `linear_tolerance` applies to all RoadCurve-level computations. It must be
  /// positive.
  ///
  /// `scale_length` constrains the maximum level of detail captured by the
  /// underlying RoadCurve. It must be positive.
  ///
  /// `computation_policy` sets the speed vs. accuracy for computations in the
  /// underlying RoadCurve.
  Connection(const std::string& id, const Endpoint& start,
             const EndpointZ& end_z, int num_lanes, double r0,
             double lane_width, double left_shoulder, double right_shoulder,
             const ArcOffset& arc_offset, double linear_tolerance,
             double scale_length, ComputationPolicy computation_policy);

  /// Returns the geometric type of the path.
  Type type() const { return type_; }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the parameters of the start point.
  const Endpoint& start() const { return start_; }

  /// Returns the parameters of the endpoint.
  const Endpoint& end() const { return end_; }

  /// Returns the length of the line (for line connections only).
  double line_length() const {
    DRAKE_DEMAND(type_ == kLine);
    return line_length_;
  }

  /// Returns the radius of the arc (for arc connections only).
  double radius() const {
    DRAKE_DEMAND(type_ == kArc);
    return radius_;
  }

  /// Returns the angle of the arc (for arc connections only).
  double d_theta() const {
    DRAKE_DEMAND(type_ == kArc);
    return d_theta_;
  }

  /// Returns the number of lanes the Segment will contain.
  int num_lanes() const { return num_lanes_; }

  /// Returns the lateral offset from the reference curve to the first Lane
  /// centerline.
  double r0() const { return r0_; }

  /// Returns lanes' width.
  double lane_width() const { return lane_width_; }

  /// Returns the left shoulder distance of the segment.
  double left_shoulder() const { return left_shoulder_; }

  /// Returns the right shoulder distance of the segment.
  double right_shoulder() const { return right_shoulder_; }

  /// Returns `lane_index` lane lateral distance to the reference curve.
  ///
  /// `lane_index` must be non-negative and smaller than the number of lanes of
  /// this connection.
  double lane_offset(int lane_index) const {
    DRAKE_DEMAND(lane_index >= 0 && lane_index < num_lanes_);
    return r0_ + lane_width_ * static_cast<double>(lane_index);
  }

  /// Returns the distance from the reference curve to the right extent of the
  /// Segment.
  double r_min() const { return r_min_; }

  /// Returns the distance from the reference curve to the left extent of the
  /// Segment.
  double r_max() const { return r_max_; }

  /// Returns the linear tolerance, in meters, that applies to RoadCurve
  /// instances as constructed by this Connection. Refer to RoadCurve class
  /// documentation for further details.
  double linear_tolerance() const { return linear_tolerance_; }

  /// Returns the scale length, in meters, that applies to RoadCurve instances
  /// as constructed by this Connection. Refer to RoadCurve class documentation
  /// for further details.
  double scale_length() const { return scale_length_; }

  /// Returns the computation policy that applies to RoadCurve instances as
  /// constructed by this Connection. Refer to RoadCurve class documentation
  /// for further details.
  ComputationPolicy computation_policy() const { return computation_policy_; }

  /// Returns an Endpoint describing the start of the `lane_index` lane.
  Endpoint LaneStart(int lane_index) const;

  /// Returns an Endpoint describing the end of the `lane_index` lane.
  Endpoint LaneEnd(int lane_index) const;

  /// Creates a RoadCurve that describes the reference curve of this Connection.
  std::unique_ptr<RoadCurve> CreateRoadCurve() const;

 private:
  const Type type_{};
  const std::string id_;
  const Endpoint start_{};
  Endpoint end_{};
  const int num_lanes_{};
  const double r0_{};
  const double lane_width_{};
  const double left_shoulder_{};
  const double right_shoulder_{};
  const double r_min_{};
  const double r_max_{};
  const double linear_tolerance_{};
  const double scale_length_{};
  const ComputationPolicy computation_policy_;
  std::unique_ptr<RoadCurve> road_curve_;
  // Bits specific to type_ == kLine:
  double line_length_{};
  // Bits specific to type_ == kArc:
  double radius_{};
  double d_theta_{};
  double theta0_{};
  double cx_{};
  double cy_{};
};

/// A group of Connections.
///
/// Upon building the RoadGeometry, a Group yields a Junction containing the
/// corresponding Segments specified by all the Connections in the Group.
class Group {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Group)

  /// Constructs an empty Group with the specified `id`.
  explicit Group(const std::string& id) : id_(id) {}

  /// Constructs a Group with `id`, populated by `connections`.
  ///
  /// `connections` must not contain duplicates.
  Group(const std::string& id,
        const std::vector<const Connection*>& connections)
      : id_(id) {
    for (const Connection* connection : connections) {
      Add(connection);
    }
  }

  /// Adds a Connection.
  ///
  /// `connection` must not already be added.
  void Add(const Connection* connection) {
    auto result = connection_set_.insert(connection);
    DRAKE_DEMAND(result.second);
    connection_vector_.push_back(connection);
  }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the grouped Connections.
  const std::vector<const Connection*>& connections() const {
    return connection_vector_;
  }

 private:
  std::string id_;
  std::unordered_set<const Connection*> connection_set_;
  std::vector<const Connection*> connection_vector_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
