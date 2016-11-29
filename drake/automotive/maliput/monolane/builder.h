#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {

namespace api {
class RoadGeometry;
}

namespace monolane {

/// Builder for monolane road networks.
///
/// monolane is a simple, highly-constrained network:
///  - single lane per segment.
///  - constant lane_bounds and driveable_bounds, same for all lanes
///  - only linear and constant-curvature-arc primitives in XY-plane
///  - cubic polynomials (parameterized on XY-arc-length) for elevation
///    and superelevation


struct XYPoint {
  XYPoint() {}

  XYPoint(double ax, double ay, double aheading)
      :x(ax), y(ay), heading(aheading) {}


  XYPoint reverse() const {
    return XYPoint(x, y,
                   std::atan2(-std::sin(heading), -std::cos(heading)));
  }

  double x{};
  double y{};
  double heading{};  // radians, zero == x direction
};


struct ZPoint {
  ZPoint reverse() const {
    return {z, -zdot, -theta, -thetadot};
  }

  double z{};
  double zdot{};

  double theta{};  // superelevation
  double thetadot{};
};


struct XYZPoint {
  XYZPoint() {}

  XYZPoint(const XYPoint& axy, const ZPoint& az) : xy(axy), z(az) {}

  XYZPoint reverse() const {
    return {xy.reverse(), z.reverse()};
  }

  XYPoint xy{};
  ZPoint z{};
};


// radius_ must be non-negative.
// d_theta_ > 0. is counterclockwise ('veer to left').
// d_theta_ < 0. is clockwise ('veer to right').
struct ArcOffset {
  ArcOffset() {}

  ArcOffset(const double aradius, const double ad_theta)
      : radius(aradius), d_theta(ad_theta) {
    DRAKE_DEMAND(radius > 0.);
  }

  double radius{};
  double d_theta{};
};


// TODO(maddog)  Make non-copyable.
class Connection {
 public:
  enum Type { kLine, kArc };

  Connection(const Type type, const std::string& id,
             const XYZPoint& start, const XYZPoint& end)
      : type_(type), id_(id), start_(start), end_(end) {}

  Connection(const Type type, const std::string& id,
             const XYZPoint& start, const XYZPoint& end,
             double cx, double cy, double radius, double d_theta)
      : type_(type), id_(id), start_(start), end_(end),
        cx_(cx), cy_(cy), radius_(radius), d_theta_(d_theta) {}

  Type type() const { return type_; }

  const std::string& id() const { return id_; }

  const XYZPoint& start() const { return start_; }

  const XYZPoint& end() const { return end_; }

  double cx() const { return cx_; }

  double cy() const { return cy_; }

  double radius() const { return radius_; }

  double d_theta() const { return d_theta_; }

  ~Connection() {}

 private:
  Type type_{};
  std::string id_;
  XYZPoint start_;
  XYZPoint end_;

  // Bits specific to type_ == kArc:
  double cx_{};
  double cy_{};
  double radius_{};
  double d_theta_{};
};


// TODO(maddog) make noncopyable
class Group {
 public:
  explicit Group(const std::string& id) : id_(id) {}

  Group(const std::string& id,
        const std::vector<const Connection*>& connections)
      : id_(id), connections_(connections) {}

  void Add(const Connection* connection) {
    // TODO(maddog) Ensure uniqueness.
    connections_.push_back(connection);
  }

  const std::string& id() const { return id_; }

  const std::vector<const Connection*>& connections() const {
    return connections_;
  }

 private:
  std::string id_;
  std::vector<const Connection*> connections_;
};


// TODO(maddog) make noncopyable
class Builder {
 public:
  Builder(const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const double linear_tolerance,
          const double angular_tolerance);

  // TODO(maddog) Provide for explicit branch-point siding of ends...
  //              e.g. to allow 3-way intersection.
  // TODO(maddog) Handle setting default branch-points.

  // Connect a start point to an end point relative to the start,
  // with a linear displacement.
  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const double length,
      const ZPoint& z_end);

  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const double length,
      const XYZPoint& explicit_end);

  // Connect a start point to an end point relative to the start,
  // with an arc displacement.
  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const ArcOffset& arc,
      const ZPoint& z_end);

  // Hackery until generic Connect(start, end) implemented.
  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const ArcOffset& arc,
      const XYZPoint& explicit_end);

  void SetDefaultBranch(
      const Connection* in, const api::LaneEnd::Which in_end,
      const Connection* out, const api::LaneEnd::Which out_end);

  Group* MakeGroup(const std::string& id);

  Group* MakeGroup(const std::string& id,
                   const std::vector<const Connection*>& connections);

  // Produce a RoadGeometry.
  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const;

 private:
  class XYZPointFuzzyOrder {
   public:
    XYZPointFuzzyOrder(const double linear_tolerance,
                       const double angular_tolerance);

    // TODO(maddog) This should perhaps incorporate heading/anti-heading.
    // TODO(maddog) This should perhaps incorporate linear_tolerance.
    bool operator()(const XYZPoint& lhs, const XYZPoint& rhs) const {
      auto as_tuple = [](const XYZPoint& p) {
        return std::tie(p.xy.x, p.xy.y, p.z.z);
      };
      return as_tuple(lhs) < as_tuple(rhs);
    }

   private:
    const double pos_pre_{};
    const double ori_pre_{};
  };

  struct DefaultBranch {
    DefaultBranch() {}

    DefaultBranch(
        const Connection* ain, const api::LaneEnd::Which ain_end,
        const Connection* aout, const api::LaneEnd::Which aout_end)
        : in(ain), in_end(ain_end), out(aout), out_end(aout_end) {}

    const Connection* in{};
    api::LaneEnd::Which in_end{};
    const Connection* out{};
    api::LaneEnd::Which out_end{};
  };

  Lane* BuildConnection(
      const Connection* const cnx,
      Junction* const junction,
      RoadGeometry* const rg,
      std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* const bp_map) const;

  BranchPoint* FindOrCreateBranchPoint(
      const XYZPoint& point,
      RoadGeometry* rg,
      std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* const bp_map) const;

  void AttachBranchPoint(
      const XYZPoint& point, Lane* const lane, const api::LaneEnd::Which end,
      RoadGeometry* rg,
      std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* bp_map) const;

  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::vector<DefaultBranch> default_branches_;
  std::vector<std::unique_ptr<Group>> groups_;
};



}  // namespace monolane
}  // namespace maliput
}  // namespace drake
