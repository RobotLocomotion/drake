#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace monolane {

Builder::Builder(const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds,
                 const double linear_tolerance,
                 const double angular_tolerance)
    : lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance) {
  DRAKE_DEMAND(lane_bounds_.r_min >= driveable_bounds_.r_min);
  DRAKE_DEMAND(lane_bounds_.r_max <= driveable_bounds_.r_max);
}


const Connection* Builder::Connect(
    const std::string& id,
    const Endpoint& start,
    const double length,
    const EndpointZ& z_end) {

  const Endpoint end(
      EndpointXy(start.xy().x() + (length * std::cos(start.xy().heading())),
                 start.xy().y() + (length * std::sin(start.xy().heading())),
                 start.xy().heading()),
      z_end);
  connections_.push_back(std::make_unique<Connection>(id, start, end));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const Endpoint& start,
    const ArcOffset& arc,
    const EndpointZ& z_end) {
  const double alpha = start.xy().heading();
  const double theta0 = alpha - std::copysign(M_PI / 2., arc.d_theta());
  const double theta1 = theta0 + arc.d_theta();

  const double cx = start.xy().x() - (arc.radius() * std::cos(theta0));
  const double cy = start.xy().y() - (arc.radius() * std::sin(theta0));

  const Endpoint end(EndpointXy(cx + (arc.radius() * std::cos(theta1)),
                                cy + (arc.radius() * std::sin(theta1)),
                                alpha + arc.d_theta()),
                     z_end);

  connections_.push_back(std::make_unique<Connection>(
      id, start, end, cx, cy, arc.radius(), arc.d_theta()));
  return connections_.back().get();
}


void Builder::SetDefaultBranch(
    const Connection* in, const api::LaneEnd::Which in_end,
    const Connection* out, const api::LaneEnd::Which out_end) {
  default_branches_.push_back({in, in_end, out, out_end});
}


Group* Builder::MakeGroup(const std::string& id) {
  groups_.push_back(std::make_unique<Group>(id));
  return groups_.back().get();
}


Group* Builder::MakeGroup(const std::string& id,
                          const std::vector<const Connection*>& connections) {
  groups_.push_back(std::make_unique<Group>(id, connections));
  return groups_.back().get();
}


namespace {
// Construct a CubicPolynomial such that:
//    f(0) = Y0 / dX           f'(0) = Ydot0
//    f(1) = (Y0 + dY) / dX    f'(1) = Ydot1
//
// This is equivalent to taking a cubic polynomial g such that:
//    g(0) = Y0          g'(0) = Ydot0
//    g(dX) = Y0 + dY    g'(1) = Ydot1
// and isotropically scaling it (scale both axes) by a factor of 1/dX
CubicPolynomial MakeCubic(double dX, double Y0, double dY,
                          double Ydot0, double Ydot1) {
  return CubicPolynomial(Y0 / dX,
                         Ydot0,
                         (3. * dY / dX) - (2. * Ydot0) - Ydot1,
                         Ydot0 + Ydot1 - (2. * dY / dX));
}

// Determine the heading (in xy-plane) along the centerline when
// travelling towards/into the lane, from the specified end.
double HeadingIntoLane(const api::Lane* const lane,
                       const api::LaneEnd::Which end) {
  switch (end) {
    case api::LaneEnd::kStart: {
      return lane->GetOrientation({0., 0., 0.}).yaw;
    }
    case api::LaneEnd::kFinish: {
      return lane->GetOrientation({lane->length(), 0., 0.}).yaw + M_PI;
    }
    default: { DRAKE_ABORT(); }
  }
}
}  // namespace



BranchPoint* Builder::FindOrCreateBranchPoint(
    const Endpoint& point,
    RoadGeometry* road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const {
  auto ibp = bp_map->find(point);
  if (ibp != bp_map->end()) {
    return ibp->second;
  }
  // TODO(maddog@tri.global) Generate a more meaningful id (user-specified?)
  BranchPoint* bp = road_geometry->NewBranchPoint(
      {"bp:" + std::to_string(road_geometry->num_branch_points())});
  auto result = bp_map->emplace(point, bp);
  DRAKE_DEMAND(result.second);
  return bp;
}


void Builder::AttachBranchPoint(
    const Endpoint& point, Lane* const lane, const api::LaneEnd::Which end,
    RoadGeometry* road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const {
  BranchPoint* bp = FindOrCreateBranchPoint(point, road_geometry, bp_map);
  // Tell the lane about its branch-point.
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(bp);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(bp);
      break;
    }
    default: { DRAKE_ABORT(); }
  }
  // Now, tell the branch-point about the lane.
  //
  // Is this the first lane-end added to the branch-point?
  // If so, just stick it on A-Side.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (bp->GetASide()->size() == 0) {
    bp->AddABranch({lane, end});
    return;
  }
  // Otherwise, assess if this new lane-end is parallel or anti-parallel to
  // the first lane-end.  Parallel: go to same, A-side; anti-parallel:
  // other, B-side.  Do this by examining the dot-product of the heading
  // vectors (rather than goofing around with cyclic angle arithmetic).
  const double new_h = HeadingIntoLane(lane, end);
  const api::LaneEnd old_le = bp->GetASide()->get(0);
  const double old_h = HeadingIntoLane(old_le.lane, old_le.end);
  if (((std::cos(new_h) * std::cos(old_h)) +
       (std::sin(new_h) * std::sin(old_h))) > 0.) {
    bp->AddABranch({lane, end});
  } else {
    bp->AddBBranch({lane, end});
  }
}


Lane* Builder::BuildConnection(
    const Connection* const conn,
    Junction* const junction,
    RoadGeometry* const road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const {
  Segment* segment = junction->NewSegment({std::string("s:") + conn->id()});
  Lane* lane{};
  api::LaneId lane_id{std::string("l:") + conn->id()};

  switch (conn->type()) {
    case Connection::kLine: {
      const V2 xy0(conn->start().xy().x(),
                   conn->start().xy().y());
      const V2 dxy(conn->end().xy().x() - xy0.x(),
                   conn->end().xy().y() - xy0.y());
      const CubicPolynomial elevation(MakeCubic(
          dxy.norm(),
          conn->start().z().z(),
          conn->end().z().z() - conn->start().z().z(),
          conn->start().z().z_dot(),
          conn->end().z().z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          dxy.norm(),
          conn->start().z().theta(),
          conn->end().z().theta() - conn->start().z().theta(),
          conn->start().z().theta_dot(),
          conn->end().z().theta_dot()));

      lane = segment->NewLineLane(lane_id,
                                  xy0, dxy,
                                  lane_bounds_, driveable_bounds_,
                                  elevation, superelevation);
      break;
    }
    case Connection::kArc: {
      const V2 center(conn->cx(), conn->cy());
      const double radius = conn->radius();
      const double theta0 = std::atan2(conn->start().xy().y() - center.y(),
                                       conn->start().xy().x() - center.x());
      const double d_theta = conn->d_theta();
      const double arc_length = radius * std::abs(d_theta);
      const CubicPolynomial elevation(MakeCubic(
          arc_length,
          conn->start().z().z(),
          conn->end().z().z() - conn->start().z().z(),
          conn->start().z().z_dot(),
          conn->end().z().z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          arc_length,
          conn->start().z().theta(),
          conn->end().z().theta() - conn->start().z().theta(),
          conn->start().z().theta_dot(),
          conn->end().z().theta_dot()));

      lane = segment->NewArcLane(lane_id,
                                 center, radius, theta0, d_theta,
                                 lane_bounds_, driveable_bounds_,
                                 elevation, superelevation);
      break;
    }
    default: {
      DRAKE_ABORT();
    }
  }

  AttachBranchPoint(
      conn->start(), lane, api::LaneEnd::kStart, road_geometry, bp_map);
  AttachBranchPoint(
      conn->end(), lane, api::LaneEnd::kFinish, road_geometry, bp_map);
  return lane;
}


std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) const {
  auto road_geometry = std::make_unique<RoadGeometry>(
      id, linear_tolerance_, angular_tolerance_);
  std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder> bp_map(
      (EndpointFuzzyOrder(linear_tolerance_)));
  std::map<const Connection*, Lane*> lane_map;
  std::set<const Connection*> remaining_connections;

  for (const std::unique_ptr<Connection>& connection : connections_) {
    remaining_connections.insert(connection.get());
  }

  for (const std::unique_ptr<Group>& group : groups_) {
    Junction* junction =
        road_geometry->NewJunction({std::string("j:") + group->id()});
    drake::log()->debug("junction: {}", junction->id().id);
    for (auto& connection : group->connections()) {
      drake::log()->debug("connection: {}", connection->id());
      // Remove connection from remaining_connections, and ensure that it
      // was indeed in there.
      DRAKE_DEMAND(remaining_connections.erase(connection) == 1);
      lane_map[connection] = BuildConnection(
          connection, junction, road_geometry.get(), &bp_map);
    }
  }

  for (const Connection* const connection : remaining_connections) {
    Junction* junction =
        road_geometry->NewJunction({std::string("j:") + connection->id()});
    drake::log()->debug("junction: {}", junction->id().id);
    drake::log()->debug("connection: {}", connection->id());
    lane_map[connection] =
        BuildConnection(connection, junction, road_geometry.get(), &bp_map);
  }

  for (const DefaultBranch& def : default_branches_) {
    Lane* in_lane = lane_map[def.in];
    Lane* out_lane = lane_map[def.out];
    DRAKE_DEMAND((def.in_end == api::LaneEnd::kStart) ||
                 (def.in_end == api::LaneEnd::kFinish));
    ((def.in_end == api::LaneEnd::kStart) ?
     in_lane->start_bp() : in_lane->end_bp())
        ->SetDefault({in_lane, def.in_end},
                     {out_lane, def.out_end});
  }

  // Make sure we didn't screw up!
  std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const auto& s : failures) {
    drake::log()->error(s);
  }
  DRAKE_DEMAND(failures.size() == 0);

  return std::move(road_geometry);
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
