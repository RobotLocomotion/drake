#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace monolane {

Builder::XYZPointFuzzyOrder::
XYZPointFuzzyOrder(const double linear_tolerance,
                   const double angular_tolerance)
        : pos_pre_(linear_tolerance),
          ori_pre_(angular_tolerance) {}


Builder::Builder(const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds,
                 const double linear_tolerance,
                 const double angular_tolerance)
    : lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance) {}

const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const double length,
    const ZPoint& z_end) {

  const XYZPoint end(
      XYPoint(start.xy.x + (length * std::cos(start.xy.heading)),
              start.xy.y + (length * std::sin(start.xy.heading)),
              start.xy.heading),
      z_end);
  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kLine, id,
      start, end));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const double length,
    const XYZPoint& explicit_end) {
  // TODO(maddog)  Validate length/heading vs forced_end.
  DRAKE_DEMAND(length);
  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kLine, id,
      start, explicit_end));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const ArcOffset& arc,
    const ZPoint& z_end) {
  const double alpha = start.xy.heading;
  const double theta0 = alpha - std::copysign(M_PI / 2., arc.d_theta);
  const double theta1 = theta0 + arc.d_theta;

  const double cx = start.xy.x - (arc.radius * std::cos(theta0));
  const double cy = start.xy.y - (arc.radius * std::sin(theta0));

  const XYZPoint end(XYPoint(cx + (arc.radius * std::cos(theta1)),
                             cy + (arc.radius * std::sin(theta1)),
                             alpha + arc.d_theta),
                     z_end);

  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kArc, id,
      start, end, cx, cy, arc.radius, arc.d_theta));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const ArcOffset& arc,
    const XYZPoint& explicit_end) {
  const double alpha = start.xy.heading;
  const double theta0 = alpha - std::copysign(M_PI / 2., arc.d_theta);

  const double cx = start.xy.x - (arc.radius * std::cos(theta0));
  const double cy = start.xy.y - (arc.radius * std::sin(theta0));

  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kArc, id,
      start, explicit_end, cx, cy, arc.radius, arc.d_theta));
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
CubicPolynomial MakeCubic(const double dX, const double Y0, const double dY,
                          const double Ydot0, const double Ydot1) {
  return CubicPolynomial(Y0 / dX,
                         Ydot0,
                         (3. * dY / dX) - (2. * Ydot0) - Ydot1,
                         Ydot0 + Ydot1 - (2. * dY / dX));
}

/// Determine the heading (in xy-plane) along the centerline when
/// travelling towards/into the lane, from the specified end.
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
    const XYZPoint& point,
    RoadGeometry* rg,
    std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* bp_map) const {
  auto ibp = bp_map->find(point);
  if (ibp != bp_map->end()) {
    return ibp->second;
  }
  // TODO(maddog) Generate a real id.
  BranchPoint* bp = rg->NewBranchPoint(
      {"bp:" + std::to_string(rg->num_branch_points())});
  auto result = bp_map->emplace(point, bp);
  DRAKE_DEMAND(result.second);
  return bp;
}


void Builder::AttachBranchPoint(
    const XYZPoint& point, Lane* const lane, const api::LaneEnd::Which end,
    RoadGeometry* rg,
    std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* bp_map) const {
  BranchPoint* bp = FindOrCreateBranchPoint(point, rg, bp_map);
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
  // If so, just stick it on Side-A.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (bp->GetASide()->size() == 0) {
    bp->AddABranch({lane, end});
    return;
  }
  // Otherwise, assess if this new lane-end is parallel or anti-parallel to
  // the first lane-end.  Parallel: go to same, A-side; anti-parallel:
  // other, B-side.  Do this by examining dot-product of heading vectors
  // (rather than goofing around with cyclic angle arithmetic).
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
    const Connection* const cnx,
    Junction* const junction,
    RoadGeometry* const rg,
    std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* const bp_map) const {
  Segment* segment = junction->NewSegment({std::string("s:") + cnx->id()});
  Lane* lane;
  api::LaneId lane_id({std::string("l:") + cnx->id()});

  switch (cnx->type()) {
    case Connection::kLine: {
      const V2 xy0(cnx->start().xy.x,
                   cnx->start().xy.y);
      const V2 dxy(cnx->end().xy.x - xy0.x,
                   cnx->end().xy.y - xy0.y);
      const CubicPolynomial elevation(MakeCubic(
          dxy.length(),
          cnx->start().z.z,
          cnx->end().z.z - cnx->start().z.z,
          cnx->start().z.zdot,
          cnx->end().z.zdot));
      const CubicPolynomial superelevation(MakeCubic(
          dxy.length(),
          cnx->start().z.theta,
          cnx->end().z.theta - cnx->start().z.theta,
          cnx->start().z.thetadot,
          cnx->end().z.thetadot));

      lane = segment->NewLineLane(lane_id,
                                  xy0, dxy,
                                  lane_bounds_, driveable_bounds_,
                                  elevation, superelevation);
      break;
    }
    case Connection::kArc: {
      const V2 center(cnx->cx(), cnx->cy());
      const double radius = cnx->radius();
      const double theta0 = std::atan2(cnx->start().xy.y - center.y,
                                       cnx->start().xy.x - center.x);
      // TODO(maddog) Uh, which direction?  Info lost since cnx constructed!
      //              ...and deal with wrap-arounds, too.
      const double d_theta = cnx->d_theta();
      const double arc_length = radius * std::abs(d_theta);
      const CubicPolynomial elevation(MakeCubic(
          arc_length,
          cnx->start().z.z,
          cnx->end().z.z - cnx->start().z.z,
          cnx->start().z.zdot,
          cnx->end().z.zdot));
      const CubicPolynomial superelevation(MakeCubic(
          arc_length,
          cnx->start().z.theta,
          cnx->end().z.theta - cnx->start().z.theta,
          cnx->start().z.thetadot,
          cnx->end().z.thetadot));

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

  AttachBranchPoint(cnx->start(), lane, api::LaneEnd::kStart, rg, bp_map);
  AttachBranchPoint(cnx->end(), lane, api::LaneEnd::kFinish, rg, bp_map);
  return lane;
}


std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) const {
  auto rg = std::make_unique<RoadGeometry>(
      id, linear_tolerance_, angular_tolerance_);
  std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder> bp_map(
      XYZPointFuzzyOrder(linear_tolerance_, angular_tolerance_));
  std::map<const Connection*, Lane*> lane_map;
  std::set<const Connection*> remaining_connections;

  for (auto& cnx : connections_) {
    remaining_connections.insert(cnx.get());
  }

  for (auto& grp : groups_) {
    Junction* junction = rg->NewJunction({std::string("j:") + grp->id()});
    std::cout << "jnx: " << junction->id().id << std::endl;
    for (auto& cnx : grp->connections()) {
      std::cout << "cnx: " << cnx->id() << std::endl;
      remaining_connections.erase(cnx);
      lane_map[cnx] = BuildConnection(cnx, junction, rg.get(), &bp_map);
    }
  }

  for (auto& cnx : remaining_connections) {
    Junction* junction = rg->NewJunction({std::string("j:") + cnx->id()});
    std::cout << "jnx: " << junction->id().id << std::endl;
    std::cout << "cnx: " << cnx->id() << std::endl;
    lane_map[cnx] = BuildConnection(cnx, junction, rg.get(), &bp_map);
  }

  for (auto& def : default_branches_) {
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
  std::vector<std::string> failures = rg->CheckInvariants();
  for (const auto& s: failures) {
    std::cerr << s << std::endl;
  }
  DRAKE_DEMAND(failures.size() == 0);

  return std::move(rg);
}


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
