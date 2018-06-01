#include "drake/automotive/maliput/multilane/builder.h"

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/branch_point.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/junction.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/road_geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out, const LaneLayout& lane_layout) {
  return out << "(left_shoulder: " << lane_layout.left_shoulder()
             << ", right_shoulder: " << lane_layout.right_shoulder()
             << ", num_lanes: " << lane_layout.num_lanes()
             << ", ref_lane: " << lane_layout.ref_lane()
             << ", ref_r0: " << lane_layout.ref_r0() << ")";
}

Builder::Builder(double lane_width, const api::HBounds& elevation_bounds,
                 double linear_tolerance, double angular_tolerance,
                 double scale_length, ComputationPolicy computation_policy)
    : lane_width_(lane_width),
      elevation_bounds_(elevation_bounds),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy) {
  DRAKE_DEMAND(lane_width_ >= 0.);
  DRAKE_DEMAND(linear_tolerance_ >= 0.);
  DRAKE_DEMAND(angular_tolerance_ >= 0.);
}

const Connection* Builder::Connect(const std::string& id,
                                   const LaneLayout& lane_layout,
                                   const StartReference::Spec& start_spec,
                                   const LineOffset& line_offset,
                                   const EndReference::Spec& end_spec) {
  // TODO(agalbachicar)    Once the API supports referencing to lanes, this
  //                       should be used to call the appropriate Builder
  //                       methods, r_ref will refer to any lane and will not be
  //                       r0.
  DRAKE_DEMAND(lane_layout.ref_lane() == 0);
  connections_.push_back(std::make_unique<Connection>(
      id, start_spec.endpoint(), end_spec.endpoint_z(), lane_layout.num_lanes(),
      lane_layout.ref_r0(), lane_width_, lane_layout.left_shoulder(),
      lane_layout.right_shoulder(), line_offset.length(), linear_tolerance_,
      scale_length_, computation_policy_));
  return connections_.back().get();
}

const Connection* Builder::Connect(const std::string& id,
                                   const LaneLayout& lane_layout,
                                   const StartReference::Spec& start_spec,
                                   const ArcOffset& arc_offset,
                                   const EndReference::Spec& end_spec) {
  // TODO(agalbachicar)    Once the API supports referencing to lanes, this
  //                       should be used to call the appropriate Builder
  //                       methods, r_ref will refer to any lane and will not be
  //                       r0.
  DRAKE_DEMAND(lane_layout.ref_lane() == 0);
  connections_.push_back(std::make_unique<Connection>(
      id, start_spec.endpoint(), end_spec.endpoint_z(), lane_layout.num_lanes(),
      lane_layout.ref_r0(), lane_width_, lane_layout.left_shoulder(),
      lane_layout.right_shoulder(), arc_offset, linear_tolerance_,
      scale_length_, computation_policy_));
  return connections_.back().get();
}

void Builder::SetDefaultBranch(const Connection* in, int in_lane_index,
                               const api::LaneEnd::Which in_end,
                               const Connection* out, int out_lane_index,
                               const api::LaneEnd::Which out_end) {
  default_branches_.push_back(
      {in, in_lane_index, in_end, out, out_lane_index, out_end});
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
// Determine the heading (in xy-plane) along the centerline when
// travelling towards/into the lane, from the specified end.
double HeadingIntoLane(const api::Lane* const lane,
                       const api::LaneEnd::Which end) {
  switch (end) {
    case api::LaneEnd::kStart: {
      return lane->GetOrientation({0., 0., 0.}).yaw();
    }
    case api::LaneEnd::kFinish: {
      return lane->GetOrientation({lane->length(), 0., 0.}).yaw() + M_PI;
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
      api::BranchPointId{
        "bp:" + std::to_string(road_geometry->num_branch_points())});
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

std::vector<Lane*> Builder::BuildConnection(
    const Connection* const conn, Junction* const junction,
    RoadGeometry* const road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const {
  Segment* segment = junction->NewSegment(
      api::SegmentId{std::string("s:") + conn->id()}, conn->CreateRoadCurve(),
      conn->r_min(), conn->r_max(), elevation_bounds_);
  std::vector<Lane*> lanes;
  for (int i = 0; i < conn->num_lanes(); i++) {
    Lane* lane =
        segment->NewLane(api::LaneId{std::string("l:") + conn->id() +
                                     std::string("_") + std::to_string(i)},
                         conn->lane_offset(i),
                         {-conn->lane_width() / 2., conn->lane_width() / 2.});
    AttachBranchPoint(conn->LaneStart(i), lane, api::LaneEnd::kStart,
                      road_geometry, bp_map);
    AttachBranchPoint(conn->LaneEnd(i), lane, api::LaneEnd::kFinish,
                      road_geometry, bp_map);
    lanes.push_back(lane);
  }
  return lanes;
}

std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) const {
  auto road_geometry = std::make_unique<RoadGeometry>(
      id, linear_tolerance_, angular_tolerance_);
  std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder> bp_map(
      (EndpointFuzzyOrder(linear_tolerance_)));
  std::map<const Connection*, std::vector<Lane*>> lane_map;
  std::map<const Connection*, bool> connection_was_built;

  for (const std::unique_ptr<Connection>& connection : connections_) {
    connection_was_built.emplace(connection.get(), false);
  }

  for (const std::unique_ptr<Group>& group : groups_) {
    Junction* junction =
        road_geometry->NewJunction(
            api::JunctionId{std::string("j:") + group->id()});
    drake::log()->debug("junction: {}", junction->id().string());
    for (auto& connection : group->connections()) {
      drake::log()->debug("connection: {}", connection->id());
      DRAKE_DEMAND(!connection_was_built[connection]);
      lane_map[connection] = BuildConnection(
          connection, junction, road_geometry.get(), &bp_map);
      connection_was_built[connection] = true;
    }
  }

  for (const std::unique_ptr<Connection>& connection : connections_) {
    if (connection_was_built[connection.get()]) {
      continue;
    }
    Junction* junction =
        road_geometry->NewJunction(
            api::JunctionId{std::string("j:") + connection->id()});
    drake::log()->debug("junction: {}", junction->id().string());
    drake::log()->debug("connection: {}", connection->id());
    lane_map[connection.get()] =
        BuildConnection(connection.get(),
                        junction, road_geometry.get(), &bp_map);
  }

  for (const DefaultBranch& def : default_branches_) {
    Lane* in_lane = lane_map[def.in][def.in_lane_index];
    Lane* out_lane = lane_map[def.out][def.out_lane_index];
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

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
