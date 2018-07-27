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

std::ostream& operator<<(std::ostream& out,
                         const StartReference::Spec& start_spec) {
  return out << "(endpoint: " << start_spec.endpoint() << ")";
}

std::ostream& operator<<(std::ostream& out, const StartLane::Spec& start_spec) {
  return out << "(endpoint: " << start_spec.endpoint()
             << ", lane_id: " << start_spec.lane_id() << ")";
}

std::ostream& operator<<(std::ostream& out,
                         const EndReference::Spec& end_spec) {
  return out << "(endpoint_z: " << end_spec.endpoint_z() << ")";
}

std::ostream& operator<<(std::ostream& out, const EndLane::Spec& end_spec) {
  return out << "(endpoint_z: " << end_spec.endpoint_z()
             << ", lane_id: " << end_spec.lane_id() << ")";
}

std::ostream& operator<<(std::ostream& out, const LaneLayout& lane_layout) {
  return out << "(left_shoulder: " << lane_layout.left_shoulder()
             << ", right_shoulder: " << lane_layout.right_shoulder()
             << ", num_lanes: " << lane_layout.num_lanes()
             << ", ref_lane: " << lane_layout.ref_lane()
             << ", ref_r0: " << lane_layout.ref_r0() << ")";
}

Builder::Builder(double lane_width, const api::HBounds& elevation_bounds,
                 double linear_tolerance, double angular_tolerance,
                 double scale_length, ComputationPolicy computation_policy,
                 std::unique_ptr<GroupFactoryBase> group_factory)
    : lane_width_(lane_width),
      elevation_bounds_(elevation_bounds),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy),
      group_factory_(std::move(group_factory)) {
  DRAKE_DEMAND(lane_width_ >= 0.);
  DRAKE_DEMAND(linear_tolerance_ >= 0.);
  DRAKE_DEMAND(angular_tolerance_ >= 0.);
  DRAKE_DEMAND(group_factory_ != nullptr);
}

namespace {
// Let theta_dot_A, z_dot_A, K_A be the rate of change in superelevation with
// respect to arc length of the reference path, grade (rate of change of
// elevation with respect to arc length of the reference path) and the
// curvature at one end point of connection's curve A. And let theta_dot_B,
// z_dot_B, K_B be the same values at one end point of connection's curve
// B. Then, if we want to connect both curves at their respective end points A
// and B, making:
//
// theta_dot_A - (K_A * sin(-atan(z_dot_A))) = 0
// theta_dot_B - (K_B * sin(-atan(z_dot_B))) = 0
//
// It is enough to make the connection joint be G1. Given that, `curvature` is
// K and `endpointz` is z_dot. `endpointz.theta_dot` is computed as:
//
// theta_dot = K * sin(-atan(z_dot))
void ComputeContinuityConstraint(double curvature, EndpointZ* endpointz) {
  DRAKE_DEMAND(endpointz != nullptr);
  endpointz->get_mutable_theta_dot() =
      curvature * std::sin(-std::atan(endpointz->z_dot()));
}
}  // namespace

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

  const double curvature{0.};
  Endpoint start_endpoint = start_spec.endpoint();
  if (!start_endpoint.z().theta_dot()) {
    ComputeContinuityConstraint(curvature, &(start_endpoint.get_mutable_z()));
  }
  EndpointZ end_endpoint_z = end_spec.endpoint_z();
  if (!end_endpoint_z.theta_dot()) {
    ComputeContinuityConstraint(curvature, &end_endpoint_z);
  }
  connections_.push_back(std::make_unique<Connection>(
      id, start_endpoint, end_endpoint_z, lane_layout.num_lanes(),
      lane_layout.ref_r0(), lane_width_, lane_layout.left_shoulder(),
      lane_layout.right_shoulder(), line_offset, linear_tolerance_,
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

  const double curvature =
      std::copysign(1., arc_offset.d_theta()) / arc_offset.radius();

  Endpoint start_endpoint = start_spec.endpoint();
  if (!start_endpoint.z().theta_dot()) {
    ComputeContinuityConstraint(curvature, &(start_endpoint.get_mutable_z()));
  }
  EndpointZ end_endpoint_z = end_spec.endpoint_z();
  if (!end_endpoint_z.theta_dot()) {
    ComputeContinuityConstraint(curvature, &end_endpoint_z);
  }
  connections_.push_back(std::make_unique<Connection>(
      id, start_endpoint, end_endpoint_z, lane_layout.num_lanes(),
      lane_layout.ref_r0(), lane_width_, lane_layout.left_shoulder(),
      lane_layout.right_shoulder(), arc_offset, linear_tolerance_,
      scale_length_, computation_policy_));
  return connections_.back().get();
}

const Connection* Builder::Connect(const std::string& id,
                                   const LaneLayout& lane_layout,
                                   const StartLane::Spec& start_spec,
                                   const LineOffset& line_offset,
                                   const EndLane::Spec& end_spec) {
  DRAKE_DEMAND(start_spec.lane_id() >= 0 &&
               start_spec.lane_id() < lane_layout.num_lanes());
  DRAKE_DEMAND(end_spec.lane_id() >= 0 &&
               end_spec.lane_id() < lane_layout.num_lanes());

  const double curvature{0.};

  // Gets the initial heading and superelevation.
  const double start_heading = start_spec.endpoint().xy().heading();
  const double start_superelevation = start_spec.endpoint().z().theta();
  const Vector3<double> start_lane_position(start_spec.endpoint().xy().x(),
                                            start_spec.endpoint().xy().y(),
                                            start_spec.endpoint().z().z());
  const double start_lane_offset =
      -lane_layout.ref_r0() +
      lane_width_ *
          static_cast<double>(start_spec.lane_id() - lane_layout.ref_lane());
  // Computes the displacement vector from lane point to the reference curve
  // point.
  const Vector3<double> start_lane_to_ref(
      -start_lane_offset * std::cos(start_superelevation) *
          std::sin(start_heading),
      start_lane_offset * std::cos(start_superelevation) *
          std::cos(start_heading),
      start_lane_offset * std::sin(start_superelevation));
  const Vector3<double> start_reference_position =
      start_lane_position - start_lane_to_ref;
  // Assigns the start endpoint.
  Endpoint start{
      EndpointXy(start_reference_position.x(), start_reference_position.y(),
                 start_heading),
      EndpointZ(start_reference_position.z(), start_spec.endpoint().z().z_dot(),
                start_spec.endpoint().z().theta(), {})};
  ComputeContinuityConstraint(curvature, &(start.get_mutable_z()));

  // Computes the r distance from the end lane to the reference curve.
  const double end_lane_offset =
      -lane_layout.ref_r0() +
      lane_width_ *
          static_cast<double>(end_spec.lane_id() - lane_layout.ref_lane());
  // Computes the end endpoint.
  const double z_end =
      end_spec.endpoint_z().z() -
      end_lane_offset * std::sin(end_spec.endpoint_z().theta());
  EndpointZ end_z{
      z_end, end_spec.endpoint_z().z_dot(), end_spec.endpoint_z().theta(), {}};
  ComputeContinuityConstraint(curvature, &end_z);

  connections_.push_back(std::make_unique<Connection>(
      id, start, end_z, lane_layout.num_lanes(),
      -lane_layout.ref_r0() -
          static_cast<double>(lane_layout.ref_lane()) * lane_width_,
      lane_width_, lane_layout.left_shoulder(), lane_layout.right_shoulder(),
      line_offset, linear_tolerance_, scale_length_, computation_policy_));
  return connections_.back().get();
}

const Connection* Builder::Connect(const std::string& id,
                                   const LaneLayout& lane_layout,
                                   const StartLane::Spec& start_spec,
                                   const ArcOffset& arc_offset,
                                   const EndLane::Spec& end_spec) {
  DRAKE_DEMAND(start_spec.lane_id() >= 0 &&
               start_spec.lane_id() < lane_layout.num_lanes());
  DRAKE_DEMAND(end_spec.lane_id() >= 0 &&
               end_spec.lane_id() < lane_layout.num_lanes());

  const double curvature =
      std::copysign(1., arc_offset.d_theta()) / arc_offset.radius();

  // Fills arc related parameters, computes end Endpoint and creates the
  // RoadCurve.
  const double theta0 = start_spec.endpoint().xy().heading() -
                        std::copysign(M_PI / 2., arc_offset.d_theta());
  // Gets the initial heading and superelevation.
  const double start_superelevation = start_spec.endpoint().z().theta();

  // Computes the center.
  const double start_lane_offset =
      -lane_layout.ref_r0() +
      lane_width_ *
          static_cast<double>(start_spec.lane_id() - lane_layout.ref_lane());
  const double start_lane_radius = arc_offset.radius() -
                                   start_lane_offset *
                                       std::cos(start_superelevation) *
                                       std::copysign(1., arc_offset.d_theta());
  const double cx =
      start_spec.endpoint().xy().x() - std::cos(theta0) * start_lane_radius;
  const double cy =
      start_spec.endpoint().xy().y() - std::sin(theta0) * start_lane_radius;
  const double start_z_dot = start_spec.endpoint().z().z_dot() *
                             start_lane_radius / arc_offset.radius();
  // Assigns the start endpoint.
  Endpoint start{
      EndpointXy(cx + arc_offset.radius() * std::cos(theta0),
                 cy + arc_offset.radius() * std::sin(theta0),
                 start_spec.endpoint().xy().heading()),
      EndpointZ(start_spec.endpoint().z().z() -
                    start_lane_offset * std::sin(start_superelevation) *
                        std::copysign(1., arc_offset.d_theta()),
                start_z_dot, start_spec.endpoint().z().theta(), {})};
  ComputeContinuityConstraint(curvature, &(start.get_mutable_z()));

  // Assigns the end endpoint.
  const double end_superelevation = end_spec.endpoint_z().theta();
  const double end_lane_offset =
      -lane_layout.ref_r0() +
      lane_width_ *
          static_cast<double>(end_spec.lane_id() - lane_layout.ref_lane());
  const double end_z_dot = end_spec.endpoint_z().z_dot() *
                           (arc_offset.radius() -
                            end_lane_offset * std::cos(end_superelevation) *
                                std::copysign(1., arc_offset.d_theta())) /
                           arc_offset.radius();
  EndpointZ end_z{end_spec.endpoint_z().z() -
                      end_lane_offset * std::sin(end_superelevation) *
                          std::copysign(1., arc_offset.d_theta()),
                  end_z_dot,
                  end_spec.endpoint_z().theta(),
                  {}};
  ComputeContinuityConstraint(curvature, &end_z);

  connections_.push_back(std::make_unique<Connection>(
      id, start, end_z, lane_layout.num_lanes(),
      -lane_layout.ref_r0() -
          static_cast<double>(lane_layout.ref_lane()) * lane_width_,
      lane_width_, lane_layout.left_shoulder(), lane_layout.right_shoulder(),
      arc_offset, linear_tolerance_, scale_length_, computation_policy_));
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
  groups_.push_back(group_factory_->Make(id));
  return groups_.back().get();
}


Group* Builder::MakeGroup(const std::string& id,
                          const std::vector<const Connection*>& connections) {
  groups_.push_back(group_factory_->Make(id, connections));
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
      id, linear_tolerance_, angular_tolerance_, scale_length_);
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
