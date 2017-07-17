#include "drake/automotive/maliput/rndf/builder.h"

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace maliput {
namespace rndf {

// Let p and q be the position of two RNDF waypoints w1 and w2 which belong to
// different RNDF lanes on the same RNDF segment. Let F1 and F2 be the curve
// functions that parameterizes each lane. In addition, let q' be the
// intersection point of the normal vector of q at F1's trajectory. We can
// define the distance between q' and p as D, and when D is less than
// kWaypointDistancePhase there is no new DirectedWaypoint inserted into the
// set of DirectedWaypoints that will describe F1.
static const double kWaypointDistancePhase = 2.0;

// Let F be a function that describes a reference curve for a Lane. F will be
// sampled at kLinearStep so as to get a point over the curve whose distance
// is minimum with respect to another point in space.
static const double kLinearStep = 1e-2;

// Let S be a cubic Bezier spline, with control points p0, p1, p2 and p3.
//                        Define a critical point c where lines (p1-p0)*t +
//      p1 -- p2          p0 and (p2-p3)*w + p3 meet. To prevent loops and
//      /       \         cusps, it sufficient to ensure that control points
//     p0        \        p1 and p2 lie in the line segments (c - p0)*u + p0
//               p3       and (c - p3)*v + p3.
// Here we make u = v = kBezierScaling. Note that a unitary scaling will
// place inner control points p1 and p2 at the critical point c.
static const double kBezierScaling = 1.0;

void Builder::CreateConnection(double width,
                               const ignition::rndf::UniqueId& exit,
                               const ignition::rndf::UniqueId& entry) {
  const auto& exit_it = directed_waypoints_.find(exit.String());
  const auto& entry_it = directed_waypoints_.find(entry.String());
  DRAKE_THROW_UNLESS(exit_it != directed_waypoints_.end());
  DRAKE_THROW_UNLESS(entry_it != directed_waypoints_.end());

  std::vector<DirectedWaypoint> waypoints =
      CreateDirectedWaypointsForConnections(exit_it->second, entry_it->second);
  std::string key_id = exit.String() + std::string("-") + entry.String();
  InsertConnection(key_id, width, waypoints);
}

std::vector<DirectedWaypoint> Builder::CreateDirectedWaypointsForConnections(
    const DirectedWaypoint& exit, const DirectedWaypoint& entry) const {
  // Converts the the point and tangents to Bezier base.
  const std::vector<ignition::math::Vector3d>& bezier_points = SplineToBezier(
      exit.position(), exit.tangent(), entry.position(), entry.tangent());
  // Validates that these points will not generate a loop / cusp.
  const std::vector<ignition::math::Vector3d>& adapted_bezier_points =
      MakeBezierCurveMonotonic(bezier_points, kBezierScaling);
  // Converts back those Bezier control points to Hermite spline base.
  const std::vector<ignition::math::Vector3d>& hermite_points =
      BezierToSpline(adapted_bezier_points[0], adapted_bezier_points[1],
                     adapted_bezier_points[2], adapted_bezier_points[3]);
  // Creates a pair of DirectedWaypoints and returns them.
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(exit.id(), hermite_points[0],
                                       hermite_points[1], true, false));
  waypoints.push_back(DirectedWaypoint(entry.id(), hermite_points[2],
                                       hermite_points[3], false, true));
  return waypoints;
}

void Builder::SetInvertedConnections(std::vector<Connection>* connections) {
  DRAKE_DEMAND(connections != nullptr);
  if (connections->size() == 0) {
    return;
  }
  // Creates a center of rotation outside the bounding box, for connection
  // direction estimation by means of "momentum" computations.
  const ignition::math::Vector3d center_of_rotation =
      std::get<0>(bounding_box_) - ignition::math::Vector3d(1., 1., 0);
  // Gets all the "momentum"s and mark the connections if necessary.
  int first_connection_sign_momentum, other_connection_sign_momentum;
  double momentum = CalculateConnectionMomentum(center_of_rotation,
                                                connections->at(0).waypoints());
  first_connection_sign_momentum = std::copysign(1.0, momentum);
  for (int i = 1; i < static_cast<int>(connections->size()); i++) {
    momentum = CalculateConnectionMomentum(center_of_rotation,
                                           connections->at(i).waypoints());
    other_connection_sign_momentum = std::copysign(1.0, momentum);
    if (other_connection_sign_momentum != first_connection_sign_momentum) {
      connections->at(i).set_inverse_direction(true);
    }
  }
}

void Builder::CreateSegmentConnections(int segment_id,
                                       std::vector<Connection>* connections) {
  DRAKE_THROW_UNLESS(connections != nullptr);
  DRAKE_THROW_UNLESS(connections->size() > 0);
  // Builds the vector of waypoints.
  for (Connection& connection : *connections) {
    std::vector<DirectedWaypoint> waypoints(connection.waypoints());
    BuildTangentsForWaypoints(&waypoints);
    connection.set_waypoints(waypoints);
  }
  // Checks the momentum that each lane produces and then sets the connected
  // lane accordingly.
  SetInvertedConnections(connections);
  // Splits the connections into groups with the same direction.
  std::map<int, std::vector<Connection>> segment_groups;
  GroupConnectionsByDirection(*connections, &segment_groups);
  for (auto& it : segment_groups) {
    // Adds DirectedWaypoints when necessary to match projected positions from
    // one RNDF lane onto the other.
    CreateNewControlPointsForConnections(&(it.second));
    for (int i = 0; i < static_cast<int>((it.second[0].waypoints().size() - 1));
         i++) {
      // Finds the valid RNDF lane pieces. Then, it orders them from right to
      // left.
      std::vector<int> valid_lane_ids;
      for (int j = 0; j < static_cast<int>(it.second.size()); j++) {
        if (it.second[j].waypoints()[i].id().Valid() &&
            it.second[j].waypoints()[i + 1].id().Valid()) {
          valid_lane_ids.push_back(j);
        }
      }
      OrderConnectionIds(it.second, &valid_lane_ids, 0);
      // Creates a segment name.
      std::string segment_key_name =
          std::to_string(segment_id) + std::string("-") +
          std::to_string(it.first) + std::string("-") + std::to_string(i);
      // Iterates over the valid lane ids.
      for (auto lane_it = valid_lane_ids.begin();
           lane_it != valid_lane_ids.end(); ++lane_it) {
        // Creates a two waypoint connection..
        std::vector<DirectedWaypoint> wps;
        wps.push_back(it.second[*lane_it].waypoints()[i]);
        wps.push_back(it.second[*lane_it].waypoints()[i + 1]);
        InsertConnection(segment_key_name, it.second[*lane_it].width(), wps);
        // Adds the pair of waypoints to the map.
        directed_waypoints_[wps.front().id().String()] = wps.front();
        directed_waypoints_[wps.back().id().String()] = wps.back();
      }
    }
  }
}

void Builder::CreateConnectionsForZones(
    double width, std::vector<DirectedWaypoint>* perimeter_waypoints) {
  DRAKE_THROW_UNLESS(perimeter_waypoints != nullptr);
  DRAKE_THROW_UNLESS(perimeter_waypoints->size() > 0);
  // Computes the mean coordinates from all the waypoints of the perimeter.
  ignition::math::Vector3d center(0., 0., 0.);
  for (const DirectedWaypoint& waypoint : *perimeter_waypoints) {
    center += waypoint.position();
  }
  center /= static_cast<double>(perimeter_waypoints->size());
  // Fills the tangents for the entries and exits pointing to center.
  std::vector<DirectedWaypoint> entries, exits;
  for (DirectedWaypoint& waypoint : *perimeter_waypoints) {
    if (waypoint.is_entry()) {
      waypoint.set_tangent((center - waypoint.position()).Normalize());
      entries.push_back(waypoint);
      directed_waypoints_[waypoint.id().String()] = waypoint;
    } else if (waypoint.is_exit()) {
      waypoint.set_tangent((waypoint.position() - center).Normalize());
      exits.push_back(waypoint);
      directed_waypoints_[waypoint.id().String()] = waypoint;
    }
  }
  // Creates connection that join exit waypoints with entry waypoints.
  for (const DirectedWaypoint& entry : entries) {
    for (const DirectedWaypoint& exit : exits) {
      std::vector<DirectedWaypoint> control_points = {entry, exit};
      const std::string key_id =
          exit.id().String() + std::string("-") + entry.id().String();
      InsertConnection(key_id, width, control_points);
    }
  }
}

std::unique_ptr<ignition::math::Spline> Builder::CreateSpline(
    const std::vector<DirectedWaypoint>& waypoints) {
  std::vector<ignition::math::Vector3d> positions;
  for (const DirectedWaypoint& waypoint : waypoints) {
    if (!(waypoint.id().Valid())) {
      continue;
    }
    positions.push_back(waypoint.position());
  }
  DRAKE_DEMAND(positions.size() > 1);
  // PChip will provide a first derivative with null vectors at one of the
  // extents which is not a valid value, so for that specific case, a spline
  // is built directly from provided points.
  if (positions.size() == 2) {
    std::unique_ptr<ignition::math::Spline> spline =
        std::make_unique<ignition::math::Spline>();
    spline->AutoCalculate(true);
    spline->AddPoint(positions[0]);
    spline->AddPoint(positions[1]);
    return spline;
  }
  return CreatePChipBasedSpline(positions);
}

void Builder::GroupConnectionsByDirection(
    const std::vector<Connection>& connections,
    std::map<int, std::vector<Connection>>* connection_groups) const {
  DRAKE_DEMAND(connection_groups != nullptr);

  int connection_group_id = 0;
  bool current_inversion = connections.front().inverse_direction();
  for (const Connection& connection : connections) {
    if (current_inversion != connection.inverse_direction()) {
      current_inversion = connection.inverse_direction();
      connection_group_id++;
    }
    // Creates an entry for the map if it doesn't exist.
    if (connection_groups->find(connection_group_id) ==
        connection_groups->end()) {
      (*connection_groups)[connection_group_id] = std::vector<Connection>();
    }
    // Adds the lane to the group.
    (*connection_groups)[connection_group_id].push_back(connection);
  }
}

void Builder::OrderConnectionIds(const std::vector<Connection>& connections,
                                 std::vector<int>* ids, int index) {
  DRAKE_DEMAND(ids != nullptr);
  // Checks for the single connection case, where it is none sense
  // to compute anything.
  if (ids->size() == 1) return;
  // Fills the id_waypoint_list.
  std::vector<std::pair<int, DirectedWaypoint>> id_waypoint_list;
  for (int i = 0; i < static_cast<int>(ids->size()); i++) {
    id_waypoint_list.push_back(
        std::make_pair(ids->at(i), connections.at(i).waypoints().front()));
  }
  // Sorts the list using the dot product between the position of waypoint
  // B in waypoint A frame `p_BA` and the normalized normal to waypoint A
  // direction `n_A`, essentially using a metric that gets larger in the
  // positive numbers as waypoint B is farther to the right of waypoint A
  // (and viceversa).
  std::sort(id_waypoint_list.begin(), id_waypoint_list.end(),
            [](const std::pair<int, DirectedWaypoint>& a,
               const std::pair<int, DirectedWaypoint>& b) {
              ignition::math::Vector3d p_ba =
                  b.second.position() - a.second.position();
              ignition::math::Vector3d n_a(-a.second.tangent().Y(),
                                           a.second.tangent().X(), 0.0);
              n_a.Normalize();
              return (p_ba.Dot(n_a) > 0.0);
            });
  ids->clear();
  for (const auto& it : id_waypoint_list) {
    ids->push_back(it.first);
  }
}

void Builder::BuildTangentsForWaypoints(
    std::vector<DirectedWaypoint>* waypoints) {
  DRAKE_DEMAND(waypoints != nullptr);
  // Builds a spline so as to fill tangent information.
  std::unique_ptr<ignition::math::Spline> spline = CreateSpline(*waypoints);
  int base_id = 0;
  for (int i = 0; i < static_cast<int>(waypoints->size()); i++) {
    if (!waypoints->at(i).id().Valid()) {
      base_id = i;
      continue;
    } else {
      waypoints->at(i).set_tangent(spline->Tangent(i - base_id));
    }
  }
}

double Builder::ComputeProjectedDistance(const DirectedWaypoint& base,
                                         const DirectedWaypoint& target) const {
  return (target.position() - base.position())
      .Dot(ignition::math::Vector3d(base.tangent()).Normalize());
}

std::vector<int> Builder::GetInitialConnectionToProcess(
    const std::vector<Connection>& connections, int index) {
  // Computes the distance matrix.
  std::vector<std::vector<double>> distances_matrix;
  for (int i = 0; i < static_cast<int>(connections.size()); i++) {
    distances_matrix.push_back(std::vector<double>());
    for (int j = 0; j < static_cast<int>(connections.size()); j++) {
      if (i == j) {
        // Since we are comparing against ourselves, we set it as invalid.
        distances_matrix[i].push_back(-1.);
      } else if (index >= static_cast<int>(connections[i].waypoints().size()) ||
                 index >= static_cast<int>(connections[j].waypoints().size())) {
        // In this case it's none sense to compute the distance as one of the
        // connections is shorter that the other in terms of waypoints.
        distances_matrix[i].push_back(-1.);
      } else {
        // Computes the distance.
        distances_matrix[i].push_back(
            ComputeProjectedDistance(connections[i].waypoints()[index],
                                     connections[j].waypoints()[index]));
      }
    }
  }
  // Computes the number of valid distances in the matrix.
  std::vector<std::pair<int, int>> index_valid_distances;
  int i = 0;
  for (const std::vector<double>& distances : distances_matrix) {
    int number_of_valid_distances = 0;
    for (const double& d : distances) {
      if (d > kWaypointDistancePhase) {
        number_of_valid_distances++;
      }
    }
    index_valid_distances.push_back(
        std::make_pair(i, number_of_valid_distances));
    i++;
  }
  // Sorts the vector by increasing number of valid distances. It gives us as
  // the first items, the ids of the Connections to process.
  std::sort(std::begin(index_valid_distances), std::end(index_valid_distances),
            [](const std::pair<int, int>& t_a, const std::pair<int, int>& t_b) {
              return t_a.second > t_b.second;
            });
  // Creates a vector with all the connection ids that appear first.
  std::vector<int> ids;
  for (const std::pair<int, int>& id_zeros : index_valid_distances) {
    if (id_zeros.second == index_valid_distances[0].second)
      ids.push_back(id_zeros.first);
  }
  // In case we have all the lane ids, no one is the first, all are on the same
  // line.
  if (ids.size() == index_valid_distances.size()) {
    ids.clear();
  }
  return ids;
}

void Builder::AddWaypointIfNecessary(const std::vector<int>& ids,
                                     std::vector<Connection>* connections,
                                     int index) {
  DRAKE_DEMAND(connections != nullptr);
  for (int i = 0; i < static_cast<int>(connections->size()); i++) {
    Connection& connection = connections->at(i);
    // Checks that `i` index is in the ids vector which holds
    // the indexes of connections with reference waypoints.
    if (std::find(ids.begin(), ids.end(), i) != ids.end()) {
      continue;
    }

    int waypoint_count = static_cast<int>(connection.waypoints().size());
    // Checks if we don't have any index on the vector.
    if (ids.size() == 0 && waypoint_count > index) {
      continue;
    }
    if (waypoint_count <= index) {
      // Adds an invalid waypoint to keep consistency since there is no more
      // valid waypoints in the connection.
      connection.AddWaypoint(DirectedWaypoint(), waypoint_count);
    } else if (connection.waypoints()[index].id().Z() == 1) {
      // As the waypoint is at the top of the connection's vector, it adds an
      // invalid one before the first waypoint.
      connection.AddWaypoint(DirectedWaypoint(), 0);
    } else {
      // Adds a waypoint to the projected position of the side lane.
      std::unique_ptr<ignition::math::Spline> spline =
          CreateSpline(connections->at(i).waypoints());
      std::unique_ptr<ArcLengthParameterizedSpline> arc_length_param_spline =
          std::make_unique<ArcLengthParameterizedSpline>(std::move(spline),
                                                         linear_tolerance_);
      const double s = arc_length_param_spline->FindClosestPointTo(
          connections->at(ids[0]).waypoints()[index].position(), kLinearStep);
      // Builds a new waypoint and adds it to the connection.
      DirectedWaypoint new_wp(
          ignition::rndf::UniqueId(
              connections->at(i).waypoints()[index - 1].id().X(),
              connections->at(i).waypoints()[index - 1].id().Y(),
              connections->at(i).waypoints().size() + 1),
          arc_length_param_spline->InterpolateMthDerivative(0, s),
          arc_length_param_spline->InterpolateMthDerivative(1, s), false,
          false);
      connection.AddWaypoint(new_wp, index);
    }
  }
}

void Builder::CreateNewControlPointsForConnections(
    std::vector<Connection>* connections) {
  DRAKE_DEMAND(connections != nullptr);
  // Loads the tangents for each lane waypoints.
  for (Connection& connection : *connections) {
    std::vector<DirectedWaypoint> waypoints(connection.waypoints());
    BuildTangentsForWaypoints(&waypoints);
    connection.set_waypoints(waypoints);
  }
  int i = 0;
  bool should_continue = true;
  while (should_continue) {
    // Gets the connection ids which appear first.
    std::vector<int> ids = GetInitialConnectionToProcess(*connections, i);
    // Checks if we need to create a waypoint for the other lane and adds them
    // if necessary.
    AddWaypointIfNecessary(ids, connections, i);
    // Checks if index is bounded to any of the connections.
    i++;
    should_continue = false;
    for (const Connection& connection : *connections) {
      if (i < static_cast<int>(connection.waypoints().size())) {
        should_continue = true;
        break;
      }
    }
  }
}

std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) {
  auto branch_point_map =
      std::make_unique<std::map<std::string, BranchPoint*>>();
  auto road_geometry =
      std::make_unique<RoadGeometry>(id, linear_tolerance_, angular_tolerance_);

  // Builds a lane per connection and creates their respective BranchPoints.
  for (const auto& it_connection : connections_) {
    // Builds a junction and the segment for related lanes.
    Junction* junction =
        road_geometry->NewJunction({std::string("j:") + it_connection.first});
    DRAKE_DEMAND(junction != nullptr);

    Segment* segment =
        junction->NewSegment({std::string("s:") + it_connection.first});
    DRAKE_DEMAND(segment != nullptr);

    for (const auto& connection : it_connection.second) {
      // Creates a Lane.
      drake::maliput::rndf::Lane* lane = BuildConnection(*connection, segment);
      DRAKE_DEMAND(lane != nullptr);
      // Builds the branch points of necessary for the lane.
      BuildOrUpdateBranchpoints(connection.get(), lane, branch_point_map.get(),
                                road_geometry.get());
    }
  }
  // Checks there is no failure when checking RoadGeometry failure.
  std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const std::string& s : failures) {
    drake::log()->error(s);
  }
  DRAKE_THROW_UNLESS(failures.size() == 0);

  return std::move(road_geometry);
}

void Builder::InsertConnection(const std::string& key_id, double width,
                               const std::vector<DirectedWaypoint>& waypoints) {
  DRAKE_DEMAND(waypoints.size() > 1);
  const ignition::rndf::UniqueId& start_id = waypoints.front().id();
  const ignition::rndf::UniqueId& end_id = waypoints.back().id();
  const std::string name =
      start_id.String() + std::string("-") + end_id.String();
  if (connections_.find(key_id) == connections_.end()) {
    connections_[key_id] = std::vector<std::unique_ptr<Connection>>();
  }
  connections_[key_id].push_back(
      std::make_unique<Connection>(name, waypoints, width, false));
}

namespace {
// Determines the heading (in xy-plane) along the centerline when travelling
// towards/into the lane, from the specified end.
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

void Builder::AttachLaneEndToBranchPoint(const api::LaneEnd::Which end,
                                         Lane* lane,
                                         BranchPoint* branch_point) {
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point != nullptr);
  // Tells the lane about its branch-point.
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(branch_point);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(branch_point);
      break;
    }
    default: { DRAKE_ABORT(); }
  }
  // Tells the BranchPoint about the lane. When the A-Side is empty, it adds
  // the LaneEnd to it.
  if (branch_point->GetASide()->size() == 0) {
    branch_point->AddABranch({lane, end});
    return;
  }
  // Checks the direction of the LaneEnd. When it is parallel to the first
  // LaneEnd, it will be added to the A-Side. If not, it will be added to the
  // B-Side.
  const double new_h = HeadingIntoLane(lane, end);
  const api::LaneEnd old_le = branch_point->GetASide()->get(0);
  const double old_h = HeadingIntoLane(old_le.lane, old_le.end);
  if (((std::cos(new_h) * std::cos(old_h)) +
       (std::sin(new_h) * std::sin(old_h))) > 0.) {
    branch_point->AddABranch({lane, end});
  } else {
    branch_point->AddBBranch({lane, end});
  }
}

Lane* Builder::BuildConnection(const Connection& connection, Segment* segment) {
  DRAKE_DEMAND(segment != nullptr);
  // Creates a new segment and assigns a lane to it.
  Lane* lane{};
  api::LaneId lane_id{std::string("l:") + connection.id()};

  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      points_tangents;
  for (const DirectedWaypoint& directed_waypoint : connection.waypoints()) {
    points_tangents.push_back(std::make_tuple(directed_waypoint.position(),
                                              directed_waypoint.tangent()));
  }
  lane = segment->NewSplineLane(lane_id, points_tangents, connection.width());

  return lane;
}

void Builder::BuildOrUpdateBranchpoints(
    Connection* connection, Lane* lane,
    std::map<std::string, BranchPoint*>* branch_point_map,
    RoadGeometry* road_geometry) {
  DRAKE_DEMAND(connection != nullptr);
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point_map != nullptr);
  DRAKE_DEMAND(road_geometry != nullptr);
  // Sets the start of the BranchPoint.
  BranchPoint* bp{nullptr};
  auto it = branch_point_map->find(connection->start().id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(
        {"bp:" + std::to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->start().id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(api::LaneEnd::kStart, lane, bp);
  // Sets the end of the BranchPoint.
  bp = nullptr;
  it = branch_point_map->find(connection->end().id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(
        {"bp:" + std::to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->end().id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(api::LaneEnd::kFinish, lane, bp);
}

double Builder::CalculateMomentum(
    const ignition::math::Vector3d& center_of_rotation,
    const DirectedWaypoint& waypoint) {
  // Calculates waypoint W position with respect to the center
  // of rotation R.
  const ignition::math::Vector3d p_WR =
      waypoint.position() - center_of_rotation;
  // Computes torque t on waypoint W applied around the center
  // of rotation R.
  ignition::math::Vector3d f_W = waypoint.tangent().Normalized();
  const ignition::math::Vector3d t_WR = f_W.Cross(p_WR);
  // As all the points should lie on the x-y plane, the cross product should
  // be collinear with the z-axis, thus the only non zero component is z.
  return t_WR.Z();
}

double Builder::CalculateConnectionMomentum(
    const ignition::math::Vector3d& center_of_rotation,
    const std::vector<DirectedWaypoint>& waypoints) {
  double momentum = 0.0;
  for (const DirectedWaypoint& waypoint : waypoints) {
    momentum += CalculateMomentum(center_of_rotation, waypoint);
  }
  return momentum;
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
