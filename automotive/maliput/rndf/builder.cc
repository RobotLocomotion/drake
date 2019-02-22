#include "drake/automotive/maliput/rndf/builder.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>

#include "ignition/rndf/UniqueId.hh"

#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace rndf {

namespace {

using std::cos;
using std::copysign;
using std::find;
using std::make_unique;
using std::map;
using std::move;
using std::pair;
using std::sin;
using std::sort;
using std::string;
using std::to_string;
using std::tuple;
using std::unique_ptr;
using std::vector;

// Let p and q be the position of two RNDF waypoints w1 and w2 which belong
// to two different RNDF lanes on the same RNDF segment. Let F1 and F2 be the
// curve functions that parametrizes each lane. In addition, let q' be the
// intersection point of the normal vector of q at F1's trajectory. We can
// define the distance between q' and p as D, and when D is less than
// kWaypointDistancePhase no new DirectedWaypoint will be inserted into the
// set of DirectedWaypoints that will describe F1. In other words,
// kWaypointDistancePhase sets a tolerance for waypoint alignment and thus
// prevents the proliferation of very short segments that in turn would pose
// a strain on spline interpolation attempting to ensure convexity and
// monotonicity.
const double kWaypointDistancePhase = 2.0;

// Let F be a function that describes a reference curve for a Lane. To find
// the point on the reference curve whose Euclidean distance to another
// arbitrary point is minimum, F will be sampled every kLinearStep, directly
// affecting the accuracy of the approximation.
const double kLinearStep = 1e-2;

// Let S be a cubic Bezier spline, with control points p0, p1, p2 and p3.
//          c
//         / \        Define a critical point c where lines (p1-p0)*t + p0
//        /   \       and (p2-p3)*w + p3 meet, where t and w are real scalars.
//       /     \      To prevent loops and cusps, it is sufficient to ensure
//      p1 --- p2     that control points p1 and p2 lie in the region bounded
//      /        \    by the line segments (c - p0)*u + p0 and (c - p3)*v + p3,
//     p0         \   where u and v are real scalars.
//                p3
// Here we make u = v = kBezierScaling. Note that a unitary scaling will place
// inner control points p1 = (c - p0)*u + p0 and p2 = (c - p3)*v + p3 at the
// critical point c.
const double kBezierScaling = 1.0;

// Determines the heading (in xy-plane) along the centerline when traveling
// towards/into the @p lane, from the specified @p end.
double HeadingIntoLane(const api::Lane* const lane,
                       const api::LaneEnd::Which end) {
  switch (end) {
    case api::LaneEnd::kStart: {
      return lane->GetOrientation({0., 0., 0.}).yaw();
    }
    case api::LaneEnd::kFinish: {
      return lane->GetOrientation({lane->length(), 0., 0.}).yaw() + M_PI;
    }
  }
  DRAKE_UNREACHABLE();
}

// Computes the Euclidean distance between @p base and @p target,
// projected along base's tangent.
double ComputeProjectedDistance(const DirectedWaypoint& base,
                                const DirectedWaypoint& target) {
  return (target.position() - base.position())
      .Dot(ignition::math::Vector3d(base.tangent()).Normalize());
}

// Builds an ignition::math::Spline from a set of @p waypoints.
//
// When at least three (3) valid waypoints are provided, a PChip
// algorithm is used to ensure monotonicity and convexity by means
// of calling CreatePChipBasedSpline().
// @remarks Any invalid waypoint in @p waypoints will not be taken into
// account.
// @pre There must be at least two (2) valid waypoints in @p waypoints.
// @warning This method will abort if any preconditions are not met.
unique_ptr<ignition::math::Spline> CreateSpline(
    const vector<DirectedWaypoint>& waypoints) {
  vector<ignition::math::Vector3d> positions;
  for (const DirectedWaypoint& waypoint : waypoints) {
    if (waypoint.id().Valid()) {
      positions.push_back(waypoint.position());
    }
  }
  DRAKE_DEMAND(positions.size() > 1);
  // PChip will provide a first derivative with null vectors at one of the
  // extents which is not a valid value, so for that specific case, a spline
  // is built directly from provided points.
  if (positions.size() == 2) {
    unique_ptr<ignition::math::Spline> spline =
        make_unique<ignition::math::Spline>();
    spline->AutoCalculate(true);
    spline->AddPoint(positions[0]);
    spline->AddPoint(positions[1]);
    return spline;
  }
  return CreatePChipBasedSpline(positions);
}

// Interpolates tangents for each of the @p waypoints using a spline curve,
// applying CreateSpline() with the @p waypoints' positions only.
// @pre The given @p waypoints collection must not be a nullptr.
// @warning This method will abort if any preconditions are not met.
void BuildTangentsForWaypoints(vector<DirectedWaypoint>* waypoints) {
  DRAKE_DEMAND(waypoints != nullptr);
  // Builds a spline to obtain tangent information.
  const unique_ptr<ignition::math::Spline> spline = CreateSpline(*waypoints);
  for (size_t base_index = 0, index = 0; index < waypoints->size(); index++) {
    if (!waypoints->at(index).id().Valid()) {
      base_index = index;
    } else {
      waypoints->at(index).set_tangent(spline->Tangent(index - base_index));
    }
  }
}

// Identifies the @p connections that come first in the direction the whole
// collection of connections flow, by computing the projection of every
// waypoint at @p index onto every other waypoint's tangent at @p index so
// as to identify those that have the most projections ahead of them in the
// relative direction sense.
// @pre All @p connections flow in the same relative direction.
// @param connections A collection of Connections to be inspected.
// @param index The index to the waypoints within the @p connections.
// @return A collection with the indexes of the @p connections that come
// first. When the collection is empty, it means that all the @p connections'
// waypoints are in line with the @p connections' normal (thus no one comes
// first).
vector<int> GetInitialConnectionToProcess(const vector<Connection>& connections,
                                          size_t index) {
  // Computes the number of valid distances between connections.
  vector<pair<int, int>> index_valid_distances;
  for (size_t i = 0; i < connections.size(); ++i) {
    int number_of_valid_distances = 0;
    for (size_t j = 0; j < connections.size(); ++j) {
      // Skips distance computation if one of the connections is shorter than
      // the other in terms of waypoints or a waypoint is being compared against
      // itself (i == j).
      if (i != j && index < connections[i].waypoints().size() &&
          index < connections[j].waypoints().size()) {
        // Computes the distance, that will only be considered if it's above
        // the kWaypointDistancePhase minimum.
        double distance =
            ComputeProjectedDistance(connections[i].waypoints()[index],
                                     connections[j].waypoints()[index]);
        if (distance > kWaypointDistancePhase) {
          number_of_valid_distances++;
        }
      }
    }
    index_valid_distances.emplace_back(i, number_of_valid_distances);
  }
  // Sorts the vector by increasing number of valid distances.
  sort(index_valid_distances.begin(), index_valid_distances.end(),
       [](const pair<int, int>& t_a, const pair<int, int>& t_b) {
         return t_a.second > t_b.second;
       });
  // Creates a vector with all the connection ids that appear first.
  vector<int> ids;
  for (const pair<int, int>& id_zeros : index_valid_distances) {
    if (id_zeros.second == index_valid_distances[0].second)
      ids.push_back(id_zeros.first);
  }
  // In case we have all the connection ids, no one is the first, all
  // are on the same line.
  if (ids.size() == index_valid_distances.size()) {
    ids.clear();
  }
  return ids;
}

// Computes the momentum @f$ \tau^W @f$ exerted by the fictitious unitary
// force @f$ f^W @f$ on @p waypoint @f$ W @f$ around the @p center_of_rotation
// @f$ R @f$. This force is applied on the @p waypoint's tangent direction.
// @remarks Since RNDF provides no information on the z coordinate, all
// waypoints are on the @f$ z = 0 @f$ plane and the torque will always be
// applied entirely on the z-axis.
// @return The computed momentum's z-axis component.
double CalculateMomentum(const ignition::math::Vector3d& center_of_rotation,
                         const DirectedWaypoint& waypoint) {
  // Calculates waypoint W position with respect to the center of rotation R.
  const ignition::math::Vector3d p_WR =
      waypoint.position() - center_of_rotation;
  // Computes torque t on waypoint W applied around the center of rotation R.
  const ignition::math::Vector3d f_W = waypoint.tangent().Normalized();
  const ignition::math::Vector3d t_WR = f_W.Cross(p_WR);
  // As all the points should lie on the x-y plane, the cross product should
  // be collinear with the z-axis, thus the only non zero component is z.
  return t_WR.Z();
}

// Computes the momentum sum of all the @p waypoints around @p origin
// using CalculateMomentum(). This is helpful to determine the relative
// direction of a given connection with respect to adjacent ones (lanes in
// a segment).
double CalculateConnectionMomentum(
    const ignition::math::Vector3d& center_of_rotation,
    const vector<DirectedWaypoint>& waypoints) {
  double momentum = 0.0;
  for (const DirectedWaypoint& waypoint : waypoints) {
    momentum += CalculateMomentum(center_of_rotation, waypoint);
  }
  return momentum;
}

// Groups @p connections in separate @p connection_groups based on
// their relative direction.
// @param connections A collection of Connections to be grouped.
// @param connection_groups A mapping for connection group numbers
// to connection groups.
// @pre The given @p connection_groups collection must not be a nullptr.
// @warning This method will abort execution if any preconditions are not met.
void GroupConnectionsByDirection(
    const vector<Connection>& connections,
    map<int, vector<Connection>>* connection_groups) {
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
      (*connection_groups)[connection_group_id] = vector<Connection>();
    }
    // Adds the lane to the group.
    (*connection_groups)[connection_group_id].push_back(connection);
  }
}

// Orders a collection of @p ids, each identifying a connection at the
// same index in @p connections, in a right to left sense. To perform such
// ordering, it uses the first waypoint on each connection.
// @param connections A collection of Connections.
// @param ids The collection of ids for each one of the @p connections.
// @pre The given @p ids collection must not be a nullptr.
// @warning This method will abort execution if any preconditions are not met.
void OrderConnectionIds(const vector<Connection>& connections,
                        vector<int>* ids) {
  DRAKE_DEMAND(ids != nullptr);
  // Checks for the single connection case, where it is nonsense to compute
  // anything.
  if (ids->size() == 1) return;
  // Fills the id_waypoint_list.
  vector<pair<int, DirectedWaypoint>> id_waypoint_list;
  for (size_t i = 0; i < ids->size(); ++i) {
    id_waypoint_list.emplace_back(ids->at(i),
                                  connections.at(i).waypoints().front());
  }
  // Sorts the list using the dot product between the position of waypoint
  // B in waypoint A frame `p_AB` and the normalized normal to waypoint A's
  // direction `n_A`, essentially using a metric that gets larger in the
  // positive numbers as waypoint B is farther to the right of waypoint A
  // (and vice versa).
  sort(id_waypoint_list.begin(), id_waypoint_list.end(),
       [](const pair<int, DirectedWaypoint>& a,
          const pair<int, DirectedWaypoint>& b) {
         const ignition::math::Vector3d p_ab =
             b.second.position() - a.second.position();
         ignition::math::Vector3d n_a(-a.second.tangent().Y(),
                                      a.second.tangent().X(), 0.0);
         n_a.Normalize();
         return (p_ab.Dot(n_a) > 0.0);
       });
  ids->clear();
  for (const auto& it : id_waypoint_list) {
    ids->push_back(it.first);
  }
}

// Builds a Lane from the given @p connection into the parent @p segment.
// @param connection A Connection that holds the waypoints to build the lane.
// @param segment The parent Segment.
// @return The built Lane.
// @pre The given @p segment must not be a nullptr.
// @warning This method will abort if preconditions are not met.
Lane* BuildConnection(const Connection& connection, Segment* segment) {
  DRAKE_DEMAND(segment != nullptr);
  const api::LaneId lane_id{"l:" + connection.id()};
  vector<tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      points_tangents;
  for (const DirectedWaypoint& directed_waypoint : connection.waypoints()) {
    points_tangents.emplace_back(directed_waypoint.position(),
                                 directed_waypoint.tangent());
  }
  return segment->NewSplineLane(lane_id, points_tangents, connection.width());
}

// Attaches a @p lane to a @p branch_point at the specified @p end.
// @pre The given @p lane must not be a nullptr.
// @pre The given @p branch_point must not be a nullptr.
// @warning This method will abort if preconditions are not met.
void AttachLaneEndToBranchPoint(const api::LaneEnd::Which end, Lane* lane,
                                BranchPoint* branch_point) {
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point != nullptr);
  // Tells the lane about its branch-point.
  DRAKE_DEMAND((end == api::LaneEnd::kStart) || (end == api::LaneEnd::kFinish));
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(branch_point);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(branch_point);
      break;
    }
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
  if (((cos(new_h) * cos(old_h)) + (sin(new_h) * sin(old_h))) > 0.) {
    branch_point->AddABranch({lane, end});
  } else {
    branch_point->AddBBranch({lane, end});
  }
}

// Builds or updates a BranchPoint given a @p connection and the corresponding
// @p lane. The former provides the start and end waypoints of the latter,
// which are necessary to lookup the branch points in the @p branch_point_map.
// @param connection The Connection that provides the start and end waypoints.
// @param lane The Lane to be attached to the right corresponding
// branch points.
// @param branch_point_map The mapping from RNDF (entry or exit) waypoint IDs
// to BranchPoints.
// @param road_geometry The RoadGeometry to create the BranchPoints into.
// @pre The given @p connection must not be a nullptr.
// @pre The given @p lane must not be a nullptr.
// @pre The given @p branch_point_map must not be a nullptr.
// @pre The given @p road_geometry must not be a nullptr.
// @warning This method will abort if preconditions are not met.
void BuildOrUpdateBranchpoints(Connection* connection, Lane* lane,
                               map<string, BranchPoint*>* branch_point_map,
                               RoadGeometry* road_geometry) {
  DRAKE_DEMAND(connection != nullptr);
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point_map != nullptr);
  DRAKE_DEMAND(road_geometry != nullptr);
  // Sets the start of the BranchPoint.
  BranchPoint* bp{nullptr};
  auto it = branch_point_map->find(connection->start().id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(api::BranchPointId{
        "bp:" + to_string(road_geometry->num_branch_points())});
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
    bp = road_geometry->NewBranchPoint(api::BranchPointId{
        "bp:" + to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->end().id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(api::LaneEnd::kFinish, lane, bp);
}

// Adds either invalid or interpolated extra waypoints on those connections
// not listed in the @p ids of the @p connections that come first for the
// given @p index, as computed by GetInitialConnectionToProcess(), to
// make all waypoints at @p index lie in line with @p connections's normal.
// That is to say, to lie in a row.
// @param ids A collection of the indexes of the @p connections that come
// first.
// @param connections A collection of Connections to add waypoints to if
// necessary.
// @param index The index of the @p connection's waypoints to look at.
// @param linear_tolerance The tolerance for spline interpolation of
// connection waypoints.
// @pre The given @p connections collection must not be a nullptr.
// @warning This method will abort execution if any preconditions are not met.
void AddWaypointsIfNecessary(const vector<int>& ids,
                             vector<Connection>* connections, size_t index,
                             double linear_tolerance) {
  DRAKE_DEMAND(connections != nullptr);
  for (size_t i = 0; i < connections->size(); i++) {
    Connection& connection = connections->at(i);
    // Checks that `i` index is in the ids vector which holds
    // the indexes of connections.
    if (find(ids.begin(), ids.end(), i) != ids.end()) {
      continue;
    }

    const size_t waypoint_count = connection.waypoints().size();
    // Checks if we don't have any index on the vector.
    if (ids.size() == 0 && waypoint_count > index) {
      continue;
    }
    if (waypoint_count <= index) {
      // Adds an invalid waypoint to keep consistency since there is no more
      // valid waypoints in the connection.
      connection.AddWaypoint(DirectedWaypoint(), waypoint_count);
    } else if (connection.waypoints()[index].id().Z() == 1) {
      // As the waypoint is at the beginning of the connection's vector, it
      // adds an invalid one before the first waypoint.
      connection.AddWaypoint(DirectedWaypoint(), 0);
    } else {
      // Index cannot ever be zero, as every first waypoint would be
      // caught by the upper if clause.
      DRAKE_DEMAND(index != 0);
      // Adds a waypoint to the projected position of the side lane.
      unique_ptr<ignition::math::Spline> spline =
          CreateSpline(connections->at(i).waypoints());
      unique_ptr<ArcLengthParameterizedSpline> arc_length_param_spline =
          make_unique<ArcLengthParameterizedSpline>(move(spline),
                                                    linear_tolerance);
      const double s = arc_length_param_spline->FindClosestPointTo(
          connections->at(ids[0]).waypoints()[index].position(), kLinearStep);
      // Builds a new waypoint and adds it to the connection.
      const DirectedWaypoint new_wp(
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

// Adds waypoints to the given @p connections to ensure that their spatial
// distribution is such that for every index all valid waypoints share the same
// s coordinate on the reference lane curve frame.
// @param connections A collection of Connections to create waypoints into
// if necessary.
// @param linear_tolerance The tolerance for spline interpolation of
// connection waypoints.
// @pre The given @p connections collection must not be a nullptr.
// @warning This method will abort execution if any preconditions are not met.
void CreateNewControlPointsForConnections(std::vector<Connection>* connections,
                                          double linear_tolerance) {
  DRAKE_DEMAND(connections != nullptr);
  // Loads the tangents for each of the connections' waypoints.
  for (Connection& connection : *connections) {
    vector<DirectedWaypoint> waypoints(connection.waypoints());
    BuildTangentsForWaypoints(&waypoints);
    connection.set_waypoints(waypoints);
  }
  size_t i = 0;
  bool should_continue = true;
  while (should_continue) {
    // Gets the connection ids which appear first.
    vector<int> ids = GetInitialConnectionToProcess(*connections, i);
    // Checks if we need to add waypoints on other connections and does so
    // if necessary.
    AddWaypointsIfNecessary(ids, connections, i, linear_tolerance);
    // Checks if index is bounded to any of the connections.
    i++;
    should_continue = false;
    for (const Connection& connection : *connections) {
      if (i < connection.waypoints().size()) {
        should_continue = true;
        break;
      }
    }
  }
}

// Establishes the relative direction for each connection in @p connections.
//
// To do this, it checks the connection momentum as computed by
// CalculateConnectionMomentum() and looks for successive sign changes
// with respect to a reference set by the first connection.
// @param bounding_box The bounding box for all given @p connections.
// @param connections A collection of Connections to compare the way
// connections' waypoints are disposed with respect to the first connection.
// @pre The given @p connections collection must not be a nullptr.
// @warning This method will abort execution if any preconditions are not met.
void SetInvertedConnections(const pair<ignition::math::Vector3d,
                                       ignition::math::Vector3d>& bounding_box,
                            vector<Connection>* connections) {
  DRAKE_DEMAND(connections != nullptr);
  if (connections->size() == 0) {
    return;
  }
  // Creates a center of rotation outside the bounding box, for connection
  // direction estimation by means of "momentum" computations.
  const ignition::math::Vector3d center_of_rotation =
      bounding_box.first - ignition::math::Vector3d(1., 1., 0);
  // Gets all the "momentum"s and mark the connections if necessary.
  double momentum = CalculateConnectionMomentum(center_of_rotation,
                                                connections->at(0).waypoints());
  const int first_connection_sign_momentum = copysign(1.0, momentum);
  for (size_t i = 1; i < connections->size(); ++i) {
    momentum = CalculateConnectionMomentum(center_of_rotation,
                                           connections->at(i).waypoints());
    const int other_connection_sign_momentum = copysign(1.0, momentum);
    if (other_connection_sign_momentum != first_connection_sign_momentum) {
      connections->at(i).set_inverse_direction(true);
    }
  }
}

// Creates a pair of waypoints based on the given @p exit and @p entry,
// keeping their heading but affecting tangent norms to achieve smooth
// transitions by making use of cubic Bezier interpolants. This is helpful
// for connecting lanes at intersections.
// @param exit The start waypoint of the lane's curve.
// @param entry The end waypoint of the lane's curve.
// @return A vector with the two (2) waypoints that represent the
// extents of the connection.
vector<DirectedWaypoint> CreateDirectedWaypointsForConnection(
    const DirectedWaypoint& exit, const DirectedWaypoint& entry) {
  // Converts points and tangents into Bezier control points.
  const vector<ignition::math::Vector3d> bezier_points = SplineToBezier(
      exit.position(), exit.tangent(), entry.position(), entry.tangent());
  // Adjusts these controls points to prevent loops and cusps.
  const vector<ignition::math::Vector3d> adapted_bezier_points =
      MakeBezierCurveMonotonic(bezier_points, kBezierScaling);
  // Converts those Bezier control points to Hermite control points.
  const vector<ignition::math::Vector3d> hermite_points =
      BezierToSpline(adapted_bezier_points[0], adapted_bezier_points[1],
                     adapted_bezier_points[2], adapted_bezier_points[3]);
  // Creates a pair of DirectedWaypoints and returns them.
  vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(exit.id(), hermite_points[0],
                                       hermite_points[1], true, false));
  waypoints.push_back(DirectedWaypoint(entry.id(), hermite_points[2],
                                       hermite_points[3], false, true));
  return waypoints;
}

}  // namespace

void Builder::CreateConnection(double width,
                               const ignition::rndf::UniqueId& exit_id,
                               const ignition::rndf::UniqueId& entry_id) {
  const auto& exit_it = directed_waypoints_.find(exit_id.String());
  const auto& entry_it = directed_waypoints_.find(entry_id.String());
  DRAKE_THROW_UNLESS(exit_it != directed_waypoints_.end());
  DRAKE_THROW_UNLESS(entry_it != directed_waypoints_.end());

  const std::vector<DirectedWaypoint> waypoints =
      CreateDirectedWaypointsForConnection(exit_it->second, entry_it->second);
  const std::string key_id = exit_id.String() + "-" + entry_id.String();
  InsertConnection(key_id, width, waypoints);
}

void Builder::CreateConnectionsForZones(
    double width, std::vector<DirectedWaypoint>* perimeter_waypoints) {
  DRAKE_THROW_UNLESS(perimeter_waypoints != nullptr);
  DRAKE_THROW_UNLESS(perimeter_waypoints->size() > 0);
  // Computes the mean coordinates of all the perimeter waypoints.
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
  // Creates connections that join exit waypoints with entry waypoints.
  for (const DirectedWaypoint& entry : entries) {
    for (const DirectedWaypoint& exit : exits) {
      const std::vector<DirectedWaypoint> control_points{entry, exit};
      const std::string key_id = exit.id().String() + "-" + entry.id().String();
      InsertConnection(key_id, width, control_points);
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
  SetInvertedConnections(bounding_box_, connections);
  // Splits the connections into groups with the same direction.
  std::map<int, std::vector<Connection>> segment_groups;
  GroupConnectionsByDirection(*connections, &segment_groups);
  for (auto& it : segment_groups) {
    // Adds DirectedWaypoints when necessary to match projected positions from
    // one RNDF lane onto the other.
    CreateNewControlPointsForConnections(&(it.second), linear_tolerance_);
    for (int i = 0; i < static_cast<int>((it.second[0].waypoints().size() - 1));
         i++) {
      // Finds the valid RNDF lane pieces and then orders them from right to
      // left.
      std::vector<int> valid_lane_ids;
      for (int j = 0; j < static_cast<int>(it.second.size()); j++) {
        if (it.second[j].waypoints()[i].id().Valid() &&
            it.second[j].waypoints()[i + 1].id().Valid()) {
          valid_lane_ids.push_back(j);
        }
      }
      OrderConnectionIds(it.second, &valid_lane_ids);
      const std::string segment_key_name = std::to_string(segment_id) + "-" +
                                           std::to_string(it.first) + "-" +
                                           std::to_string(i);
      // Iterates over the valid lane ids.
      for (auto lane_it = valid_lane_ids.begin();
           lane_it != valid_lane_ids.end(); ++lane_it) {
        // Creates a two waypoint connection.
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
        road_geometry->NewJunction(api::JunctionId{"j:" + it_connection.first});
    DRAKE_DEMAND(junction != nullptr);

    Segment* segment =
        junction->NewSegment(api::SegmentId{"s:" + it_connection.first});
    DRAKE_DEMAND(segment != nullptr);

    for (const auto& connection : it_connection.second) {
      // Creates a Lane.
      rndf::Lane* lane = BuildConnection(*connection, segment);
      DRAKE_DEMAND(lane != nullptr);
      // Builds the branch points for the lane if necessary.
      BuildOrUpdateBranchpoints(connection.get(), lane, branch_point_map.get(),
                                road_geometry.get());
    }
  }
  // Checks there is no failure when checking RoadGeometry invariants.
  const std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const std::string& s : failures) {
    log()->error(s);
  }
  DRAKE_THROW_UNLESS(failures.size() == 0);

  return move(road_geometry);
}

void Builder::InsertConnection(const std::string& key_id, double width,
                               const std::vector<DirectedWaypoint>& waypoints) {
  DRAKE_DEMAND(waypoints.size() > 1);
  const ignition::rndf::UniqueId& start_id = waypoints.front().id();
  const ignition::rndf::UniqueId& end_id = waypoints.back().id();
  const std::string name = start_id.String() + "-" + end_id.String();
  if (connections_.find(key_id) == connections_.end()) {
    connections_[key_id] = std::vector<std::unique_ptr<Connection>>();
  }
  connections_[key_id].push_back(
      std::make_unique<Connection>(name, waypoints, width, false));
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
