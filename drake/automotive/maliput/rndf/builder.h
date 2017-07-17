#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/rndf/UniqueId.hh"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/rndf/connection.h"
#include "drake/automotive/maliput/rndf/directed_waypoint.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

/// A class to ease the construction of a RoadGeometry from Connections and
/// DirectedWaypoints.
///
/// RNDF segments and lanes are mapped to Maliput's Segment and Lane entities,
/// respectively. This mapping is not straightforward, as Maliput is based on
/// analytical curve parameterizations while RNDF provides a sampled geometry
/// based on waypoints. RNDF waypoints are thus used as control points in a
/// cubic spline interpolation that results in SplineLanes. RNDF lanes'
/// direction is implied by RNDF waypoint IDs
/// (`segment_num.lane_num.waypoint_num`) which is captured by the
/// interpolation and reflected in DirectedWaypoints. These DirectedWaypoints
/// are grouped in Connections, which describe not only RNDF lanes but also the
/// connections between pairs of exit and entry RNDF waypoints. During the build
/// process, interpolated waypoints are added to these Connections so as to keep
/// their distribution akin to a grid (and thus enable index-based operations).
///
/// As lanes in an RNDF segment may or may not flow in the same direction,
/// segment connections are also grouped by its direction relative to the first
/// connection found in the collection. An arbitrary point outside the bounding
/// box of the road geometry is selected to be a "center of rotation" and the
/// sum of all the DirectedWaypoints "momentums" (with normalized tangents) is
/// computed. It must be said that the "momentum" equation is used to derive
/// connection direction, but it bears no physical meaning. Since all the
/// waypoints lay in the @f$ z = 0 @f$ plane, the "momentum" vector will only
/// contain a non-zero z coordinate value. The sign of the "momentum"'s z
/// coordinate will be taken as a reference to define the sense of flow. For
/// all other connections, the "momentum" is computed and compared against the
/// sign of the reference.
///
/// RNDF zones do not have a direct mapping to Maliput either. Consequently,
/// fake connections are added between every entry and exit waypoints on these
/// to keep them driveable. These waypoints' direction is set so as to head
/// towards the centroid of all the zone perimeter waypoints. There's currently
/// no support for RNDF zones' parking spots.
///
/// The resulting RoadGeometry presents the following naming for its composed
/// entities:
/// - Lane naming: "l:$1-$2", where
///    1. RNDF exit waypoint ID.
///    2. RNDF entry waypoint ID.
/// - Segment naming: "s:$1-$2-$3", where
///    1. RNDF segment ID.
///    2. Direction-based grouping index.
///    3. RNDF segment piece index (in between every 2 waypoints).
/// - Junction naming: "j:$1-$2-$3", where
///    1. RNDF segment ID.
///    2. Direction-based grouping index.
///    3. RNDF segment piece index (in between every 2 waypoints).
/// - BranchPoint naming: "bp:$1", where
///    1. Index in RoadGeometry's inner collection.
///
/// An example of how this is achieved is depicted in the following example.
/// Please, note that '+' denotes RNDF waypoints, 'x' denotes invalid waypoints
/// and 'o' denotes interpolated waypoints. Also '|' denotes Segment boundaries.
///
/// <pre>
///          1.1.1                   1.1.2
///          +-----------------------+            <---- Connection: 1.1.1-1.1.2
/// 1.2.1                                       1.2.2
/// +-------------------------------------------+ <---- Connection: 1.2.1-1.2.2
///          1.3.1                   1.3.2
///          +-----------------------+            <---- Connection: 1.3.1-1.3.2
/// </pre>
///
/// Mapping the above waypoints to SplineLane and Segment objects will be done
/// by the Builder. In the end, we end up with something like:
///
/// <pre>
/// |        |1.1.1                  |1.1.2     |
/// x--------+-----------------------+----------x <---- Connection: 1.1.1-1.1.2
/// |1.2.1   |1.2.3                  |1.2.4     |1.2.2
/// +--------o-----------------------o----------+ <---- Connection: 1.2.1-1.2.2
/// |        |1.3.1                  |1.3.2     |
/// x--------+-----------------------+----------x <---- Connection: 1.3.1-1.3.2
/// </pre>
///
/// Those new waypoints that appear make possible the concept of Maliput Segment
/// as a surface that holds all the trajectories (represented by Maliput Lanes).
/// At the building stage, we can now match waypoints across lanes using an
/// index-based algorithm, a pair of valid consecutive waypoints in a Connection
/// are used to create a SplineLane. At the same time, we should
/// create those Lanes inside the same segment, which is not difficult to do
/// using the index approach. To sum up, Builder will create the following:
///
/// - Segment 1-0-0:
///   - Lane 1.2.1-1.2.3
/// - Segment 1-0-1:
///   - Lane 1.3.1-1.3.2
///   - Lane 1.2.3-1.3.4
///   - Lane 1.1.1-1.3.2
/// - Segment 1-0-2:
///   - Lane 1.2.4-1.2.2
///
/// General workflow with this class should be:
/// -# Create a Builder.
/// -# Call SetBoundingBox().
/// -# Call CreateSegmentConnections() for each RNDF segment.
/// -# Call CreateConnectionsForZones() for each RNDF zone.
/// -# Call CreateConnection() for each pair of entry-exit in RNDF lanes and
/// perimeters.
/// -# Call Build to get the built api::RoadGeometry.
class Builder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Builder)

  /// Constructs a Builder which can be used to specify and assemble
  /// an RNDF implementation of an api::RoadGeometry.
  ///
  /// @param linear_tolerance Linear tolerance for RoadGeometry construction.
  /// @param angular_tolerance Angular tolerance for RoadGeometry construction.
  Builder(double linear_tolerance, double angular_tolerance)
      : linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  /// Sets the bounding box of the RNDF map.
  ///
  /// @param bounding_box A tuple containing the lower left corner and
  /// upper right corner positions of the bounding box respectively.
  void SetBoundingBox(
      const std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>&
          bounding_box) {
    bounding_box_ = bounding_box;
  }

  /// Populates the Builder's inner connection map with the given
  /// @p connections representing an RNDF segment.
  ///
  /// In order to do so, the @p connections' waypoints are first used to derive
  /// a geometry. Then, these @p connections are grouped based on relative
  /// direction using the first connection found as a reference. Once grouped,
  /// extra waypoints are added to each of them on a per group basis as
  /// necessary so as to ensure a grid-like distribution of waypoints.
  /// @param segment_id The RNDF segment ID.
  /// @param connections A collection of Connections representing each RNDF lane
  /// in the segment.
  /// @throw std::runtime_error When @p connections is a nullptr.
  /// @throw std::runtime_error When @p connections is an empty collection.
  void CreateSegmentConnections(int segment_id,
                                std::vector<Connection>* connections);

  /// Creates a collection of Connections between every pair of entry and exit
  /// waypoints in @p perimeter_waypoints.
  ///
  /// RNDF defines zones as areas where free-path driving is allowed. Since
  /// Maliput does not cover this concept, and to avoid having lanes with closed
  /// ends, every pair entry and exit waypoints is connected. These waypoints'
  /// direction is set so as to head towards the centroid of all the
  /// @p perimeter_waypoints.
  /// @param width The width of this zone's inner connections.
  /// @param perimeter_waypoints A collection of DirectedWaypoints describing
  /// the zone perimeter.
  /// @throw std::runtime_error When @p perimeter_waypoints is a nullptr.
  /// @throw std::runtime_error When @p perimeter_waypoints' is an empty
  /// collection.
  void CreateConnectionsForZones(
      double width, std::vector<DirectedWaypoint>* perimeter_waypoints);

  /// Creates a connection between two RNDF lanes based on a pair of @p exit and
  /// @p entry ids that map to specific, existing waypoints.
  /// @param width The connection's width.
  /// @param exit The start waypoint ID of the connection.
  /// @param entry The end waypoint ID of the connection.
  /// @throw std::runtime_error When neither @p exit nor @p entry are found.
  void CreateConnection(double width, const ignition::rndf::UniqueId& exit,
                        const ignition::rndf::UniqueId& entry);

  /// Builds an api::RoadGeometry.
  ///
  /// All the groups of connections are traversed. A Junction with a single
  /// Segment is created per group, and for each connection in that group,
  /// a Lane is added to the Segment. BranchPoints are updated as needed.
  /// @param id ID of the api::RoadGeometry to be built.
  /// @return A pointer to the built api::RoadGeometry.
  /// @throw std::runtime_error When the built RoadGeometry does not satisfy
  /// Maliput roads' constraints (see api::RoadGeometry::CheckInvariants()).
  std::unique_ptr<const api::RoadGeometry> Build(const api::RoadGeometryId& id);

 private:
  // Inserts a connection of the given @p width, with the given @p key_id, using
  // the given @p waypoints to the inner connection map.
  // @pre The given @p waypoints collection size must be at least two (2).
  // @warning This method will abort if preconditions are not met.
  void InsertConnection(const std::string& key_id, double width,
                        const std::vector<DirectedWaypoint>& waypoints);

  // Attaches a @p lane to a @p branch_point at the specified @p end.
  // @pre The given @p lane must not be a nullptr.
  // @pre The given @p branch_point must not be a nullptr.
  // @warning This method will abort if preconditions are not met.
  void AttachLaneEndToBranchPoint(const api::LaneEnd::Which end, Lane* lane,
                                  BranchPoint* branch_point);

  // Builds or updates a BranchPoint given a @p connection and the corresponding
  // @p lane. The former provides the start and end waypoints of the lane,
  // necessary to lookup the branch points in the @p branch_point_map.
  // @param connection A pointer to the Connection that provides the start
  // and end waypoints.
  // @param lane A pointer to the Lane to be attached to the right correspoding
  // branch points.
  // @param branch_point_map A pointer to the mapping from RNDF (entry or exit)
  // waypoint IDs to BranchPoints.
  // @param road_geometry A pointer to the RoadGeometry to create the
  // BranchPoints into.
  // @pre The given @p connection must not be a nullptr.
  // @pre The given @p lane must not be a nullptr.
  // @pre The given @p branch_point_map must not be a nullptr.
  // @pre The given @p road_geometry must not be a nullptr.
  // @warning This method will abort if preconditions are not met.:
  void BuildOrUpdateBranchpoints(
      Connection* connection, Lane* lane,
      std::map<std::string, BranchPoint*>* branch_point_map,
      RoadGeometry* road_geometry);

  // Builds a Lane from the given @p connection information into the parent
  // @p segment.
  // @param connection A Connection that holds the waypoints to build the
  // lane.
  // @param segment A pointer to the parent Segment.
  // @return A pointer to the built Lane.
  // @pre The given @p segment must not be a nullptr.
  // @warning This method will abort if preconditions are not met.
  Lane* BuildConnection(const Connection& connection, Segment* segment);

  // Builds an ignition::math::Spline from a set of @p waypoints.
  //
  // When at least three (3) valid waypoints are provided, a PChip
  // algorithm is used to ensure monotonicity and convexity by means.
  // of calling CreatePChipBasedSpline().
  // @remarks Any invalid waypoint in @p waypoints will not be taken into
  // account.
  // @pre There must be at least two (2) valid waypoints in @p waypoints.
  // @warning This method will abort if any preconditions are not met.
  std::unique_ptr<ignition::math::Spline> CreateSpline(
      const std::vector<DirectedWaypoint>& waypoints);

  // Identifies the @p connections that come first in the direction these flow,
  // by computing the projection of every waypoint at @p index onto every other
  // waypoint's tangent at @p index so as to identify those that have the most
  // projections ahead of them in the relative direction sense.
  // @pre All @p connections flow in the same relative direction.
  // @param connections A collection of Connections to be inspected.
  // @param index The index to the waypoints within the @p connections'.
  // @return A collection with the indexes of the @p connections that come first
  // first. When the collection is empty, it means that all the @p connections'
  // waypoints are in line with the @p connections' normal (thus no one comes
  // first).
  std::vector<int> GetInitialConnectionToProcess(
      const std::vector<Connection>& connections, int index);

  // Interpolates tangents for each of the @p waypoints using a spline curve,
  // applying CreateSpline() with the @p waypoints' positions only.
  // @pre The given @p waypoints collection must not be a nullptr.
  // @warning This method will abort if any preconditions are not met.
  void BuildTangentsForWaypoints(std::vector<DirectedWaypoint>* waypoints);

  // Computes the euclidean distance between @p base and @p target waypoints,
  // projected along @p base's tangent.
  double ComputeProjectedDistance(const DirectedWaypoint& base,
                                  const DirectedWaypoint& target) const;

  // Adds either invalid or interpolated extra waypoints on those @p connections
  // not listed in the @p ids of the @p connections that come first for the
  // given @p index, as computed by GetInitialConnectionToProcess(), so as to
  // make all waypoints at @p index lie in line with @p connections's normal.
  // That is to say, to lie in a row.
  // @param ids A collection of the indexes of the @p connections that come
  // first.
  // @param connections A collection of Connections to add waypoints to if
  // necessary.
  // @param index The index of the @p connection's waypoints to look at.
  // @pre The given @p connections collection must not be a nullptr.
  // @warning This method will abort execution if any preconditions are not met.
  void AddWaypointIfNecessary(const std::vector<int>& ids,
                              std::vector<Connection>* connections, int index);

  // Adds waypoints to the given @p connections so as to ensure that their
  // spatial distribution is akin to a grid (thus enabling index operations).
  // @param connections A collection of Connections to create waypoints into
  // if necessary.
  // @pre The given @p connections collection must not be a nullptr.
  // @warning This method will abort execution if any preconditions are not met.
  void CreateNewControlPointsForConnections(
      std::vector<Connection>* connections);

  // Orders a collection of @p ids, each identifying a connection at the
  // same index in @p connections, in a right to left sense. To perform such
  // ordering, the waypoints on each connection at @p index are used.
  // @param connections A collection of Connections.
  // @param ids The collection of ids for each one of the @p connections.
  // @param index The waypoint index to use for the ordering.
  // @pre The given @p ids collection must not be a nullptr.
  // @warning This method will abort execution if any preconditions are not met.
  void OrderConnectionIds(const std::vector<Connection>& connections,
                          std::vector<int>* ids, int index);

  // Computes the momentum @f$ \tau^W @f$ exerted by the fictitious unitary
  // force @f$ f^W @f$ on @p waypoint @f$ W @f$ around the @p center_of_rotation
  // @f$ R @f$. This force is applied on the @p waypoint's tangent direction.
  // @remarks Since RNDF provides no information on the z coordinate, all
  // waypoints are on the @f$ z = 0 @f$ plane and the torque will always be
  // applied entirely on the z-axis.
  // @return The computed momentum's z-axis component.
  double CalculateMomentum(const ignition::math::Vector3d& center_of_rotation,
                           const DirectedWaypoint& waypoint);

  // Computes the momentum sum of all the @p waypoints around @p origin
  // using CalculateMomentum(). This is helpful to determine the relative
  // direction of a given connection with respect to adjacent ones (lanes in
  // a segment).
  double CalculateConnectionMomentum(
      const ignition::math::Vector3d& origin,
      const std::vector<DirectedWaypoint>& waypoints);

  // Establishes the relative direction for each connection in @p connections.
  //
  // To do this, it checks the connection momentum as computed by
  // CalculateConnectionMomentum() and looks for successive sign changes
  // with respect to a reference set with the first connection.
  // @param connections A collection of Connections to compare the way
  // connections' waypoints are disposed with respect to the first connection.
  // @pre The given @p connections collection must not be a nullptr.
  // @warning This method will abort execution if any preconditions are not met.
  void SetInvertedConnections(std::vector<Connection>* connections);

  // Groups @p connections in separate @p connection_groups based on
  // their relative direction.
  // @param connections A collection of Connections to be grouped.
  // @param connection_groups A mapping for connection group numbers
  // to connection groups.
  // @pre The given @p connection_groups collection must not be a nullptr.
  // @warning This method will abort execution if any preconditions are not met.
  void GroupConnectionsByDirection(
      const std::vector<Connection>& connections,
      std::map<int, std::vector<Connection>>* connection_groups) const;

  // Creates a pair of waypoints based on the given @p exit and @p entry
  // ones, keeping their heading but affecting tangent norms so as to
  // achieve smooth transitions by making use of cubic Bezier interpolants.
  // This is helpful for connecting lanes at intersections.
  // @param exit The start DirectedWaypoint of the lane's reference curve.
  // @param entry The end DirectedWaypoint of the lane's reference curve.
  // @return A vector with the two (2) waypoints that represent the
  // extents of the connection.
  std::vector<DirectedWaypoint> CreateDirectedWaypointsForConnections(
      const DirectedWaypoint& exit, const DirectedWaypoint& entry) const;

  // The tolerance used by the RoadGeometry to check linear invariants.
  const double linear_tolerance_{};
  // The tolerance used by the RoadGeometry to check angular invariants.
  const double angular_tolerance_{};
  // A map to hold all the connections while they are created.
  std::map<std::string, std::vector<std::unique_ptr<Connection>>> connections_;
  // A map to hold all the DirectedWaypoints that are used as Lane extents.
  std::map<std::string, DirectedWaypoint> directed_waypoints_;
  // The coordinates of the bounding box that encloses RNDF's waypoints.
  std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> bounding_box_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
