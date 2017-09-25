#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/connection.h"
#include "drake/automotive/maliput/rndf/directed_waypoint.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// A class to ease the construction of a RoadGeometry from Connection and
/// DirectedWaypoint objects.
///
/// RNDF segments and lanes are mapped to Maliput's Segment and Lane entities,
/// respectively. This mapping is not straightforward as Maliput is based on
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
/// Since lanes in an RNDF segment may flow in different directions, segment
/// connections are also grouped by their directions relative to the first
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
/// RNDF zones also do not have a direct representation in Maliput either.
/// To model zones, fake connections are added between every pair of entry
/// and exit waypoints in a zone. The directions of these waypoints are set to
/// head towards the centroid of all the zone perimeter waypoints. There's
/// currently no support for RNDF zones' parking spots.
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
/// Note that '+' denotes RNDF waypoints, 'x' denotes invalid waypoints
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
///   - Lane 1.2.3-1.2.4
///   - Lane 1.1.1-1.1.2
/// - Segment 1-0-2:
///   - Lane 1.2.4-1.2.2
///
/// General workflow with this class should be:
/// -# Create a Builder.
/// -# Call SetBoundingBox().
/// -# Call CreateSegmentConnections() for each RNDF segment.
/// -# Call CreateConnectionsForZones() for each RNDF zone.
/// -# Call CreateConnection() for each pair of entry-exit waypoints
///    between RNDF lanes and zone perimeters (actually connecting
///    their fake inner lanes to the outer real ones).
/// -# Call Build() to get the built api::RoadGeometry.
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
  /// @param bounding_box The lower left and upper right corners' position
  /// pair for the bounding box.
  /// @remarks Bounding box definition is kept in 3D space for the sake
  /// of generality, even though there's currently no support for nonplanar
  /// RNDF geometries and, most of the time, the z-component of the given
  /// corners will be zero.
  void SetBoundingBox(const std::pair<ignition::math::Vector3d,
                                      ignition::math::Vector3d>& bounding_box) {
    bounding_box_ = bounding_box;
  }

  /// Populates the Builder's inner connection map with the given
  /// @p connections representing an RNDF segment.
  ///
  /// To do this, the @p connections' waypoints are first used to derive
  /// a geometry. Then, these @p connections are grouped based on relative
  /// direction using the first connection found as a reference. Once grouped,
  /// extra waypoints are added to each of them on a per group basis as
  /// necessary to ensure a grid-like distribution of waypoints.
  /// @param segment_id The RNDF segment ID.
  /// @param connections A collection of Connections representing each RNDF lane
  /// in the segment.
  /// @throw std::runtime_error When @p connections is a nullptr.
  /// @throw std::runtime_error When @p connections is an empty collection.
  void CreateSegmentConnections(int segment_id,
                                std::vector<Connection>* connections);

  /// Creates a collection of Connection objects between every pair of entry and
  /// exit waypoints in @p perimeter_waypoints.
  ///
  /// RNDF defines zones as areas where free-path driving is allowed. Since
  /// Maliput does not cover this concept, and to avoid having dead-end lanes,
  /// every pair of entry and exit waypoints is connected. These waypoints'
  /// directions are set to head towards the centroid of all
  /// @p perimeter_waypoints.
  /// @param width The width of this zone's inner connections (and thus the
  /// fake inner lanes' lane bounds).
  /// @param perimeter_waypoints A collection of DirectedWaypoint objects
  /// describing the zone's perimeter.
  /// @throw std::runtime_error When @p perimeter_waypoints is a nullptr.
  /// @throw std::runtime_error When @p perimeter_waypoints is an empty
  /// collection.
  void CreateConnectionsForZones(
      double width, std::vector<DirectedWaypoint>* perimeter_waypoints);

  /// Creates a connection between two RNDF lanes based on a pair of @p exit and
  /// @p entry ids that map to specific, existing waypoints. This is helpful
  /// when building intersections.
  /// @param width The connection's width.
  /// @param exit_id The start waypoint ID of the connection.
  /// @param entry_id The end waypoint ID of the connection.
  /// @throw std::runtime_error When neither @p exit_id nor @p entry_id are
  /// found.
  void CreateConnection(double width, const ignition::rndf::UniqueId& exit_id,
                        const ignition::rndf::UniqueId& entry_id);

  /// Builds an api::RoadGeometry.
  ///
  /// All the groups of connections are traversed. A Junction with a single
  /// Segment is created per group, and for each connection in that group,
  /// a Lane is added to the Segment. BranchPoints are updated as needed.
  /// @param id ID of the api::RoadGeometry to be built.
  /// @return The built api::RoadGeometry.
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

  // The tolerance used by the RoadGeometry to check linear invariants.
  const double linear_tolerance_{};
  // The tolerance used by the RoadGeometry to check angular invariants.
  const double angular_tolerance_{};
  // A map to hold all the connections while they are created.
  std::map<std::string, std::vector<std::unique_ptr<Connection>>> connections_;
  // A map to hold all the DirectedWaypoints that are used as Lane extents.
  std::map<std::string, DirectedWaypoint> directed_waypoints_;
  // The coordinates of the bounding box that encloses RNDF's waypoints.
  std::pair<ignition::math::Vector3d, ignition::math::Vector3d> bounding_box_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
