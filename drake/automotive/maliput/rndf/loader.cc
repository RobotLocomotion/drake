#include "drake/automotive/maliput/rndf/loader.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "ignition/math/SphericalCoordinates.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/Perimeter.hh"
#include "ignition/rndf/RNDF.hh"
#include "ignition/rndf/RNDFNode.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/Zone.hh"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/builder.h"
#include "drake/automotive/maliput/rndf/connection.h"
#include "drake/automotive/maliput/rndf/directed_waypoint.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

namespace {

// Converts the given @p position in UTM (latitude/longitude) coordinates
// to ENU (cartesian) coordinates whose frame is located at @p origin.
// @remarks As the underlying implementation is essentially planar, the
// elevation coordinate will be forced to 0.
// @param origin frame origin in latitude / longitude coordinates.
// @param position position in latitude / longitude coordinates to
// be transformed into @p origin's frame.
// @return @p position at @p origin frame in cartesian coordinates.
ignition::math::Vector3d ToGlobalCoordinates(
    const ignition::math::SphericalCoordinates& origin,
    const ignition::math::SphericalCoordinates& position) {
  const ignition::math::Vector3d position_as_vector(
      position.LatitudeReference().Radian(),
      position.LongitudeReference().Radian(), position.ElevationReference());
  ignition::math::Vector3d position_from_origin = origin.PositionTransform(
      position_as_vector, ignition::math::SphericalCoordinates::SPHERICAL,
      ignition::math::SphericalCoordinates::GLOBAL);
  position_from_origin.Z() = 0;
  return position_from_origin;
}

// Computes the lower left and upper right corners' coordinates of the
// bounding box @p bbox that comprises the whole @p rndf_info map. Coordinates
// are expresed in the cartesian frame located at @p origin.
// @param segments The Segment collection to compute a bounding box for.
// @param origin The global cartesian frame location in latitude / longitude
// coordinates.
// @param bbox The computed bounding box for the given segments.
// @pre The given @p bbox is not a nullptr.
// @warning This function will abort if preconditions are not met.
void BuildBoundingBox(
    const std::vector<ignition::rndf::Segment>& segments,
    const ignition::math::SphericalCoordinates& origin,
    std::pair<ignition::math::Vector3d, ignition::math::Vector3d>* bbox) {
  DRAKE_DEMAND(bbox != nullptr);
  std::vector<DirectedWaypoint> waypoints;
  for (const ignition::rndf::Segment& segment : segments) {
    for (const ignition::rndf::Lane& lane : segment.Lanes()) {
      for (const ignition::rndf::Waypoint& waypoint : lane.Waypoints()) {
        const ignition::math::Vector3d global_location =
            ToGlobalCoordinates(origin, waypoint.Location());
        waypoints.push_back(
            DirectedWaypoint(ignition::rndf::UniqueId(), global_location,
                             ignition::math::Vector3d::Zero, false, false));
      }
    }
  }
  *bbox = DirectedWaypoint::CalculateBoundingBox(waypoints);
}

// Extracts RNDF @p segment_lanes from @p segment, using the given
// @p road_characteristics as needed (i.e. to determine the default width
// for lanes). Coordinates are expressed in the global cartesian frame at
// @p origin.
// @param segment The RNDF segment to extract data from.
// @param origin The global cartesian frame location in latitude / longitude
// coordinates.
// @param default_width The default width for segment lanes, if no nonzero width
// was specified.
// @param segment_lanes The output vector of Connections.
// @pre The given @p segment_lanes collection is not a nullptr.
// @warning This function will abort if preconditions are not met.
void ExtractSegmentLanes(const ignition::rndf::Segment& segment,
                         const ignition::math::SphericalCoordinates& origin,
                         double default_width,
                         std::vector<Connection>* segment_lanes) {
  DRAKE_DEMAND(segment_lanes != nullptr);
  for (const ignition::rndf::Lane& lane : segment.Lanes()) {
    std::vector<DirectedWaypoint> lane_waypoints;
    for (const ignition::rndf::Waypoint& waypoint : lane.Waypoints()) {
      const ignition::math::Vector3d global_location =
          ToGlobalCoordinates(origin, waypoint.Location());
      lane_waypoints.push_back(DirectedWaypoint(
          ignition::rndf::UniqueId(segment.Id(), lane.Id(), waypoint.Id()),
          global_location, ignition::math::Vector3d::Zero, waypoint.IsEntry(),
          waypoint.IsExit()));
    }
    double width = lane.Width();
    if (width == 0.0) width = default_width;
    const std::string id =
        (std::to_string(segment.Id()) + "." + std::to_string(lane.Id()));
    segment_lanes->push_back(Connection(id, lane_waypoints, width, false));
  }
}

// Extracts RNDF zone @p perimeter_waypoints from @p zone as @p. Coordinates
// are expressed in the global cartesian frame at @p origin.
// @param zone The RNDF zone to extract data from.
// @param origin The global cartesian frame location in latitude / longitude
// coordinates.
// @param perimeter_waypoints The RNDF zone perimeter waypoints.
// @pre The given @p perimeter_waypoints collection is not a nullptr.
// @warning This method will abort if preconditions are not met.
void ExtractZonePerimeters(const ignition::rndf::Zone& zone,
                           const ignition::math::SphericalCoordinates& origin,
                           std::vector<DirectedWaypoint>* perimeter_waypoints) {
  DRAKE_DEMAND(perimeter_waypoints != nullptr);
  // Figures out what's the exit lane width.
  const ignition::rndf::Perimeter& perimeter = zone.Perimeter();
  // Retrieves all perimeter waypoints in the global cartesian frame.
  for (const ignition::rndf::Waypoint& waypoint : perimeter.Points()) {
    const ignition::math::Vector3d global_location =
        ToGlobalCoordinates(origin, waypoint.Location());
    const ignition::rndf::UniqueId id(zone.Id(), 0, waypoint.Id());
    perimeter_waypoints->push_back(
        DirectedWaypoint(id, global_location, ignition::math::Vector3d::Zero,
                         waypoint.IsEntry(), waypoint.IsExit()));
  }
}

// Computes the minimum Lane width for an intersection given the @p entry_id
// and @p exit_id of the waypoints that define it, based on the widths of
// the Lanes this intersection would connect.
// @param rndf_info The RNDF map description.
// @param entry_id The intersection's entry waypoint RNDF uid.
// @param exit_id The intersection's exit waypoint RNDF uid.
// @param default_width The default width for intersection lanes, if no nonzero
// width was specified.
// @return The minimum lane width for the intersection.
// @pre The given @p entry_id is a valid lane waypoint uid within @p rndf_info.
// @pre The given @p exit_id is a valid lane waypoint uid within @p rndf_info.
// @warning This method will abort if preconditions are not met.
double ComputeIntersectionWidth(const ignition::rndf::RNDF& rndf_info,
                                const ignition::rndf::UniqueId& entry_id,
                                const ignition::rndf::UniqueId& exit_id,
                                double default_width) {
  const ignition::rndf::RNDFNode* entry_node = rndf_info.Info(entry_id);
  const ignition::rndf::RNDFNode* exit_node = rndf_info.Info(exit_id);
  DRAKE_DEMAND(entry_node != nullptr);
  DRAKE_DEMAND(exit_node != nullptr);
  double width = std::numeric_limits<double>::infinity();
  if (entry_node->Lane() != nullptr) {
    width = std::min(entry_node->Lane()->Width(), width);
  }
  if (exit_node->Lane() != nullptr) {
    width = std::min(exit_node->Lane()->Width(), width);
  }
  if (std::isinf(width) || width == 0) {
    width = default_width;
  }
  return width;
}

// Computes the lane width on a per zone basis.
//
// As RNDF zones do not have a direct mapping to Maliput abstractions, fake
// lanes connect every pair of entry and exit waypoints in the zone perimeter.
// To that end, zone entry and exit waypoints are matched with their exit and
// entry lane waypoints, respectively, and the minimum lane width that would
// yield safe connections on each zone is computed and retrieved.
// @param rndf_info The RNDF map description.
// @param default_width The default width for zone lanes, if no nonzero width
// was specified.
// @param lane_width_per_zone The mapping from RNDF zones uid to zone widths.
// @pre The given @p entry_id is a valid lane waypoint uid within @p rndf_info.
// @pre The given @p lane_width_per_zone mapping is not a nullptr.
// @warning This method will abort if preconditions are not met.
void ComputeZoneLaneWidths(const ignition::rndf::RNDF& rndf_info,
                           double default_width,
                           std::map<int, double>* lane_width_per_zone) {
  DRAKE_DEMAND(lane_width_per_zone != nullptr);
  // Looks up the width of all the lanes that exit into zones.
  for (const ignition::rndf::Segment& segment : rndf_info.Segments()) {
    for (const ignition::rndf::Lane& lane : segment.Lanes()) {
      for (const ignition::rndf::Exit& exit : lane.Exits()) {
        const ignition::rndf::RNDFNode* node = rndf_info.Info(exit.EntryId());
        DRAKE_DEMAND(node != nullptr);
        if (node->Zone() != nullptr) {
          double width = lane.Width();
          if (lane_width_per_zone->count(node->Zone()->Id()) != 0) {
            width = std::min(width, (*lane_width_per_zone)[node->Zone()->Id()]);
          }
          (*lane_width_per_zone)[node->Zone()->Id()] = width;
        }
      }
    }
  }
  // Looks up the width of all the lanes that the zones exit into.
  for (const ignition::rndf::Zone& zone : rndf_info.Zones()) {
    for (const ignition::rndf::Exit& exit : zone.Perimeter().Exits()) {
      const ignition::rndf::UniqueId& waypointId = exit.EntryId();
      if (waypointId.Y() != 0) {
        // Only lane waypoints must be taken into consideration.
        const ignition::rndf::RNDFNode* node = rndf_info.Info(waypointId);
        DRAKE_DEMAND(node != nullptr);
        DRAKE_DEMAND(node->Lane() != nullptr);
        double width = node->Lane()->Width();
        if (lane_width_per_zone->count(zone.Id()) != 0) {
          width = std::min(width, (*lane_width_per_zone)[zone.Id()]);
        }
        (*lane_width_per_zone)[zone.Id()] = width;
      }
    }
    if (lane_width_per_zone->count(zone.Id()) == 0) {
      (*lane_width_per_zone)[zone.Id()] = default_width;
    }
  }
}

}  // namespace

std::unique_ptr<const api::RoadGeometry> Loader::LoadFile(
    const std::string& filepath) const {
  // Attempts to load the given file as an RNDF.
  const ignition::rndf::RNDF rndf_info(filepath);
  DRAKE_THROW_UNLESS(rndf_info.Valid());

  Builder builder(road_characteristics_.linear_tolerance,
                  road_characteristics_.angular_tolerance);

  // Gets the segments in the given RNDF.
  const std::vector<ignition::rndf::Segment>& segments = rndf_info.Segments();
  DRAKE_THROW_UNLESS(segments.size() > 0);
  DRAKE_THROW_UNLESS(segments[0].Lanes().size() > 0);
  DRAKE_THROW_UNLESS(segments[0].Lanes()[0].Waypoints().size() > 0);

  // Gets the location of the first waypoint on the first lane of the first
  // segment and uses it as the origin for Maliput's RoadGeometry cartesian
  // frame.
  const ignition::math::SphericalCoordinates& origin_location =
      segments[0].Lanes()[0].Waypoints()[0].Location();
  std::pair<ignition::math::Vector3d, ignition::math::Vector3d> bounding_box;
  BuildBoundingBox(segments, origin_location, &bounding_box);
  builder.SetBoundingBox(bounding_box);

  // Extracts all segments' lanes and creates the corresponding connections. All
  // segments are built first, zones afterwards, so that all waypoints are
  // known to the Builder before going into further lane connections.
  for (const ignition::rndf::Segment& segment : rndf_info.Segments()) {
    std::vector<Connection> segment_lanes;
    ExtractSegmentLanes(segment, origin_location,
                        road_characteristics_.default_width, &segment_lanes);
    builder.CreateSegmentConnections(segment.Id(), &segment_lanes);
  }

  // Computes each zone's fake inner lanes.
  std::map<int, double> lane_width_per_zone;
  ComputeZoneLaneWidths(rndf_info, road_characteristics_.default_width,
                        &lane_width_per_zone);
  // Extracts zone perimeter's waypoints and creates fake inner lanes between
  // every entry and exit waypoint.
  for (const ignition::rndf::Zone& zone : rndf_info.Zones()) {
    std::vector<DirectedWaypoint> perimeter_waypoints;
    ExtractZonePerimeters(zone, origin_location, &perimeter_waypoints);
    builder.CreateConnectionsForZones(lane_width_per_zone[zone.Id()],
                                      &perimeter_waypoints);
  }

  // Iterates over each lane exit and creates the intersection lane that
  // connects the corresponding exit and entry waypoints.
  for (const ignition::rndf::Segment& segment : rndf_info.Segments()) {
    for (const ignition::rndf::Lane& lane : segment.Lanes()) {
      for (const ignition::rndf::Exit& exit : lane.Exits()) {
        const double width =
            ComputeIntersectionWidth(rndf_info, exit.EntryId(), exit.ExitId(),
                                     road_characteristics_.default_width);
        builder.CreateConnection(width, exit.ExitId(), exit.EntryId());
      }
    }
  }
  // Builds and returns the RoadGeometry for the given RNDF description.
  return builder.Build(api::RoadGeometryId{rndf_info.Name()});
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
