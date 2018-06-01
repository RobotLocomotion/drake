#include "drake/automotive/maliput/multilane/loader.h"

#include <cmath>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/builder_spec.h"
#include "drake/automotive/maliput/multilane/connection.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

// Types of Endpoints based on how they are assigned in the YAML.
enum class EndpointType {
  kRaw,        //< A point listed in "points" YAML map.
  kReference,  //< A point that belongs to a connection's reference curve.
  kLane,       //< A point that belongs to a connection's lane curve.
};

// Types of connection's curves.
enum class CurveType {
  kReferenceCurve,  //< Reference curve.
  kLaneCurve,       //< Lane curve.
};

// A structure to hold loaded Endpoints as they are being loaded into the
// catalog.
struct LoadedEndpoint {
  // Type of endpoint.
  EndpointType type{};
  // Parsed endpoint.
  Endpoint endpoint{};
  // When the endpoint is not a free one, it explicitly belongs to a
  // connection's curve. Given that, a curvature can be assigned.
  optional<double> curvature{};
};

// Parses a YAML `node` and returns an api::HBounds object from it.
// `node` must be a sequence and have two doubles. First item will be minimum
// height and the second item will be the maximum height.
api::HBounds h_bounds(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return api::HBounds(node[0].as<double>(), node[1].as<double>());
}

// Converts `degrees` angle into radians.
double deg_to_rad(double degrees) {
  return degrees * M_PI / 180.;
}

// Parses a YAML `node` and returns an EndpointXy object from it.
// `node` must be a sequence of three doubles. First item will be x coordinate,
// second item will be y coordinate and the third item will be heading angle in
// degrees.
EndpointXy endpointxy(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3);
  return EndpointXy(node[0].as<double>(), node[1].as<double>(),
                    deg_to_rad(node[2].as<double>()));
}

// Parses a YAML `node` and returns an EndpointZ object from it.
// `node` must be a sequence of three or four doubles, the last item is not
// mandatory. The first two items are z coordinate and z_dot. The last
// two items are theta angle and theta_dot which are expressed in degrees and,
// degrees per meter respectively.
// theta_dot is optional since continuity constraints may need to be adjusted.
EndpointZ endpointz(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3 || node.size() == 4);
  return node.size() == 4
             ? EndpointZ(node[0].as<double>(), node[1].as<double>(),
                         deg_to_rad(node[2].as<double>()),
                         deg_to_rad(node[3].as<double>()))
             : EndpointZ(node[0].as<double>(), node[1].as<double>(),
                         deg_to_rad(node[2].as<double>()));
}

// Parses a YAML `node` and returns an Endpoint object from it.
// `node` must be a map and contain a sequence node named "xypoint" that
// represents and EndpointXY as well as another sequence node named "zpoint"
// that represents an EndpointZ.
Endpoint endpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsMap());
  return Endpoint(endpointxy(node["xypoint"]), endpointz(node["zpoint"]));
}

// Parses a YAML `node` and returns a tuple object from it.
// `node` must be a sequence of two integers and one double. The first item
// will be the number of lanes in the connection, the second item will be the
// reference lane and the third item will be the distance from the reference
// lane to the reference curve.
std::tuple<int, int, double> lanes(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3);
  const int num_lanes = node[0].as<int>();
  const int ref_lane = node[1].as<int>();
  const double r_ref = node[2].as<double>();
  // Checks that the number of lanes is bigger than 0.
  DRAKE_DEMAND(num_lanes > 0);
  // Checks that the reference lane is within lanes range.
  DRAKE_DEMAND(ref_lane >= 0 && ref_lane < num_lanes);
  return std::make_tuple(num_lanes, ref_lane, r_ref);
}

// Builds a LaneLayout based `node`'s lane node, the left and right shoulders.
LaneLayout ResolveLaneLayout(const YAML::Node& node,
                             double default_left_shoulder,
                             double default_right_shoulder) {
  int num_lanes{0};
  int ref_lane{0};
  double r_ref{0.};
  std::tie(num_lanes, ref_lane, r_ref) = lanes(node["lanes"]);

  // Left and right shoulders are not required, if any of them is present it
  // will override the default value.
  const double left_shoulder = node["left_shoulder"]
                                   ? node["left_shoulder"].as<double>()
                                   : default_left_shoulder;
  const double right_shoulder = node["right_shoulder"]
                                    ? node["right_shoulder"].as<double>()
                                    : default_right_shoulder;
  // Create lane layout.
  return LaneLayout(left_shoulder, right_shoulder, num_lanes, ref_lane, r_ref);
}

// Parses a YAML `node` and returns a LineOffset object from it.
// `node` must be a double scalar.
LineOffset ResolveLineOffset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsScalar());
  return LineOffset(node.as<double>());
}

// Parses a YAML `node` and returns an ArcOffset object from it.
// `node` must be a sequence of two doubles. The first item will be the radius
// and the second item will be the angle span in degrees.
ArcOffset ResolveArcOffset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return ArcOffset(node[0].as<double>(), deg_to_rad(node[1].as<double>()));
}

// Looks for the Endpoint in `xyz_catalog` given `ref` description. `ref` can
// be either a bare Endpoint in "points" YAML map, an Endpoint that references
// a connection's reference curve start / end, or an Endpoint that references a
// connection's lane start / end. Either ".forward" or ".reverse" keyword must
// be present. When ".reverse" is used, the Endpoint will be reversed.
// Otherwise the Endpoint will be returned as is.
// @return An optional<Endpoint> with the Endpoint or nullopt if it is not
// found inside `xyz_catalog`.
optional<LoadedEndpoint> FindEndpointInCatalog(
    const std::string& ref,
    const std::map<std::string, LoadedEndpoint>& xyz_catalog) {
  static const std::string kForward{".forward"};
  static const std::string kReverse{".reverse"};
  const std::string::size_type forward_pos = ref.rfind(kForward);
  const std::string::size_type reverse_pos = ref.rfind(kReverse);
  // Either ".forward" or ".reverse" must be present.
  DRAKE_DEMAND((reverse_pos != std::string::npos) !=
               (forward_pos != std::string::npos));

  const std::string catalog_reference = reverse_pos != std::string::npos
                                            ? ref.substr(0, reverse_pos)
                                            : ref.substr(0, forward_pos);
  auto it = xyz_catalog.find(catalog_reference);
  if (it == xyz_catalog.end()) {
    return nullopt;
  }
  return {{it->second.type,
           reverse_pos != std::string::npos ? it->second.endpoint.reverse()
                                            : it->second.endpoint,
           it->second.curvature}};
}

// Checks that `node` is a sequence of two elements. "ref" or "lane.NB"
// must go first and then a string that points to the Endpoint. It will be
// looked for inside `xyz_catalog`. In addition, the point be assigned to
// either a reference curve or a lane curve.
// @return A std::pair<CurveType, optional<Endpoint>> with the type of
// curve
// and the point information. If the point is not found in `xyz_catalog` a
// nullopt will be returned instead.
std::pair<CurveType, optional<LoadedEndpoint>> ResolveEndpoint(
    const YAML::Node& node,
    const std::map<std::string, LoadedEndpoint>& xyz_catalog) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference the
  //                       lane ID.
  DRAKE_DEMAND(node[0].as<std::string>() == "ref");
  return std::make_pair(
      CurveType::kReferenceCurve,
      FindEndpointInCatalog(node[1].as<std::string>(), xyz_catalog));
}

// Checks that `end_node` is a sequence with two elements. "ref" or "lane.NB"
// must go first and then a string that points to the starting Endpoint. The
// second element will be parsed as an EndpointZ.
// @return A std::pair<CurveType, EndpointZ> whose first element is the type of
// curve and the second one the the EndpointZ to be placed.
std::pair<CurveType, EndpointZ> ResolveEndpointZReference(
    const YAML::Node& end_node) {
  DRAKE_DEMAND(end_node.IsSequence());
  DRAKE_DEMAND(end_node.size() == 2);
  // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference the
  //                       lane ID.
  DRAKE_DEMAND(end_node[0].as<std::string>() == "ref");
  return std::make_pair(CurveType::kReferenceCurve, endpointz(end_node[1]));
}

// Returns a string with the following format:
//
// points.<`point_name`>
std::string PointsKey(const std::string& point_name) {
  return std::string("points.") + point_name;
}

// Parses `points` YAML node to resolve all the endpoints in the collection.
// `points` must be a YAML map.
// `xyz_catalog` will be fed with the parsed endpoints from `points`. Endpoints
// must be fully initialized, i.e. endpoint's theta_dot must be provided.
// `xyz_catalog` must not be nullptr.
void ResolveEndpointsFromPoints(
    const YAML::Node& points,
    std::map<std::string, LoadedEndpoint>* xyz_catalog) {
  DRAKE_DEMAND(points.IsMap());
  DRAKE_DEMAND(xyz_catalog != nullptr);
  for (const auto& p : points) {
    const Endpoint point = endpoint(p.second);
    DRAKE_DEMAND(point.z().theta_dot() != nullopt);
    (*xyz_catalog)[PointsKey(p.first.as<std::string>())] = {
        EndpointType::kRaw, point, {}};
  }
}

// Let theta_dot_A, z_dot_A, K_A be the rate of change in superelevation with
// respect to arc length of the reference path, grade (rate of change of
// elevation with respect to arc length of the reference path) and the
// curvature at one end point of connection's curve A. And let theta_dot_B,
// z_dot_B, K_B be the same magnitudes at one end point of connection's curve
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
  DRAKE_DEMAND(curvature >= 0.);
  DRAKE_DEMAND(endpointz != nullptr);
  endpointz->get_mutable_theta_dot() =
      curvature * std::sin(-std::atan(endpointz->z_dot()));
}

// Depending on the `type` of endpoint `ez_point` is, continuity constraint may
// need to be forced or not. When `type` is Endpoint::kRaw, it means that it
// belongs to either a point in "points" YAML map or it has been inlined in a
// "z_end" YAML sequence and the continuity constraint needs to be computed if
// theta_dot is nullopt.
// When `type` is either KReference or kLane, the continuity constraint needs
// to be computed.
void EvaluateContinuityConstraint(EndpointType type, double curvature,
                                  EndpointZ* ez_point) {
  DRAKE_DEMAND(curvature >= 0.);
  DRAKE_DEMAND(ez_point != nullptr);

  switch (type) {
    case EndpointType::kRaw: {
      if (ez_point->theta_dot() == nullopt) {
        ComputeContinuityConstraint(curvature, ez_point);
      }
      break;
    }
    case EndpointType::kReference: {
      ComputeContinuityConstraint(curvature, ez_point);
      break;
    }
    case EndpointType::kLane: {
      ComputeContinuityConstraint(curvature, ez_point);
      break;
    }
    default: { DRAKE_ABORT(); }
  }
}

// Make a Connection, if all the references in the YAML node can be resolved.
// Otherwise, return a nullptr (meaning, "try again after making some other
// connections").
const Connection* MaybeMakeConnection(
    std::string id, const YAML::Node& node,
    const std::map<std::string, LoadedEndpoint>& xyz_catalog,
    BuilderBase* builder, double default_left_shoulder,
    double default_right_shoulder) {
  DRAKE_DEMAND(node.IsMap());
  DRAKE_DEMAND(builder != nullptr);
  // "lanes" is required.
  DRAKE_DEMAND(node["lanes"]);
  // "start" required.
  DRAKE_DEMAND(node["start"]);
  // "arc" or "length" (but not both) required.
  DRAKE_DEMAND(node["arc"] || node["length"]);
  DRAKE_DEMAND(!(node["arc"] && node["length"]));
  // "z_end" or "explicit_end" (but not both) required.
  DRAKE_DEMAND(node["z_end"] || node["explicit_end"]);
  DRAKE_DEMAND(!(node["z_end"] && node["explicit_end"]));

  // Create lane layout.
  const LaneLayout lane_layout =
      ResolveLaneLayout(node, default_left_shoulder, default_right_shoulder);

  // Define which geometry type the connection is.
  const Connection::Type geometry_type =
      node["length"] ? Connection::kLine : Connection::kArc;

  LineOffset line_offset{};
  ArcOffset arc_offset{};

  // TODO(agalbachicar): Once the Builder can create lane-connected
  //                     segments, curvature for arc geometries will depend on
  //                     the r-offset that a lane has WRT the reference curve.
  double curvature{};
  if (geometry_type == Connection::kLine) {
    line_offset = ResolveLineOffset(node["length"]);
    curvature = 0.;
  } else {
    arc_offset = ResolveArcOffset(node["arc"]);
    curvature = 1. / arc_offset.radius();
  }

  // Resolve start endpoint.
  std::pair<CurveType, optional<LoadedEndpoint>> start_point =
      ResolveEndpoint(node["start"], xyz_catalog);
  if (!(start_point.second)) {
    return nullptr;
  }  // "Try to resolve later."
  // TODO(agalbachicar): Once the Builder can create lane-connected
  //                     segments, this requirement should be removed and
  //                     appropriate continuity constraint must be applied.
  DRAKE_DEMAND(start_point.first == CurveType::kReferenceCurve);
  EvaluateContinuityConstraint(start_point.second->type, curvature,
                               &(start_point.second->endpoint.get_mutable_z()));

  // Resolve end endpoint
  EndpointZ ez_point{};
  if (node["explicit_end"]) {
    std::pair<CurveType, optional<LoadedEndpoint>> end_point =
        ResolveEndpoint(node["explicit_end"], xyz_catalog);
    if (!(end_point.second)) {
      return nullptr;
    }  // "Try to resolve later."
    // TODO(agalbachicar): Once the Builder can create lane-connected
    //                     segments, this requirement should be removed and
    //                     appropriate continuity constraint must be applied.
    DRAKE_DEMAND(end_point.first == CurveType::kReferenceCurve);
    EvaluateContinuityConstraint(end_point.second->type, curvature,
                                 &(end_point.second->endpoint.get_mutable_z()));
    ez_point = end_point.second->endpoint.z();
  } else {
    std::pair<CurveType, EndpointZ> end_z_point =
        ResolveEndpointZReference(node["z_end"]);
    // TODO(agalbachicar): Once the Builder can create lane-connected
    //                     segments, this requirement should be removed and
    //                     appropriate continuity constraint must be applied.
    DRAKE_DEMAND(end_z_point.first == CurveType::kReferenceCurve);
    EvaluateContinuityConstraint(EndpointType::kRaw, curvature,
                                 &(end_z_point.second));
    ez_point = end_z_point.second;
  }

  // Call appropriate Builder method.

  // TODO(agalbachicar)    Once the API of the Builder is finished, support for
  //                       the alternative API is needed and the format may
  //                       suffer some transformations. For the time being,
  //                       this code covers Builder's reference-curve API.
  switch (geometry_type) {
    case Connection::kLine: {
      return builder->Connect(
          id, lane_layout,
          StartReference().at(start_point.second->endpoint,
                              Direction::kForward),
          line_offset, EndReference().z_at(ez_point, Direction::kForward));
    }
    case Connection::kArc: {
      return builder->Connect(
          id, lane_layout,
          StartReference().at(start_point.second->endpoint,
                              Direction::kForward),
          arc_offset, EndReference().z_at(ez_point, Direction::kForward));
    }
    default: {
      DRAKE_ABORT();
    }
  }
}

// Returns a string with the following format:
//
// connections.<`id`>.<start|end>.<`lane_index`>
//
// When `is_start` is true "start" is used, Otherwise, "end" is used.
std::string LaneEndpointKey(const std::string& id, int lane_index,
                            bool is_start) {
  return std::string("connections.") + id + (is_start ? ".start" : ".end") +
         "." + std::to_string(lane_index);
}

// Returns a string with the following format:
//
// connections.<`id`>.<start|end>.ref
//
// When `is_start` is true "start" is used, Otherwise, "end" is used.
std::string ReferenceCurveEndpointKey(const std::string& id, bool is_start) {
  return std::string("connections.") + id +
         (is_start ? ".start.ref" : ".end.ref");
}

// Fills in `xyz_catalog` with all start and end endpoints of `conn`'s curves.
void FillConnectionEndpointsInCatalog(
    const Connection& conn,
    std::map<std::string, LoadedEndpoint>* xyz_catalog) {
  DRAKE_DEMAND(xyz_catalog != nullptr);

  double curvature = conn.ReferenceCurvature();
  (*xyz_catalog)[ReferenceCurveEndpointKey(conn.id(), true)] = {
      EndpointType::kReference, conn.start(), {curvature}};
  (*xyz_catalog)[ReferenceCurveEndpointKey(conn.id(), false)] = {
      EndpointType::kReference, conn.end(), {curvature}};

  for (int i = 0; i < conn.num_lanes(); ++i) {
    curvature = conn.LaneCurvature(i);
    (*xyz_catalog)[LaneEndpointKey(conn.id(), i, true)] = {
        EndpointType::kLane, conn.LaneStart(i), {curvature}};
    (*xyz_catalog)[LaneEndpointKey(conn.id(), i, false)] = {
        EndpointType::kLane, conn.LaneEnd(i), {curvature}};
  }
}

// Parses a YAML `node` that represents a RoadGeometry.
// `node` must be a map and contain a map node called
// "maliput_multilane_builder". This last node must contain the complete
// description of the RoadGeometry.
// "maliput_multilane_builder" map node must have a "points" node as well as a
// "connections" node. "points" map node contains the description of reference
// Endpoints and "connections" describes the Connections. If provided, "groups"
// map node will contain sequences of Groups to join Connections.
std::unique_ptr<const api::RoadGeometry> BuildFrom(
    const BuilderFactoryBase& builder_factory, const YAML::Node& node) {
  DRAKE_DEMAND(node.IsMap());
  YAML::Node mmb = node["maliput_multilane_builder"];
  DRAKE_DEMAND(mmb.IsMap());

  const double default_left_shoulder = mmb["left_shoulder"].as<double>();
  DRAKE_DEMAND(default_left_shoulder >= 0.);
  const double default_right_shoulder = mmb["right_shoulder"].as<double>();
  DRAKE_DEMAND(default_right_shoulder >= 0.);

  const double lane_width = mmb["lane_width"].as<double>();
  DRAKE_DEMAND(lane_width >= 0.);
  const double linear_tolerance = mmb["linear_tolerance"].as<double>();
  DRAKE_DEMAND(linear_tolerance >= 0.);
  const double angular_tolerance =
      deg_to_rad(mmb["angular_tolerance"].as<double>());
  DRAKE_DEMAND(angular_tolerance >= 0.);

  auto builder =
      builder_factory.Make(lane_width, h_bounds(mmb["elevation_bounds"]),
                           linear_tolerance, angular_tolerance);
  DRAKE_DEMAND(builder != nullptr);

  drake::log()->debug("loading points !");
  std::map<std::string, LoadedEndpoint> xyz_catalog;
  ResolveEndpointsFromPoints(mmb["points"], &xyz_catalog);

  drake::log()->debug("loading raw connections !");
  YAML::Node connections = mmb["connections"];
  DRAKE_DEMAND(connections.IsMap());
  std::map<std::string, YAML::Node> raw_connections;
  for (const auto& c : connections) {
    raw_connections[c.first.as<std::string>()] = c.second;
  }

  drake::log()->debug("building cooked connections !");
  std::map<std::string, const Connection*> cooked_connections;
  while (!raw_connections.empty()) {
    drake::log()->debug("raw count {}  cooked count {}", raw_connections.size(),
                        cooked_connections.size());
    const size_t cooked_before_this_pass = cooked_connections.size();
    for (const auto& r : raw_connections) {
      const std::string id = r.first;
      const Connection* conn =
          MaybeMakeConnection(id, r.second, xyz_catalog, builder.get(),
                              default_left_shoulder, default_right_shoulder);
      if (!conn) {
        drake::log()->debug("...skipping '{}'", id);
        continue;
      }
      drake::log()->debug("...cooked '{}'", id);
      cooked_connections[id] = conn;
      // Adds reference curve start / end Endpoints as well as lanes start /
      // end Endpoints.
      FillConnectionEndpointsInCatalog(*conn, &xyz_catalog);
    }
    DRAKE_DEMAND(cooked_connections.size() > cooked_before_this_pass);
    for (const auto& c : cooked_connections) {
      raw_connections.erase(c.first);
    }
  }

  if (mmb["groups"]) {
    drake::log()->debug("grouping connections !");
    YAML::Node groups = mmb["groups"];
    DRAKE_DEMAND(groups.IsMap());
    std::map<std::string, const Group*> cooked_groups;
    for (const auto& g : groups) {
      const std::string gid = g.first.as<std::string>();
      drake::log()->debug("   create group '{}'", gid);
      Group* group = builder->MakeGroup(gid);
      YAML::Node cids_node = g.second;
      DRAKE_DEMAND(cids_node.IsSequence());
      for (const YAML::Node& cid_node : cids_node) {
        const std::string cid = cid_node.as<std::string>();
        drake::log()->debug("      add cnx '{}'", cid);
        group->Add(cooked_connections[cid]);
      }
    }
  }
  drake::log()->debug("building road geometry {}", mmb["id"].Scalar());
  return builder->Build(api::RoadGeometryId{mmb["id"].Scalar()});
}

}  // namespace

std::unique_ptr<const api::RoadGeometry> Load(
    const BuilderFactoryBase& builder_factory, const std::string& input) {
  return BuildFrom(builder_factory, YAML::Load(input));
}

std::unique_ptr<const api::RoadGeometry> LoadFile(
    const BuilderFactoryBase& builder_factory, const std::string& filename) {
  return BuildFrom(builder_factory, YAML::LoadFile(filename));
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
