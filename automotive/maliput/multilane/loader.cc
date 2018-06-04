#include "drake/automotive/maliput/multilane/loader.h"

#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/connection.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

// String literals used several times to parse endpoints. The string reference
// must take one of the following formats:
//
// - Points:        points.P_ID.<forward|reverse>
// - Connections:   connection.C_ID.<start|end>.<ref|LANE_ID>.<forward|reverse>
//
// Where:
// - P_ID is the ID of the Endpoint in "points" YAML map.
// - C_ID is the ID of the Connection.
// - LANE_ID is a non negative integer that refers to C_ID Connection's lane.
const char kConnections[]{"connections."};
const char kPoints[]{"points."};
const char kReference[]{".ref."};
const char kStartKey[]{".start."};
const char kEndKey[]{".end."};
const char kForward[]{".forward"};
const char kReverse[]{".reverse"};

// Converts `degrees` angle into radians.
double deg_to_rad(double degrees) { return degrees * M_PI / 180.; }

// Parses a YAML `node` and returns an api::HBounds object from it.
// `node` must be a sequence and have two doubles. First item will be minimum
// height and the second item will be the maximum height.
api::HBounds ParseHBounds(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return api::HBounds(node[0].as<double>(), node[1].as<double>());
}

// Parses a YAML `node` and returns an EndpointXy object from it.
// `node` must be a sequence of three doubles. First item will be x coordinate,
// second item will be y coordinate and the third item will be heading angle in
// degrees.
EndpointXy ParseEndpointXy(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3);
  return EndpointXy(node[0].as<double>(), node[1].as<double>(),
                    deg_to_rad(node[2].as<double>()));
}

// Parses a YAML `node` and returns an EndpointZ object from it.
// `node` must be a sequence of three or four doubles, the last item is
// optional. The first two items are z coordinate and z_dot. The last
// two items are theta angle and theta_dot which are expressed in degrees and
// degrees per meter respectively.
// theta_dot is optional since, in general, its value must be derived from other
// endpoint/connection parameters in order to preserve continuity.
EndpointZ ParseEndpointZ(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3 || node.size() == 4);
  return EndpointZ(node[0].as<double>(), node[1].as<double>(),
                   deg_to_rad(node[2].as<double>()),
                   node.size() == 4
                       ? optional<double>(deg_to_rad(node[3].as<double>()))
                       : nullopt);
}

// Parses a YAML `node` and returns an Endpoint object from it.
// `node` must be a map and contain a sequence node named "xypoint" that
// represents and EndpointXY as well as another sequence node named "zpoint"
// that represents an EndpointZ.
Endpoint ParseEndpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsMap());
  return Endpoint(ParseEndpointXy(node["xypoint"]),
                  ParseEndpointZ(node["zpoint"]));
}

// Parses a YAML `node` and returns a tuple object from it.
// `node` must be a sequence of two integers and one double. The first item
// will be the number of lanes in the connection, the second item will be the
// reference lane and the third item will be the distance from the reference
// lane to the reference curve.
std::tuple<int, int, double> ParseLanes(const YAML::Node& node) {
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

// Parses a YAML `node` and returns a LineOffset object from it.
// `node` must be a double scalar.
LineOffset ParseLineOffset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsScalar());
  return LineOffset(node.as<double>());
}

// Parses a YAML `node` and returns an ArcOffset object from it.
// `node` must be a sequence of two doubles. The first item will be the radius
// and the second item will be the angle span in degrees.
ArcOffset ParseArcOffset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return ArcOffset(node[0].as<double>(), deg_to_rad(node[1].as<double>()));
}

// Parses a yaml `node` that represents a ComputationPolicy.
// `node` must be a string scalar, with one of the following
// values: "prefer-accuracy", "prefer-speed".
ComputationPolicy ParseComputationPolicy(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsScalar());
  const std::string& policy = node.Scalar();
  if (policy == "prefer-accuracy") {
    return ComputationPolicy::kPreferAccuracy;
  }
  if (policy == "prefer-speed") {
    return ComputationPolicy::kPreferSpeed;
  }
  DRAKE_ABORT();
}

// Builds a LaneLayout based `node`'s lane node, the left and right shoulders.
LaneLayout ResolveLaneLayout(const YAML::Node& node,
                             double default_left_shoulder,
                             double default_right_shoulder) {
  int num_lanes{0};
  int ref_lane{0};
  double r_ref{0.};
  std::tie(num_lanes, ref_lane, r_ref) = ParseLanes(node["lanes"]);

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

// Looks for the Endpoint in `xyz_catalog` given `ref` description. `ref` should
// be a bare Endpoint in "points" YAML map. Either ".forward" or ".reverse"
// keyword must be present.
// @return An optional<Endpoint> with the Endpoint or nullopt if it is not
// found inside `xyz_catalog`.
optional<Endpoint> FindEndpointInCatalog(
    const std::string& ref,
    const std::map<std::string, Endpoint>& xyz_catalog) {
  const std::string::size_type forward_pos = ref.rfind(kForward);
  const std::string::size_type reverse_pos = ref.rfind(kReverse);
  // Either ".forward" or ".reverse" must be present.
  DRAKE_DEMAND((reverse_pos != std::string::npos) !=
               (forward_pos != std::string::npos));

  const std::string catalog_reference = reverse_pos != std::string::npos
                                            ? ref.substr(0, reverse_pos)
                                            : ref.substr(0, forward_pos);
  auto it = xyz_catalog.find(catalog_reference);
  return it == xyz_catalog.end() ? nullopt : optional<Endpoint>(it->second);
}

// Looks for the Connection in `connection_catalog` given `endpoint_key`
// description. `endpoint_key` should be the token specified to reference a
// connection, its curve, the end and the direction.
// @return An optional<const Connection*> with the Connection or nullopt if it
// is not found inside `connection_catalog`.
optional<const Connection*> FindConnectionInCatalog(
    const std::string& endpoint_key,
    const std::map<std::string, const Connection*>& connection_catalog) {
  const std::string::size_type connection_pos = endpoint_key.find(kConnections);
  DRAKE_DEMAND(connection_pos != std::string::npos);

  const std::string::size_type start_pos = endpoint_key.find(kStartKey);
  const std::string::size_type end_pos = endpoint_key.find(kEndKey);
  // Either ".start." or ".end." must be present.
  DRAKE_DEMAND((start_pos != std::string::npos) !=
               (end_pos != std::string::npos));
  const std::string::size_type end_connection_id_pos =
      std::string::npos != start_pos ? start_pos : end_pos;

  const std::string connection_id =
      endpoint_key.substr(std::strlen(kConnections),
                          end_connection_id_pos - std::strlen(kConnections));
  return connection_catalog.find(connection_id) == connection_catalog.end()
             ? nullopt
             : optional<const Connection*>(
                   connection_catalog.at(connection_id));
}

// Returns the Direction that `endpoint_key` sets to the Endpoint / EndpointZ.
Direction ResolveDirection(const std::string& endpoint_key) {
  const std::string::size_type forward_pos = endpoint_key.rfind(kForward);
  const std::string::size_type reverse_pos = endpoint_key.rfind(kReverse);
  // Either ".forward" or ".reverse" must be present.
  DRAKE_DEMAND((reverse_pos != std::string::npos) !=
               (forward_pos != std::string::npos));
  return forward_pos != std::string::npos ? Direction::kForward
                                          : Direction::kReverse;
}

// Returns the Direction that `endpoint_key` sets to the Endpoint / EndpointZ.
api::LaneEnd::Which ResolveEnd(const std::string& endpoint_key) {
  const std::string::size_type start_pos = endpoint_key.find(kStartKey);
  const std::string::size_type end_pos = endpoint_key.find(kEndKey);
  // Either ".start." or ".end." must be present.
  DRAKE_DEMAND((start_pos != std::string::npos) !=
               (end_pos != std::string::npos));
  return start_pos != std::string::npos ? api::LaneEnd::Which::kStart
                                        : api::LaneEnd::Which::kFinish;
}

// Checks that `node` is a sequence of two elements. "ref" or "lane.NB"
// must go first and then a string that points to the Endpoint. It will be
// looked for inside `xyz_catalog` or a Connection will be selected from
// `connection_catalog`.
// @return An optional<StartRefernece::Spec> with the point information when
// it is possible to identify. When the point refers to a Connection that is
// not in `connection_catalog`, nullopt is returned.
// TODO(agalbachicar)  Review return type once lane-to-lane methods in Builder
//                     are supported.
optional<StartReference::Spec> ResolveEndpoint(
    const YAML::Node& node, const std::map<std::string, Endpoint>& xyz_catalog,
    const std::map<std::string, const Connection*>& connection_catalog) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference the
  //                       lane ID.
  DRAKE_DEMAND(node[0].as<std::string>() == "ref");
  DRAKE_DEMAND(node[1].IsScalar());
  const std::string endpoint_key = node[1].as<std::string>();

  // Identifies if it's querying an Endpoint from "points" or from a Connection.
  const std::string::size_type point_pos = endpoint_key.find(kPoints);
  if (point_pos != std::string::npos && point_pos == 0) {
    // Endpoint in "points".
    optional<Endpoint> endpoint =
        FindEndpointInCatalog(endpoint_key, xyz_catalog);
    DRAKE_DEMAND(endpoint.has_value());
    return {
        StartReference().at(endpoint.value(), ResolveDirection(endpoint_key))};
  }
  const std::string::size_type connection_pos = endpoint_key.find(kConnections);
  if (connection_pos != std::string::npos && connection_pos == 0) {
    // Endpoint from a connection.
    optional<const Connection*> connection =
        FindConnectionInCatalog(endpoint_key, connection_catalog);
    if (!connection) {
      return {};
    }
    const std::string::size_type reference_pos = endpoint_key.find(kReference);
    // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference
    //                       the lane ID.
    DRAKE_DEMAND(reference_pos != std::string::npos);

    return {StartReference().at(*connection.value(), ResolveEnd(endpoint_key),
                                ResolveDirection(endpoint_key))};
  }
  DRAKE_ABORT();
}

// Checks that `node` is a sequence of two elements. "ref" or "lane.NB"
// must go first and then a string that points to the EndpointZ, or a sequence
// that represents an EndpointZ.
// When the node has a sequence as its second element, it will be parsed as
// an EndpointZ and Direction::kForward will be assumed. Otherwise, the second
// element will be checked to be a scalar of string type. It must refer to
// either a point in `xyz_catalog` or to an EndpointZ of any Connection in
// `connection_catalog`.
// @return An optional<EndRefernece::Spec> with the point information when
// it is possible to identify. When the point refers to a Connection that is
// not in `connection_catalog`, nullopt is returned.
// TODO(agalbachicar)  Review return type once lane-to-lane methods in Builder
//                     are supported.
optional<EndReference::Spec> ResolveEndpointZ(
    const YAML::Node& node, const std::map<std::string, Endpoint>& xyz_catalog,
    const std::map<std::string, const Connection*>& connection_catalog) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference the
  //                       lane ID.
  DRAKE_DEMAND(node[0].as<std::string>() == "ref");
  if (node[1].IsSequence()) {
    return {EndReference().z_at(ParseEndpointZ(node[1]), Direction::kForward)};
  } else if (node[1].IsScalar()) {
    const std::string endpoint_key = node[1].as<std::string>();
    // Identifies if it's querying an Endpoint from "points" or from a
    // Connection.
    const std::string::size_type point_pos = endpoint_key.find(kPoints);
    if (point_pos != std::string::npos && point_pos == 0) {
      // Endpoint in "points".
      optional<Endpoint> endpoint =
          FindEndpointInCatalog(endpoint_key, xyz_catalog);
      DRAKE_DEMAND(endpoint.has_value());
      return {EndReference().z_at(endpoint.value().z(),
                                  ResolveDirection(endpoint_key))};
    }
    const std::string::size_type connection_pos =
        endpoint_key.find(kConnections);
    if (connection_pos != std::string::npos && connection_pos == 0) {
      // Endpoint from a connection.
      optional<const Connection*> connection =
          FindConnectionInCatalog(endpoint_key, connection_catalog);
      if (!connection) {
        return {};
      }
      const std::string::size_type reference_pos =
          endpoint_key.find(kReference);
      // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference
      //                       the lane ID.
      DRAKE_DEMAND(reference_pos != std::string::npos);
      return {EndReference().z_at(*connection.value(), ResolveEnd(endpoint_key),
                                  ResolveDirection(endpoint_key))};
    }
  }
  DRAKE_ABORT();
}

// Returns a string with the following format:
//
// points.<`point_name`>
std::string PointsKey(const std::string& point_name) {
  return kPoints + point_name;
}

// Parses `points` YAML node to resolve all the endpoints in the collection.
// `points` must be a YAML map.
// `xyz_catalog` will be fed with the parsed endpoints from `points`.
// `xyz_catalog` must not be nullptr.
void ResolveEndpointsFromPoints(const YAML::Node& points,
                                std::map<std::string, Endpoint>* xyz_catalog) {
  DRAKE_DEMAND(points.IsMap());
  DRAKE_DEMAND(xyz_catalog != nullptr);
  for (const auto& p : points) {
    (*xyz_catalog)[PointsKey(p.first.as<std::string>())] =
        ParseEndpoint(p.second);
  }
}

// Make a Connection, if all the references in the YAML node can be resolved.
// Otherwise, return a nullptr (meaning, "try again after making some other
// connections").
const Connection* MaybeMakeConnection(
    std::string id, const YAML::Node& node,
    const std::map<std::string, Endpoint>& xyz_catalog,
    const std::map<std::string, const Connection*>& connection_catalog,
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

  // Resolve start endpoint.
  // TODO(agalbachicar): Once the Builder can create lane-connected
  //                     segments, the return type should handle also
  //                     StartLane::Spec.
  optional<StartReference::Spec> start_spec =
      ResolveEndpoint(node["start"], xyz_catalog, connection_catalog);
  if (!start_spec.has_value()) {
    return nullptr;
  }  // "Try to resolve later."

  // Resolve end endpoint.
  // TODO(agalbachicar): Once the Builder can create lane-connected
  //                     segments, the return type should handle also
  //                     EndLane::Spec.
  optional<EndReference::Spec> end_spec = ResolveEndpointZ(
      node["explicit_end"] ? node["explicit_end"] : node["z_end"], xyz_catalog,
      connection_catalog);
  if (!end_spec.has_value()) {
    return nullptr;
  }  // "Try to resolve later."

  // TODO(agalbachicar)    Once the API of the Builder is finished, support for
  //                       the alternative API is needed and the format may
  //                       suffer some transformations. For the time being,
  //                       this code covers Builder's reference-curve API.
  switch (geometry_type) {
    case Connection::kLine: {
      return builder->Connect(id, lane_layout, start_spec.value(),
                              ParseLineOffset(node["length"]),
                              end_spec.value());
    }
    case Connection::kArc: {
      return builder->Connect(id, lane_layout, start_spec.value(),
                              ParseArcOffset(node["arc"]), end_spec.value());
    }
    default: {
      DRAKE_ABORT();
    }
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
  const double scale_length = mmb["scale_length"].as<double>();
  DRAKE_DEMAND(scale_length > 0.);
  const ComputationPolicy computation_policy =
      ParseComputationPolicy(mmb["computation_policy"]);

  auto builder = builder_factory.Make(
      lane_width, ParseHBounds(mmb["elevation_bounds"]), linear_tolerance,
      angular_tolerance, scale_length, computation_policy);
  DRAKE_DEMAND(builder != nullptr);

  drake::log()->debug("loading points !");
  std::map<std::string, Endpoint> xyz_catalog;
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
      const Connection* conn = MaybeMakeConnection(
          id, r.second, xyz_catalog, cooked_connections, builder.get(),
          default_left_shoulder, default_right_shoulder);
      if (!conn) {
        drake::log()->debug("...skipping '{}'", id);
        continue;
      }
      drake::log()->debug("...cooked '{}'", id);
      cooked_connections[id] = conn;
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
