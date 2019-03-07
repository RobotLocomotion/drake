#include "drake/automotive/maliput/multilane/loader.h"

#include <cmath>
#include <cstring>
#include <map>
#include <regex>
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

// Defines if the endpoint refers to "points" map or one of the connection
// points.
enum class ReferenceType {
  kPoint,
  kConnection,
};

// Holds the parsed information of point reference.
struct ParsedReference {
  // Type of endpoint.
  ReferenceType type;
  // When `type` == kPoint, it is ID of an Endpoint in "points".
  // When `type` == kConnection, it is the ID of a Connection.
  std::string id;
  // Endpoint's direction.
  Direction direction;
  // When `type` == kConnection, is one of the extents of the Connection.
  // Otherwise, it must be nullopt.
  optional<api::LaneEnd::Which> end;
  // When `type` == kConnection, is `id` connection's lane index", or nullopt
  // if referring to a connection's reference curve.
  optional<int> lane_id;
};

// Enumerates the types of curve within a connection where endpoints may be
// laid.
enum class AnchorPointType {
  kReference,  //< Reference curve.
  kLane,       //< Lane curve.
};

// Holds the parsed information of the curve the endpoint refers to.
struct ParsedAnchorPoint {
  // Type of curve.
  AnchorPointType type;
  // When `type` == kLane, is the lane ID that the endpoint refers to, or
  // nullopt if referring to a connection's reference curve.
  optional<int> lane_id;
};

// TODO(agalbachicar)    Once std::variant is added, move this to:
// using StartSpec = std::optional<std::variant<StartReference::Spec,
//                                              StartLane::Spec>>
struct StartSpec {
  optional<StartReference::Spec> ref_spec;
  optional<StartLane::Spec> lane_spec;
};

// TODO(agalbachicar)    Once std::variant is added, move this to:
// using EndSpec = std::optional<std::variant<EndReference::Spec,
//                                            EndLane::Spec>>
struct EndSpec {
  optional<EndReference::Spec> ref_spec;
  optional<EndLane::Spec> lane_spec;
};

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
  throw std::runtime_error("Unknown ComputationPolicy");
}

// Builds a LaneLayout based `node`'s lane node, the left and right shoulders.
LaneLayout ResolveLaneLayout(const YAML::Node& node,
                             double default_left_shoulder,
                             double default_right_shoulder) {
  int num_lanes{};
  int ref_lane{};
  double r_ref{};
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

// Looks for the Endpoint in `point_catalog` given `endpoint_key` description.
// `endpoint_key` should be a bare Endpoint in "points" YAML map.
// @return An optional<Endpoint> with the Endpoint or nullopt if it is not
// found inside `point_catalog`.
optional<Endpoint> FindEndpointInCatalog(
    const std::string& endpoint_key,
    const std::map<std::string, Endpoint>& point_catalog) {
  auto it = point_catalog.find(endpoint_key);
  return it == point_catalog.end() ? nullopt : optional<Endpoint>(it->second);
}

// Looks for the Connection in `connection_catalog` given `connection_key`
// description. `connection_key` should be the token specified to reference a
// connection, its reference curve or one of its lanes, the end and the
// direction.
// @return An optional<const Connection*> with the Connection or nullopt if it
// is not found inside `connection_catalog`.
optional<const Connection*> FindConnectionInCatalog(
    const std::string& connection_key,
    const std::map<std::string, const Connection*>& connection_catalog) {
  return connection_catalog.find(connection_key) == connection_catalog.end()
             ? nullopt
             : optional<const Connection*>(
                   connection_catalog.at(connection_key));
}

// Returns the Direction that `direction_key` sets to the Endpoint / EndpointZ.
// `direction_key` must be either "forward" or "reverse".
Direction ResolveDirection(const std::string& direction_key) {
  if (direction_key == "forward") {
    return Direction::kForward;
  } else if (direction_key == "reverse") {
    return Direction::kReverse;
  }
  throw std::runtime_error("Unknown direction_key");
}

// Returns the Direction that `end_key` sets to the Endpoint / EndpointZ.
// `end_key` must be either "start" or "end".
api::LaneEnd::Which ResolveEnd(const std::string& end_key) {
  if (end_key == "start") {
    return api::LaneEnd::Which::kStart;
  } else if (end_key == "end") {
    return api::LaneEnd::Which::kFinish;
  }
  throw std::runtime_error("Unknown end_key");
}

// Returns a ParsedReference structure by extracting keywords from
// `endpoint_key`.
//
// `endpoint_key` must match one of the following formats:
//
// - Points:        points.P_ID.<forward|reverse>
// - Connections:   connections.C_ID.<start|end>.<ref|LANE_ID>.<forward|reverse>
//
// Where:
// - P_ID is the ID of the Endpoint in "points" YAML map.
// - C_ID is the ID of the Connection.
// - LANE_ID is a non negative integer that refers to C_ID Connection's lane.
ParsedReference ResolveEndpointReference(const std::string& endpoint_key) {
  ParsedReference parsed_reference{};
  static const std::regex kPointsPattern{"^points[.](.*)[.](forward|reverse)$",
                                         std::regex::ECMAScript};
  static const std::regex kConnectionsPattern{
      "^connections[.](.*)[.](start|end)[.](ref|[0-9]*)[.](forward|reverse)$",
      std::regex::ECMAScript};
  std::smatch pieces_match;
  if (std::regex_match(endpoint_key, pieces_match, kPointsPattern)) {
    parsed_reference.type = ReferenceType::kPoint;
    parsed_reference.id = pieces_match[1].str();
    parsed_reference.direction = ResolveDirection(pieces_match[2].str());
  } else if (std::regex_match(endpoint_key, pieces_match,
                              kConnectionsPattern)) {
    parsed_reference.type = ReferenceType::kConnection;
    parsed_reference.id = pieces_match[1].str();
    parsed_reference.direction = ResolveDirection(pieces_match[4].str());
    parsed_reference.end = {ResolveEnd(pieces_match[2].str())};
    parsed_reference.lane_id = pieces_match[3].str() == "ref"
        ? nullopt : optional<int>(std::atoi(pieces_match[3].str().c_str()));
  } else {
    throw std::runtime_error("Unknown endpoint reference");
  }
  return parsed_reference;
}

// Returns a ParsedAnchorPoint structure by extracting keywords from
// `anchor_key`.
//
// `anchor_key` must match one of the following formats:
//
// - Reference curve:  ref
// - Lane curve:       lane.LANE_ID
//
// Where:
// - LANE_ID is a non negative integer that refers to connection lane's ID.
ParsedAnchorPoint ParseAnchorPoint(const std::string anchor_key) {
  if (anchor_key == "ref") {
    return {AnchorPointType::kReference, {}};
  }
  const auto it = anchor_key.find("lane.");
  DRAKE_DEMAND(it != std::string::npos && it == 0);
  const std::string lane_str =
      anchor_key.substr(std::strlen("lane."),
                        anchor_key.length() - std::strlen("lane."));
  // Checks that lane_str only contains digits but no other character.
  DRAKE_DEMAND(lane_str.find_first_not_of("0123456789") == std::string::npos);
  const int lane_id = std::stoi(lane_str);
  return {AnchorPointType::kLane, {lane_id}};
}

// Checks that `node` is a sequence of two elements. "ref" or "lane.LANE_ID"
// must go first and then a string that points to the Endpoint. It will be
// looked for inside `point_catalog` or a Connection will be selected from
// `connection_catalog`.
// @return An optional<std::pair<ParsedCurve, StartSpec>> with the point
// information when it is possible to identify. When the point refers to a
// Connection that is not in `connection_catalog`, nullopt is returned.
optional<std::pair<ParsedAnchorPoint, StartSpec>> ResolveEndpoint(
    const YAML::Node& node,
    const std::map<std::string, Endpoint>& point_catalog,
    const std::map<std::string, const Connection*>& connection_catalog) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  DRAKE_DEMAND(node[1].IsScalar());

  const ParsedAnchorPoint parsed_anchor_point =
      ParseAnchorPoint(node[0].as<std::string>());

  const std::string endpoint_key = node[1].as<std::string>();
  const ParsedReference parsed_reference =
      ResolveEndpointReference(endpoint_key);

  StartSpec spec{};
  if (parsed_reference.type == ReferenceType::kPoint) {
    optional<Endpoint> endpoint =
        FindEndpointInCatalog(parsed_reference.id, point_catalog);
    DRAKE_DEMAND(endpoint.has_value());
    if (parsed_anchor_point.type == AnchorPointType::kReference) {
      spec.ref_spec =
          StartReference().at(endpoint.value(), parsed_reference.direction);
    } else {
      spec.lane_spec =
          StartLane(parsed_anchor_point.lane_id.value()).at(
              endpoint.value(), parsed_reference.direction);
    }
  } else if (parsed_reference.type == ReferenceType::kConnection) {
    optional<const Connection*> connection =
        FindConnectionInCatalog(parsed_reference.id, connection_catalog);
    if (!connection) {
      return nullopt;
    }
    if (parsed_anchor_point.type == AnchorPointType::kReference) {
      DRAKE_DEMAND(!parsed_reference.lane_id.has_value());
      spec.ref_spec = StartReference().at(*connection.value(),
                                          parsed_reference.end.value(),
                                          parsed_reference.direction);
    } else {
      DRAKE_DEMAND(parsed_reference.lane_id.has_value());
      spec.lane_spec = StartLane(parsed_anchor_point.lane_id.value()).at(
          *connection.value(), parsed_reference.lane_id.value(),
          parsed_reference.end.value(), parsed_reference.direction);
    }
  } else {
    throw std::runtime_error("Unknown endpoint");
  }
  return {{parsed_anchor_point, spec}};
}

// Checks that `node` is a sequence of two elements. "ref" or "lane.LANE_ID"
// must go first and then a string that points to the EndpointZ, or a sequence
// that represents an EndpointZ.
// When the node has a sequence as its second element, it will be parsed as
// an EndpointZ and Direction::kForward will be assumed. Otherwise, the second
// element will be checked to be a scalar of string type. It must refer to
// either a point in `point_catalog` or to an EndpointZ of any Connection in
// `connection_catalog`.
// @return An optional<std::pair<ParsedCurve, EndSpec>> with the point
// information when it is possible to identify. When the point refers to a
//  Connection that is not in `connection_catalog`, nullopt is returned.
optional<std::pair<ParsedAnchorPoint, EndSpec>> ResolveEndpointZ(
    const YAML::Node& node,
    const std::map<std::string, Endpoint>& point_catalog,
    const std::map<std::string, const Connection*>& connection_catalog) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);

  const ParsedAnchorPoint parsed_anchor_point =
      ParseAnchorPoint(node[0].as<std::string>());
  EndSpec spec{};

  if (node[1].IsSequence()) {
    if (parsed_anchor_point.type == AnchorPointType::kReference) {
      spec.ref_spec =
          EndReference().z_at(ParseEndpointZ(node[1]), Direction::kForward);
    } else {
      spec.lane_spec =
          EndLane(parsed_anchor_point.lane_id.value()).z_at(
              ParseEndpointZ(node[1]), Direction::kForward);
    }
  } else if (node[1].IsScalar()) {
    const std::string endpoint_key = node[1].as<std::string>();
    const ParsedReference parsed_reference =
        ResolveEndpointReference(endpoint_key);

    if (parsed_reference.type == ReferenceType::kPoint) {
      optional<Endpoint> endpoint =
          FindEndpointInCatalog(parsed_reference.id, point_catalog);
      DRAKE_DEMAND(endpoint.has_value());
      if (parsed_anchor_point.type == AnchorPointType::kReference) {
        spec.ref_spec =
            EndReference().z_at(endpoint.value().z(),
                                 parsed_reference.direction);
      } else {
        spec.lane_spec =
            EndLane(parsed_anchor_point.lane_id.value()).z_at(
                endpoint.value().z(), parsed_reference.direction);
      }
    } else if (parsed_reference.type == ReferenceType::kConnection) {
      optional<const Connection*> connection =
          FindConnectionInCatalog(parsed_reference.id, connection_catalog);
      if (!connection) {
        return nullopt;
      }
      if (parsed_anchor_point.type == AnchorPointType::kReference) {
        DRAKE_DEMAND(!parsed_reference.lane_id.has_value());
        spec.ref_spec = EndReference().z_at(*connection.value(),
                                            parsed_reference.end.value(),
                                            parsed_reference.direction);
      } else {
        DRAKE_DEMAND(parsed_reference.lane_id.has_value());
        spec.lane_spec = EndLane(parsed_anchor_point.lane_id.value()).z_at(
            *connection.value(), parsed_reference.lane_id.value(),
            parsed_reference.end.value(), parsed_reference.direction);
      }
    } else {
      throw std::runtime_error("Unknown EndpointZ scalar");
    }
  } else {
    throw std::runtime_error("Unknown EndpointZ node type");
  }
  return {{parsed_anchor_point, spec}};
}

// Parses `points` YAML node to resolve all the endpoints in the collection.
// `points` must be a YAML map.
// `point_catalog` will be fed with the parsed endpoints from `points`.
// `point_catalog` must not be nullptr.
void ParseEndpointsFromPoints(const YAML::Node& points,
                              std::map<std::string, Endpoint>* point_catalog) {
  DRAKE_DEMAND(points.IsMap());
  DRAKE_DEMAND(point_catalog != nullptr);
  for (const auto& p : points) {
    (*point_catalog)[p.first.as<std::string>()] = ParseEndpoint(p.second);
  }
}

// Make a Connection, if all the references in the YAML node can be resolved.
// Otherwise, return a nullptr (meaning, "try again after making some other
// connections").
const Connection* MaybeMakeConnection(
    std::string id, const YAML::Node& node,
    const std::map<std::string, Endpoint>& point_catalog,
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
  optional<std::pair<ParsedAnchorPoint, StartSpec>> start_spec =
      ResolveEndpoint(node["start"], point_catalog, connection_catalog);
  if (!start_spec.has_value()) {
    return nullptr;
  }  // "Try to resolve later."

  // Resolve end endpoint.
  // TODO(agalbachicar)  It is needed an assertion to make sure that when
  //                     "explicit_end" is used, the Endpoint it refers to
  //                     actually matches within linear and angular tolerance
  //                     the new Connection's end Endpoint.
  optional<std::pair<ParsedAnchorPoint, EndSpec>> end_spec = ResolveEndpointZ(
      node["explicit_end"] ? node["explicit_end"] : node["z_end"],
      point_catalog, connection_catalog);
  if (!end_spec.has_value()) {
    return nullptr;
  }  // "Try to resolve later."

  // Both ends must be of the same type.
  DRAKE_DEMAND(start_spec->first.type == end_spec->first.type);

  switch ((*start_spec).first.type) {
    case AnchorPointType::kReference: {
      switch (geometry_type) {
        case Connection::kLine: {
          return builder->Connect(id, lane_layout,
                                  *(start_spec->second.ref_spec),
                                  ParseLineOffset(node["length"]),
                                  *(end_spec->second.ref_spec));
        }
        case Connection::kArc: {
          return builder->Connect(id, lane_layout,
                                  *(start_spec->second.ref_spec),
                                  ParseArcOffset(node["arc"]),
                                  *(end_spec->second.ref_spec));
        }
      }
      DRAKE_UNREACHABLE();
    }
    case AnchorPointType::kLane: {
      switch (geometry_type) {
        case Connection::kLine: {
          return builder->Connect(id, lane_layout,
                                  *(start_spec->second.lane_spec),
                                  ParseLineOffset(node["length"]),
                                  *(end_spec->second.lane_spec));
        }
        case Connection::kArc: {
          return builder->Connect(id, lane_layout,
                                  *(start_spec->second.lane_spec),
                                  ParseArcOffset(node["arc"]),
                                  *(end_spec->second.lane_spec));
        }
      }
      DRAKE_UNREACHABLE();
    }
  }
  DRAKE_UNREACHABLE();
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

  DRAKE_SPDLOG_DEBUG(drake::log(), "loading points !");
  std::map<std::string, Endpoint> point_catalog;
  ParseEndpointsFromPoints(mmb["points"], &point_catalog);

  DRAKE_SPDLOG_DEBUG(drake::log(), "loading raw connections !");
  YAML::Node connections = mmb["connections"];
  DRAKE_DEMAND(connections.IsMap());
  std::map<std::string, YAML::Node> raw_connections;
  for (const auto& c : connections) {
    raw_connections[c.first.as<std::string>()] = c.second;
  }

  DRAKE_SPDLOG_DEBUG(drake::log(), "building cooked connections !");
  std::map<std::string, const Connection*> cooked_connections;
  while (!raw_connections.empty()) {
    DRAKE_SPDLOG_DEBUG(drake::log(), "raw count {}  cooked count {}",
                       raw_connections.size(), cooked_connections.size());
    const size_t cooked_before_this_pass = cooked_connections.size();
    for (const auto& r : raw_connections) {
      const std::string id = r.first;
      const Connection* conn = MaybeMakeConnection(
          id, r.second, point_catalog, cooked_connections, builder.get(),
          default_left_shoulder, default_right_shoulder);
      if (!conn) {
        DRAKE_SPDLOG_DEBUG(drake::log(), "...skipping '{}'", id);
        continue;
      }
      DRAKE_SPDLOG_DEBUG(drake::log(), "...cooked '{}'", id);
      cooked_connections[id] = conn;
    }
    DRAKE_DEMAND(cooked_connections.size() > cooked_before_this_pass);
    for (const auto& c : cooked_connections) {
      raw_connections.erase(c.first);
    }
  }

  if (mmb["groups"]) {
    DRAKE_SPDLOG_DEBUG(drake::log(), "grouping connections !");
    YAML::Node groups = mmb["groups"];
    DRAKE_DEMAND(groups.IsMap());
    std::map<std::string, const Group*> cooked_groups;
    for (const auto& g : groups) {
      const std::string gid = g.first.as<std::string>();
      DRAKE_SPDLOG_DEBUG(drake::log(), "   create group '{}'", gid);
      Group* group = builder->MakeGroup(gid);
      YAML::Node cids_node = g.second;
      DRAKE_DEMAND(cids_node.IsSequence());
      for (const YAML::Node& cid_node : cids_node) {
        const std::string cid = cid_node.as<std::string>();
        DRAKE_SPDLOG_DEBUG(drake::log(), "      add cnx '{}'", cid);
        group->Add(cooked_connections[cid]);
      }
    }
  }
  DRAKE_SPDLOG_DEBUG(drake::log(), "building road geometry {}",
                     mmb["id"].Scalar());
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
