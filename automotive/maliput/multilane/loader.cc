#include "drake/automotive/maliput/multilane/loader.h"

#include <cmath>
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

// Parses a yaml `node` and returns an api::HBounds object from it.
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
EndpointXy endpoint_xy(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3);
  return EndpointXy(node[0].as<double>(), node[1].as<double>(),
                    deg_to_rad(node[2].as<double>()));
}

// Parses a yaml `node` and returns an EndpointZ object from it.
// `node` must be a sequence of three or four doubles, the last item is
// optional. The first two items are z coordinate and z_dot. The last two items
// are theta angle and theta_dot which are expressed in degrees and degrees per
// meter respectively.
// theta_dot is optional since the Builder may need to adjust it to be compliant
// with Maliput's G1 constraint.
EndpointZ endpoint_z(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3 || node.size() == 4);
  // TODO(agalbachicar)    Once EndpointZ::theta_dot becomes a
  //                       drake::optional<double> member, zero should be
  //                       replaced by nullopt.
  return EndpointZ(node[0].as<double>(), node[1].as<double>(),
                   deg_to_rad(node[2].as<double>()),
                   node.size() == 4 ? deg_to_rad(node[3].as<double>()) : 0.);
}

// Parses a yaml `node` and returns an Endpoint object from it.
// `node` must be a map and contain a sequence node named "xypoint" that
// represents and EndpointXY as well as another sequence node named "zpoint"
// that represents an EndpointZ.
Endpoint endpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsMap());
  return Endpoint(endpoint_xy(node["xypoint"]), endpoint_z(node["zpoint"]));
}

// Parses a yaml `node` and returns a LineOffset object from it.
// `node` must be a double scalar.
LineOffset line_offset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsScalar());
  return LineOffset(node.as<double>());
}

// Parses a yaml `node` and returns an ArcOffset object from it.
// `node` must be a sequence of two doubles. The first item will be the radius
// and the second item will be the angle span in degrees.
ArcOffset arc_offset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return ArcOffset(node[0].as<double>(), deg_to_rad(node[1].as<double>()));
}

// Parses a yaml `node` and returns a tuple object from it.
// `node` must be a sequence of two integers and one double. The first item will
// be the number of lanes in the connection, the second item will be the
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

// Looks for the Endpoint in `xyz_catalog` given `ref` description. `ref` can be
// either a bare Endpoint in "points" YAML map, an Endpoint that references a
// connection's reference curve start / end, or an Endpoint that references a
// connection's lane start / end. Either ".forward" or ".reverse" keyword must
// be present. When ".reverse" is used, the Endpoint will be reversed. Otherwise
// the Endpoint will be returned as is.
// @return A drake::optional<Endpoint> with the Endpoint or nullopt if it
// is not found inside `xyz_catalog`.
optional<Endpoint> FindEndpointInCatalog(
    const std::string& ref,
    const std::map<std::string, Endpoint>& xyz_catalog) {
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
  return reverse_pos != std::string::npos ? it->second.reverse() : it->second;
}

// Checks that `node` is a sequence of two elements. "ref" or "lane.NB"
// must go first and then a string that points to the Endpoint. It will be
// looked for inside `xyz_catalog`.
// @return A drake::optional<Endpoint> with the Endpoint or nullopt if it is
// not found inside `xyz_catalog`.
drake::optional<Endpoint> ResolveEndpoint(
    const YAML::Node& node,
    const std::map<std::string, Endpoint>& xyz_catalog) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference the
  //                       lane ID.
  DRAKE_DEMAND(node[0].as<std::string>() == "ref");
  return FindEndpointInCatalog(node[1].as<std::string>(), xyz_catalog);
}

// Checks that `end_node` is a sequence with two elements. "ref" or "lane.NB"
// must go first and then a string that points to the starting Endpoint. The
// second element will be parsed as an EndpointZ.
// @return An EndpointZ.
EndpointZ ResolveEndpointZReference(const YAML::Node& end_node) {
  DRAKE_DEMAND(end_node.IsSequence());
  DRAKE_DEMAND(end_node.size() == 2);
  // TODO(agalbachicar)    Provide support for "lane.NB" so as to reference the
  //                       lane ID.
  DRAKE_DEMAND(end_node[0].as<std::string>() == "ref");
  return endpoint_z(end_node[1]);
}

// Builds a LaneLayout based `node`'s lane node, the left and right
// shoulders.
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

// Make a Connection, if all the references in the yaml node can be resolved.
// Otherwise, return a nullptr (meaning, "try again after making some other
// connections").
const Connection* MaybeMakeConnection(
    std::string id, const YAML::Node& node,
    const std::map<std::string, Endpoint>& xyz_catalog, BuilderBase* builder,
    double default_left_shoulder, double default_right_shoulder) {
  DRAKE_DEMAND(node.IsMap());
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

  // Define which type of connection is going to be built.
  const Connection::Type segment_type =
      node["length"] ? Connection::kLine : Connection::kArc;

  // Create lane layout.
  const LaneLayout lane_layout =
      ResolveLaneLayout(node, default_left_shoulder, default_right_shoulder);

  // Get the start point and build the start spec.
  const drake::optional<Endpoint> start_point =
      ResolveEndpoint(node["start"], xyz_catalog);
  if (!start_point) {
    return nullptr;
  }  // "Try to resolve later."

  // Obtains the end-point information and build the end spec.
  drake::optional<Endpoint> ee_point;
  if (node["explicit_end"]) {
    ee_point = ResolveEndpoint(node["explicit_end"], xyz_catalog);
    if (!ee_point) {
      return nullptr;
    }  // "Try to resolve later."
  }
  const EndpointZ ez_point =
      ee_point ? ee_point->z() : ResolveEndpointZReference(node["z_end"]);

  // Call appropriate Builder method.

  // TODO(agalbachicar)    Once the API of the Builder is finished, support for
  //                       the alternative API is needed and the format may
  //                       suffer some transformations. For the time being, this
  //                       code covers Builder's reference-curve API.
  switch (segment_type) {
    case Connection::kLine: {
      return builder->Connect(
          id, lane_layout,
          StartReference().at(*start_point, Direction::kForward),
          line_offset(node["length"]),
          EndReference().z_at(ez_point, Direction::kForward));
    }
    case Connection::kArc: {
      return builder->Connect(
          id, lane_layout,
          StartReference().at(*start_point, Direction::kForward),
          arc_offset(node["arc"]),
          EndReference().z_at(ez_point, Direction::kForward));
    }
    default: {
      DRAKE_ABORT();
    }
  }
}

// Parses a yaml `node` that represents a RoadGeometry.
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
  YAML::Node points = mmb["points"];
  DRAKE_DEMAND(points.IsMap());
  std::map<std::string, Endpoint> xyz_catalog;
  for (const auto& p : points) {
    xyz_catalog[std::string("points.") + p.first.as<std::string>()] =
        endpoint(p.second);
  }

  drake::log()->debug("loading raw connections !");
  YAML::Node connections = mmb["connections"];
  DRAKE_DEMAND(connections.IsMap());
  std::map<std::string, YAML::Node> raw_connections;
  for (const auto& c : connections) {
    raw_connections[c.first.as<std::string>()] = c.second;
  }

  drake::log()->debug("building cooked connections !");
  std::map<std::string, const Connection*> cooked_connections;
  // Defines some lambdas to create endpoint keys.
  auto endpoint_lane_key = [](const std::string& id, int lane_index,
                              bool is_start) {
    return std::string("connections.") + id + (is_start ? ".start" : ".end") +
           "." + std::to_string(lane_index);
  };
  auto endpoint_ref_curve_key = [](const std::string& id, bool is_start) {
    return std::string("connections.") + id +
           (is_start ? ".start.ref" : ".end.ref");
  };
  while (!raw_connections.empty()) {
    drake::log()->debug("raw count {}  cooked count {}", raw_connections.size(),
                        cooked_connections.size());
    const size_t cooked_before_this_pass = cooked_connections.size();
    for (const auto& r : raw_connections) {
      std::string id = r.first;
      const Connection* conn =
          MaybeMakeConnection(id, r.second, xyz_catalog, builder.get(),
                              default_left_shoulder, default_right_shoulder);
      if (!conn) {
        drake::log()->debug("...skipping '{}'", id);
        continue;
      }
      drake::log()->debug("...cooked '{}'", id);
      cooked_connections[id] = conn;
      // Adds reference curve start / end Endpoints as well as lanes start / end
      // Endpoints.
      xyz_catalog[endpoint_ref_curve_key(id, true)] = conn->start();
      xyz_catalog[endpoint_ref_curve_key(id, false)] = conn->end();
      for (int i = 0; i < conn->num_lanes(); ++i) {
        xyz_catalog[endpoint_lane_key(id, i, true)] = conn->LaneStart(i);
        xyz_catalog[endpoint_lane_key(id, i, false)] = conn->LaneEnd(i);
      }
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
