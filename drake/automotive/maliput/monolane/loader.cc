#include "drake/automotive/maliput/monolane/loader.h"

#include <cmath>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include "yaml-cpp/yaml.h"

#include "drake/common/drake_assert.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace api = drake::maliput::api;
namespace mono = drake::maliput::monolane;

namespace {

api::RBounds rbounds(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return {node[0].as<double>(), node[1].as<double>()};
}


double d2r(double degrees) {
  return degrees * M_PI / 180.;
}


mono::XYPoint xypoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3);
  return {
    node[0].as<double>(), node[1].as<double>(), d2r(node[2].as<double>())};
}


mono::ZPoint zpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 4);
  return {
    node[0].as<double>(), node[1].as<double>(),
        d2r(node[2].as<double>()), d2r(node[3].as<double>())};
}


mono::XYZPoint xyzpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsMap());
  return {xypoint(node["xypoint"]), zpoint(node["zpoint"])};
}


mono::ArcOffset arc_offset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return {node[0].as<double>(), d2r(node[1].as<double>())};
}


std::unique_ptr<mono::XYZPoint> ResolvePointReference(
    const std::string& ref,
    const std::map<std::string, mono::XYZPoint>& xyzpoints) {
  auto parsed = [&](){
    static const std::string kReverse {"reverse "};
    int where = ref.find(kReverse);
    if (where == 0) {
      return std::make_pair(ref.substr(kReverse.size()), true);
    } else {
      return std::make_pair(ref, false);
    }
  }();
  auto it = xyzpoints.find(parsed.first);
  if (it == xyzpoints.end()) {
    return nullptr;
  }
  return std::move(
      std::make_unique<mono::XYZPoint>(
          parsed.second ? it->second.reverse() : it->second));
}


const mono::Connection* maybe_make_connection(
    std::string id,
    YAML::Node node,
    const std::map<std::string, mono::XYZPoint>& xyzpoints,
    mono::Builder* builder) {
  DRAKE_DEMAND(node.IsMap());

  // Check if needed symbols are available.
  std::unique_ptr<mono::XYZPoint> start_point =
      ResolvePointReference(node["start"].as<std::string>(), xyzpoints);
  if (! start_point) { return nullptr; }

  // Discover segment type.
  std::unique_ptr<mono::XYZPoint> ee_point;
  enum SegmentType{ kLine, kArc } segment_type{};
  for (const auto& kv : node) {
    std::string key = kv.first.as<std::string>();

    if (key == "length") {
      segment_type = kLine;
    } else if (key == "arc") {
      segment_type = kArc;
    } else if (key == "explicit_end") {
      // More symbol resolution.
      // TODO(rico): fold this and above into a function.
      ee_point = ResolvePointReference(node["explicit_end"].as<std::string>(),
                                       xyzpoints);
      if (! ee_point) {
        // Skip this connection for now; maybe we can resolve this later.
        return nullptr;
      }
    }
  }
  // TODO(maddog)  Ensure explicit_end and z_end are mutually exclusive.

  // Call appropriate method.
  switch (segment_type) {
    case kLine: {
      if (ee_point) {
        return builder->Connect(
            id, *start_point, node["length"].as<double>(), *ee_point);
      } else {
        return builder->Connect(
            id, *start_point, node["length"].as<double>(),
            zpoint(node["z_end"]));
      }
    }
    case kArc: {
      if (ee_point) {
        return builder->Connect(id, *start_point, arc_offset(node["arc"]),
                                *ee_point);
      } else {
        return builder->Connect(id, *start_point, arc_offset(node["arc"]),
                                zpoint(node["z_end"]));
      }
    }
    default: {
      DRAKE_ABORT();
    }
  }
}


std::unique_ptr<const api::RoadGeometry> BuildFrom(YAML::Node node) {
  DRAKE_DEMAND(node.IsMap());
  YAML::Node mmb = node["maliput_monolane_builder"];
  DRAKE_DEMAND(mmb.IsMap());

  mono::Builder b(rbounds(mmb["lane_bounds"]),
                  rbounds(mmb["driveable_bounds"]),
                  mmb["position_precision"].as<double>(),
                  d2r(mmb["orientation_precision"].as<double>()));

  std::cerr << "loading points !\n";
  YAML::Node points = mmb["points"];
  DRAKE_DEMAND(points.IsMap());
  std::map<std::string, mono::XYZPoint> xyzpoints;
  for (const auto& p : points) {
    xyzpoints[std::string("points.") + p.first.as<std::string>()] =
        xyzpoint(p.second);
  }

  std::cerr << "loading raw connections !\n";
  YAML::Node connections = mmb["connections"];
  DRAKE_DEMAND(connections.IsMap());
  std::map<std::string, YAML::Node> raw_connections;
  for (const auto& c : connections) {
    raw_connections[c.first.as<std::string>()] = c.second;
  }

  std::cerr << "building cooked connections !\n";
  std::map<std::string, const mono::Connection*> cooked_connections;
  while (!raw_connections.empty()) {
    std::cerr << "raw count " << raw_connections.size()
              << " cooked count " << cooked_connections.size() << "\n";
    size_t cooked_before_this_pass = cooked_connections.size();
    for (const auto& r : raw_connections) {
      std::string id = r.first;
      const mono::Connection* conn =
          maybe_make_connection(id, r.second, xyzpoints, &b);
      if (!conn) {
        std::cerr << "...skipping '" << id << "'" << std::endl;
        continue;
      }
      std::cerr << "...cooked '" << id << "'" << std::endl;
      cooked_connections[id] = conn;
      xyzpoints[std::string("connections.") + id + ".start"] = conn->start();
      xyzpoints[std::string("connections.") + id + ".end"] = conn->end();
    }
    DRAKE_DEMAND(cooked_connections.size() > cooked_before_this_pass);
    for (const auto& c : cooked_connections) {
      raw_connections.erase(c.first);
    }
  }

  if (mmb["groups"]) {
    std::cerr << "grouping connections !\n";
    YAML::Node groups = mmb["groups"];
    DRAKE_DEMAND(groups.IsMap());
    std::map<std::string, const mono::Group*> cooked_groups;
    for (const auto& g : groups) {
      const std::string gid = g.first.as<std::string>();
      std::cerr << "   create group '" << gid << "'" << std::endl;
      mono::Group* group = b.MakeGroup(gid);

      YAML::Node cids_node = g.second;
      DRAKE_DEMAND(cids_node.IsSequence());
      for (const YAML::Node& cid_node : cids_node) {
        const std::string cid = cid_node.as<std::string>();
        std::cerr << "      add cnx '" << cid << "'" << std::endl;
        group->Add(cooked_connections[cid]);
      }
    }
  }

  std::cerr << "building road geometry !\n";
  return std::move(b.Build({mmb["id"].Scalar()}));
}

}  // namespace

namespace drake {
namespace maliput {
namespace monolane {

std::unique_ptr<const api::RoadGeometry> Load(const std::string& input) {
  return std::move(BuildFrom(YAML::Load(input)));
}


std::unique_ptr<const api::RoadGeometry> LoadFile(const std::string& filename) {
  return std::move(BuildFrom(YAML::LoadFile(filename)));
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
