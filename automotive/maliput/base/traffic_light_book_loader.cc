#include "drake/automotive/maliput/base/traffic_light_book_loader.h"

#include <unordered_map>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/rules/traffic_light_book.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/automotive/maliput/base/traffic_light_book.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

using drake::Quaternion;
using drake::maliput::api::GeoPosition;
using drake::maliput::api::Rotation;
using drake::maliput::api::rules::Bulb;
using drake::maliput::api::rules::BulbColor;
using drake::maliput::api::rules::BulbGroup;
using drake::maliput::api::rules::BulbState;
using drake::maliput::api::rules::BulbType;
using drake::maliput::api::rules::TrafficLight;

namespace YAML {

template <>
struct convert<GeoPosition> {
  static Node encode(const GeoPosition& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, GeoPosition& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }
    rhs.set_x(node[0].as<double>());
    rhs.set_y(node[1].as<double>());
    rhs.set_z(node[2].as<double>());
    return true;
  }
};

template <>
struct convert<Rotation> {
  static Node encode(const Rotation& rhs) {
    const Quaternion<double>& q = rhs.quat();
    Node node;
    node.push_back(q.w());
    node.push_back(q.x());
    node.push_back(q.y());
    node.push_back(q.z());
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, Rotation& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }
    const Quaternion<double> q(node[0].as<double>(), node[1].as<double>(),
                               node[2].as<double>(), node[3].as<double>());
    rhs.set_quat(q);
    return true;
  }
};

template <>
struct convert<Bulb::BoundingBox> {
  static Node encode(const Bulb::BoundingBox& rhs) {
    Node min_node;
    min_node.push_back(rhs.p_BMin.x());
    min_node.push_back(rhs.p_BMin.y());
    min_node.push_back(rhs.p_BMin.z());

    Node max_node;
    max_node.push_back(rhs.p_BMax.x());
    max_node.push_back(rhs.p_BMax.y());
    max_node.push_back(rhs.p_BMax.z());

    Node node;
    node["min"] = min_node;
    node["max"] = max_node;

    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, Bulb::BoundingBox& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    const Node& min_node = node["min"];
    if (!min_node.IsDefined() || !min_node.IsSequence() ||
        min_node.size() != 3) {
      return false;
    }
    const Node& max_node = node["max"];
    if (!max_node.IsDefined() || !max_node.IsSequence() ||
        max_node.size() != 3) {
      return false;
    }
    rhs.p_BMin =
        Eigen::Vector3d(min_node[0].as<double>(), min_node[1].as<double>(),
                        min_node[2].as<double>());
    rhs.p_BMax =
        Eigen::Vector3d(max_node[0].as<double>(), max_node[1].as<double>(),
                        max_node[2].as<double>());
    return true;
  }
};

template <>
struct convert<BulbColor> {
  static Node encode(const BulbColor& rhs) {
    Node node;
    node.push_back(drake::maliput::api::rules::BulbColorMapper().at(rhs));
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, BulbColor& rhs) {
    const std::string color = node.as<std::string>();
    bool result = false;
    for (const auto& it : drake::maliput::api::rules::BulbColorMapper()) {
      if (it.second == color) {
        rhs = it.first;
        result = true;
      }
    }
    return result;
  }
};

template <>
struct convert<BulbType> {
  static Node encode(const BulbType& rhs) {
    Node node;
    node.push_back(drake::maliput::api::rules::BulbTypeMapper().at(rhs));
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, BulbType& rhs) {
    const std::string type = node.as<std::string>();
    bool result = false;
    for (const auto& it : drake::maliput::api::rules::BulbTypeMapper()) {
      if (it.second == type) {
        rhs = it.first;
        result = true;
      }
    }
    return result;
  }
};

template <>
struct convert<std::vector<BulbState>> {
  static Node encode(const std::vector<BulbState>& rhs) {
    Node node;
    const auto mapper = drake::maliput::api::rules::BulbStateMapper();
    for (const auto& state : rhs) {
      node.push_back(mapper.at(state));
    }
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, std::vector<BulbState>& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    const auto mapper = drake::maliput::api::rules::BulbStateMapper();
    for (const YAML::Node& state_node : node) {
      const std::string state = state_node.as<std::string>();
      bool result = false;
      for (const auto& it : mapper) {
        if (it.second == state) {
          rhs.push_back(it.first);
          result = true;
        }
      }
      if (!result) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace YAML

namespace drake {
namespace maliput {
namespace {

Bulb BuildBulb(const YAML::Node& bulb_node) {
  DRAKE_THROW_UNLESS(bulb_node.IsDefined());
  DRAKE_THROW_UNLESS(bulb_node.IsMap());
  DRAKE_THROW_UNLESS(bulb_node["ID"].IsDefined());
  const Bulb::Id id(bulb_node["ID"].as<std::string>());
  const YAML::Node& pose_node = bulb_node["Pose"];
  DRAKE_THROW_UNLESS(pose_node.IsDefined());
  DRAKE_THROW_UNLESS(pose_node.IsMap());
  DRAKE_THROW_UNLESS(pose_node["position_bulb_group"].IsDefined());
  DRAKE_THROW_UNLESS(pose_node["orientation_bulb_group"].IsDefined());
  const GeoPosition position_bulb_group =
      pose_node["position_bulb_group"].as<GeoPosition>();
  const Rotation orientation_bulb_group =
      pose_node["orientation_bulb_group"].as<Rotation>();

  const YAML::Node& bounding_box_node = bulb_node["BoundingBox"];
  Bulb::BoundingBox bounding_box;
  if (bounding_box_node.IsDefined()) {
    DRAKE_THROW_UNLESS(bounding_box_node.IsMap());
    bounding_box = bounding_box_node.as<Bulb::BoundingBox>();
  }

  const YAML::Node& color_node = bulb_node["Color"];
  DRAKE_THROW_UNLESS(color_node.IsDefined());
  const BulbColor color = color_node.as<BulbColor>();

  const YAML::Node& type_node = bulb_node["Type"];
  DRAKE_THROW_UNLESS(type_node.IsDefined());
  const BulbType type = type_node.as<BulbType>();

  optional<double> arrow_orientation_rad = nullopt;
  if (type == BulbType::kArrow) {
    const YAML::Node& arrow_orientation_node = bulb_node["ArrowOrientation"];
    DRAKE_THROW_UNLESS(arrow_orientation_node.IsDefined());
    arrow_orientation_rad = arrow_orientation_node.as<double>();
  }

  const YAML::Node& states_node = bulb_node["States"];
  std::vector<BulbState> bulb_states({BulbState::kOn, BulbState::kOff});
  if (states_node.IsDefined()) {
    bulb_states = states_node.as<std::vector<BulbState>>();
  }

  return Bulb(id, position_bulb_group, orientation_bulb_group, color, type,
              arrow_orientation_rad, bulb_states, bounding_box);
}

BulbGroup BuildBulbGroup(const YAML::Node& bulb_group_node) {
  DRAKE_THROW_UNLESS(bulb_group_node.IsDefined());
  DRAKE_THROW_UNLESS(bulb_group_node.IsMap());
  DRAKE_THROW_UNLESS(bulb_group_node["ID"].IsDefined());
  const BulbGroup::Id id(bulb_group_node["ID"].as<std::string>());
  const YAML::Node& pose_node = bulb_group_node["Pose"];
  DRAKE_THROW_UNLESS(pose_node.IsDefined());
  DRAKE_THROW_UNLESS(pose_node.IsMap());
  DRAKE_THROW_UNLESS(pose_node["position_traffic_light"].IsDefined());
  DRAKE_THROW_UNLESS(pose_node["orientation_traffic_light"].IsDefined());
  const GeoPosition position_traffic_light =
      pose_node["position_traffic_light"].as<GeoPosition>();
  const Rotation orientation_traffic_light =
      pose_node["orientation_traffic_light"].as<Rotation>();
  const YAML::Node& bulbs_node = bulb_group_node["Bulbs"];
  DRAKE_THROW_UNLESS(bulbs_node.IsDefined());
  DRAKE_THROW_UNLESS(bulbs_node.IsSequence());
  std::vector<Bulb> bulbs;
  for (const YAML::Node& bulb_node : bulbs_node) {
    bulbs.push_back(BuildBulb(bulb_node));
  }
  return BulbGroup(id, position_traffic_light, orientation_traffic_light,
                   bulbs);
}

TrafficLight BuildTrafficLight(const YAML::Node& traffic_light_node) {
  DRAKE_THROW_UNLESS(traffic_light_node.IsMap());
  const TrafficLight::Id id(traffic_light_node["ID"].as<std::string>());
  const YAML::Node& pose_node = traffic_light_node["Pose"];
  DRAKE_THROW_UNLESS(pose_node.IsDefined());
  DRAKE_THROW_UNLESS(pose_node.IsMap());
  DRAKE_THROW_UNLESS(pose_node["position_road_network"].IsDefined());
  DRAKE_THROW_UNLESS(pose_node["orientation_road_network"].IsDefined());
  const GeoPosition position_road_network =
      pose_node["position_road_network"].as<GeoPosition>();
  const Rotation orientation_road_network =
      pose_node["orientation_road_network"].as<Rotation>();

  const YAML::Node& bulb_groups_node = traffic_light_node["BulbGroups"];
  DRAKE_THROW_UNLESS(bulb_groups_node.IsDefined());
  DRAKE_THROW_UNLESS(bulb_groups_node.IsSequence());
  std::vector<BulbGroup> bulb_groups;
  for (const YAML::Node& bulb_group_node : bulb_groups_node) {
    bulb_groups.push_back(BuildBulbGroup(bulb_group_node));
  }
  return TrafficLight(id, position_road_network, orientation_road_network,
                      bulb_groups);
}

std::unique_ptr<api::rules::TrafficLightBook> BuildFrom(
    const YAML::Node& root_node) {
  DRAKE_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& traffic_lights_node = root_node["TrafficLights"];
  DRAKE_THROW_UNLESS(traffic_lights_node.IsDefined());
  DRAKE_THROW_UNLESS(traffic_lights_node.IsSequence());
  auto result = std::make_unique<TrafficLightBook>();
  for (const YAML::Node& traffic_light_node : traffic_lights_node) {
    result->AddTrafficLight(BuildTrafficLight(traffic_light_node));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::rules::TrafficLightBook> LoadTrafficLightBook(
    const std::string& input) {
  return BuildFrom(YAML::Load(input));
}

std::unique_ptr<api::rules::TrafficLightBook> LoadTrafficLightBookFromFile(
    const std::string& filename) {
  return BuildFrom(YAML::LoadFile(filename));
}

}  // namespace maliput
}  // namespace drake
