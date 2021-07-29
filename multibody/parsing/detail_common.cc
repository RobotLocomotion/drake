#include "drake/multibody/parsing/detail_common.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace internal {

void DataSource::DemandExactlyOne() const {
  DRAKE_DEMAND((file_name != nullptr) ^ (file_contents != nullptr));
}

geometry::ProximityProperties ParseProximityProperties(
    const std::function<std::optional<double>(const char*)>& read_double,
    bool is_rigid, bool is_soft) {
  // Both being true is disallowed -- so assert is_rigid NAND is_soft.
  DRAKE_DEMAND(!(is_rigid && is_soft));
  geometry::ProximityProperties properties;
  std::optional<double> rez_hint = read_double("drake:mesh_resolution_hint");
  if (rez_hint) {
    if (is_rigid) {
      geometry::AddRigidHydroelasticProperties(*rez_hint, &properties);
    } else if (is_soft) {
      geometry::AddSoftHydroelasticProperties(*rez_hint, &properties);
    } else {
      properties.AddProperty(geometry::internal::kHydroGroup,
                             geometry::internal::kRezHint, *rez_hint);
    }
  } else {
    if (is_rigid) {
      geometry::AddRigidHydroelasticProperties(&properties);
    } else if (is_soft) {
      geometry::AddSoftHydroelasticProperties(&properties);
    }
  }

  std::optional<double> elastic_modulus = read_double("drake:elastic_modulus");

  std::optional<double> dissipation =
      read_double("drake:hunt_crossley_dissipation");

  std::optional<double> stiffness =
      read_double("drake:point_contact_stiffness");

  std::optional<double> mu_dynamic = read_double("drake:mu_dynamic");
  std::optional<double> mu_static = read_double("drake:mu_static");
  std::optional<CoulombFriction<double>> friction;
  // Note: we rely on the constructor of CoulombFriction to detect negative
  // values and bad relationship between static and dynamic coefficients.
  if (mu_dynamic && mu_static) {
    friction = CoulombFriction<double>(*mu_static, *mu_dynamic);
  } else if (mu_dynamic) {
    friction = CoulombFriction<double>(*mu_dynamic, *mu_dynamic);
  } else if (mu_static) {
    friction = CoulombFriction<double>(*mu_static, *mu_static);
  }

  geometry::AddContactMaterial(elastic_modulus, dissipation, stiffness,
                               friction, &properties);

  return properties;
}

const LinearBushingRollPitchYaw<double>& ParseLinearBushingRollPitchYaw(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const Frame<double>&(const char*)>& read_frame,
    MultibodyPlant<double>* plant) {
  const Frame<double>& frame_A = read_frame("drake:bushing_frameA");
  const Frame<double>& frame_C = read_frame("drake:bushing_frameC");

  const Eigen::Vector3d bushing_torque_stiffness =
      read_vector("drake:bushing_torque_stiffness");
  const Eigen::Vector3d bushing_torque_damping =
      read_vector("drake:bushing_torque_damping");
  const Eigen::Vector3d bushing_force_stiffness =
      read_vector("drake:bushing_force_stiffness");
  const Eigen::Vector3d bushing_force_damping =
      read_vector("drake:bushing_force_damping");

  return plant->AddForceElement<LinearBushingRollPitchYaw>(
      frame_A, frame_C, bushing_torque_stiffness, bushing_torque_damping,
      bushing_force_stiffness, bushing_force_damping);
}

void RegisterCollisionFilterGroup(
    ModelInstanceIndex model_instance,
    const MultibodyPlant<double>& plant,
    std::variant<sdf::ElementPtr, tinyxml2::XMLElement*> group_node,
    std::map<std::string, geometry::GeometrySet>* collision_filter_groups,
    std::set<SortedPair<std::string>>* collision_filter_pairs,
    const std::function< std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement*>(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement*>, const char*)> &next_child_element,
    const std::function< std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement* > (std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement* >, const char*)> &next_sibbling_element,
    const std::function<bool(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement* >, const char*)> &has_attribute,
    const std::function<std::string(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement* >, const char*)> &read_string_attribute,
    const std::function<bool(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement* >, const char*)> &read_bool_attribute) {
  DRAKE_DEMAND(plant.geometry_source_is_registered());
  if (has_attribute(group_node, "ignore")) {
    if (read_bool_attribute(group_node, "ignore")) {
      std::cout << "IGNORED!!!" << std::endl;
      return;
      }
    }

  if (!has_attribute(group_node, "name")) {
    throw std::runtime_error("ERROR: group tag is missing name attribute.");
  }
  const std::string group_name = read_string_attribute(group_node, "name");

  geometry::GeometrySet collision_filter_geometry_set;
  for (std::variant<sdf::ElementPtr, tinyxml2::XMLElement*> member_node =
        next_child_element(group_node, "drake:member");
    std::holds_alternative<sdf::ElementPtr>(member_node) ?
        std::get<sdf::ElementPtr>(member_node) != nullptr :
        std::get<tinyxml2::XMLElement*>(member_node)  != nullptr;
    member_node = next_sibbling_element(member_node, "drake:member")) {
    if (!has_attribute(member_node, "link")) {
      throw std::runtime_error(
          fmt::format("'{}':'{}': Collision filter group '{}' provides a "
                      "member tag without specifying the \"link\" attribute.",
                      __FILE__, __func__, group_name.c_str()));
    }
    const std::string body_name = read_string_attribute(member_node, "link");

    const auto& body = plant.GetBodyByName(body_name.c_str(), model_instance);
    collision_filter_geometry_set.Add(
        plant.GetBodyFrameIdOrThrow(body.index()));
  }
  collision_filter_groups->insert({group_name, collision_filter_geometry_set});

  for (std::variant<sdf::ElementPtr, tinyxml2::XMLElement*> ignore_node =
        next_child_element(group_node, "drake:ignored_collision_filter_group");
    std::holds_alternative<sdf::ElementPtr>(ignore_node) ?
        std::get<sdf::ElementPtr>(ignore_node) != nullptr :
        std::get<tinyxml2::XMLElement*>(ignore_node)  != nullptr;
    ignore_node = next_sibbling_element(
        ignore_node, "drake:collision_filter_group")) {
    if (!has_attribute(ignore_node, "name")) {
      throw std::runtime_error(fmt::format(
          "'{}':'{}': Collision filter group provides a tag specifying a "
          "group to ignore without specifying the \"name\" attribute.",
          __FILE__, __func__));
    }
    const std::string target_name = read_string_attribute(ignore_node, "name");

    // These two group names are allowed to be identical, which means the bodies
    // inside this collision filter group should be collision excluded among
    // each other.
    collision_filter_pairs->insert({group_name.c_str(), target_name.c_str()});
  }
}

void ParseCollisionFilterGroupCommon(
    ModelInstanceIndex model_instance,
    std::variant<sdf::ElementPtr, tinyxml2::XMLElement*> model_node,
    MultibodyPlant<double>* plant,
    const std::function< std::variant<sdf::ElementPtr, tinyxml2::XMLElement*>
        (std::variant<sdf::ElementPtr, tinyxml2::XMLElement*>, const char*)>
        &next_child_element,
    const std::function< std::variant<sdf::ElementPtr, tinyxml2::XMLElement*>
        (std::variant<sdf::ElementPtr, tinyxml2::XMLElement*>, const char*)>
        &next_sibbling_element,
    const std::function<bool(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement* >, const char*)> &has_attribute,
    const std::function<std::string(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement*>, const char*)> &read_string_attribute,
    const std::function<bool(std::variant<sdf::ElementPtr,
        tinyxml2::XMLElement*>, const char*)> &read_bool_attribute) {
  DRAKE_DEMAND(plant->geometry_source_is_registered());
  std::map<std::string, geometry::GeometrySet> collision_filter_groups;
  std::set<SortedPair<std::string>> collision_filter_pairs;

  for (std::variant<sdf::ElementPtr, tinyxml2::XMLElement*> group_node =
        next_child_element(model_node, "drake:collision_filter_group");
    std::holds_alternative<sdf::ElementPtr>(group_node) ?
        std::get<sdf::ElementPtr>(group_node) != nullptr :
        std::get<tinyxml2::XMLElement*>(group_node)  != nullptr;
    group_node = next_sibbling_element(
        group_node, "drake:collision_filter_group")) {
      RegisterCollisionFilterGroup(model_instance, *plant, group_node,
          &collision_filter_groups, &collision_filter_pairs, next_child_element,
          next_sibbling_element, has_attribute, read_string_attribute,
          read_bool_attribute);
  }

  for (const auto& collision_filter_pair : collision_filter_pairs) {
    const auto collision_filter_group_a =
        collision_filter_groups.find(collision_filter_pair.first());
    DRAKE_DEMAND(collision_filter_group_a != collision_filter_groups.end());
    const auto collision_filter_group_b =
        collision_filter_groups.find(collision_filter_pair.second());
    DRAKE_DEMAND(collision_filter_group_b != collision_filter_groups.end());

    plant->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        {collision_filter_group_a->first, collision_filter_group_a->second},
        {collision_filter_group_b->first, collision_filter_group_b->second});
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
