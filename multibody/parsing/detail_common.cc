#include "drake/multibody/parsing/detail_common.h"

#include <filesystem>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;

bool EndsWithCaseInsensitive(std::string_view str, std::string_view ext) {
  if (ext.size() > str.size()) { return false; }
  return std::equal(str.end() - ext.size(), str.end(), ext.begin(),
                    [](char a, char b) { return tolower(a) == tolower(b); });
}

DataSource::DataSource(DataSourceType type, const std::string* data)
    : type_(type), data_(data) {
  DRAKE_DEMAND(IsFilename() != IsContents());
  DRAKE_DEMAND(data != nullptr);
}

const std::string& DataSource::filename() const {
  DRAKE_DEMAND(IsFilename());
  return *data_;
}

const std::string& DataSource::contents() const {
  DRAKE_DEMAND(IsContents());
  return *data_;
}

std::string DataSource::GetAbsolutePath() const {
  if (IsFilename()) {
    return std::filesystem::absolute(*data_).native();
  }
  return "";
}

std::string DataSource::GetRootDir() const {
  if (IsFilename()) {
    return std::filesystem::absolute(*data_).parent_path().native();
  }
  return "";
}

std::string DataSource::GetStem() const {
  if (IsFilename()) {
    std::filesystem::path p{*data_};
    return p.stem();
  }
  return kContentsPseudoStem;
}

geometry::ProximityProperties ParseProximityProperties(
    const DiagnosticPolicy& diagnostic,
    const std::function<std::optional<double>(const char*)>& read_double,
    bool is_rigid, bool is_compliant) {
  using HT = geometry::internal::HydroelasticType;
  using geometry::internal::kComplianceType;
  using geometry::internal::kElastic;
  using geometry::internal::kHydroGroup;
  using geometry::internal::kRezHint;

  // Both being true is disallowed -- so assert is_rigid NAND is_compliant.
  DRAKE_DEMAND(!(is_rigid && is_compliant));
  geometry::ProximityProperties properties;

  if (is_rigid) {
    properties.AddProperty(kHydroGroup, kComplianceType, HT::kRigid);
  } else if (is_compliant) {
    properties.AddProperty(kHydroGroup, kComplianceType, HT::kSoft);
  }

  std::optional<double> rez_hint = read_double("drake:mesh_resolution_hint");
  if (rez_hint) {
    properties.AddProperty(kHydroGroup, kRezHint, *rez_hint);
  }

  std::optional<double> hydroelastic_modulus =
      read_double("drake:hydroelastic_modulus");
  if (hydroelastic_modulus) {
    if (is_rigid) {
      diagnostic.Warning(fmt::format(
          "Rigid geometries defined with the tag drake:rigid_hydroelastic"
          " should not contain the tag drake:hydroelastic_modulus. The"
          " specified value ({}) will be ignored.",
          *hydroelastic_modulus));
    } else {
      properties.AddProperty(kHydroGroup, kElastic, *hydroelastic_modulus);
    }
  }

  std::optional<double> dissipation =
      read_double("drake:hunt_crossley_dissipation");

  std::optional<double> relaxation_time =
      read_double("drake:relaxation_time");

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

  geometry::AddContactMaterial(dissipation, stiffness, friction, &properties);

  if (relaxation_time.has_value()) {
    if (*relaxation_time < 0) {
      throw std::logic_error(
          fmt::format("The dissipation time scale can't be negative; given {}",
                      *dissipation));
    }
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kRelaxationTime,
                           *relaxation_time);
  }

  return properties;
}

const LinearBushingRollPitchYaw<double>* ParseLinearBushingRollPitchYaw(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const Frame<double>*(const char*)>& read_frame,
    MultibodyPlant<double>* plant) {
  const Frame<double>* frame_A = read_frame("drake:bushing_frameA");
  if (!frame_A) { return {}; }
  const Frame<double>* frame_C = read_frame("drake:bushing_frameC");
  if (!frame_C) { return {}; }

  const Eigen::Vector3d bushing_torque_stiffness =
      read_vector("drake:bushing_torque_stiffness");
  const Eigen::Vector3d bushing_torque_damping =
      read_vector("drake:bushing_torque_damping");
  const Eigen::Vector3d bushing_force_stiffness =
      read_vector("drake:bushing_force_stiffness");
  const Eigen::Vector3d bushing_force_damping =
      read_vector("drake:bushing_force_damping");

  return &plant->AddForceElement<LinearBushingRollPitchYaw>(
      *frame_A, *frame_C, bushing_torque_stiffness, bushing_torque_damping,
      bushing_force_stiffness, bushing_force_damping);
}

// See ParseCollisionFilterGroupCommon at header for documentation
void CollectCollisionFilterGroup(
    const DiagnosticPolicy& diagnostic,
    ModelInstanceIndex model_instance, const MultibodyPlant<double>& plant,
    const ElementNode& group_node,
    CollisionFilterGroupResolver* resolver,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_child_element,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_sibling_element,
    const std::function<bool(const ElementNode&, const char*)>& has_attribute,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_string_attribute,
    const std::function<bool(const ElementNode&, const char*)>&
        read_bool_attribute,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_tag_string) {
  DRAKE_DEMAND(plant.geometry_source_is_registered());
  if (has_attribute(group_node, "ignore")) {
    if (read_bool_attribute(group_node, "ignore")) {
      return;
    }
  }
  const std::string group_name = read_string_attribute(group_node, "name");
  if (group_name.empty()) { return; }

  std::set<std::string> bodies;
  for (auto member_node = next_child_element(group_node, "drake:member");
       std::holds_alternative<sdf::ElementPtr>(member_node)
           ? std::get<sdf::ElementPtr>(member_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(member_node) != nullptr;
       member_node = next_sibling_element(member_node, "drake:member")) {
    const std::string body_name = read_tag_string(member_node, "link");
    if (body_name.empty()) { continue; }

    bodies.insert(body_name);
  }
  resolver->AddGroup(diagnostic, group_name, bodies, model_instance);

  for (auto ignore_node = next_child_element(
           group_node, "drake:ignored_collision_filter_group");
       std::holds_alternative<sdf::ElementPtr>(ignore_node)
           ? std::get<sdf::ElementPtr>(ignore_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(ignore_node) != nullptr;
       ignore_node = next_sibling_element(
           ignore_node, "drake:ignored_collision_filter_group")) {
    const std::string target_name = read_tag_string(ignore_node, "name");
    if (target_name.empty()) { continue; }

    // These two group names are allowed to be identical, which means the
    // bodies inside this collision filter group should be collision excluded
    // among each other.
    resolver->AddPair(diagnostic, group_name, target_name, model_instance);
  }
}

void ParseCollisionFilterGroupCommon(
    const DiagnosticPolicy& diagnostic,
    ModelInstanceIndex model_instance,
    const ElementNode& model_node,
    MultibodyPlant<double>* plant,
    CollisionFilterGroupResolver* resolver,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_child_element,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_sibling_element,
    const std::function<bool(const ElementNode&, const char*)>& has_attribute,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_string_attribute,
    const std::function<bool(const ElementNode&, const char*)>&
        read_bool_attribute,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_tag_string) {
  DRAKE_DEMAND(plant->geometry_source_is_registered());

  for (auto group_node =
           next_child_element(model_node, "drake:collision_filter_group");
       std::holds_alternative<sdf::ElementPtr>(group_node)
           ? std::get<sdf::ElementPtr>(group_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(group_node) != nullptr;
       group_node =
           next_sibling_element(group_node, "drake:collision_filter_group")) {
    CollectCollisionFilterGroup(
        diagnostic,
        model_instance, *plant, group_node, resolver, next_child_element,
        next_sibling_element, has_attribute, read_string_attribute,
        read_bool_attribute, read_tag_string);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
