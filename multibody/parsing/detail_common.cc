#include "drake/multibody/parsing/detail_common.h"

#include <filesystem>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;
using Eigen::Vector3d;

bool EndsWithCaseInsensitive(std::string_view str, std::string_view ext) {
  if (ext.size() > str.size()) {
    return false;
  }
  return std::equal(str.end() - ext.size(), str.end(), ext.begin(),
                    [](char a, char b) {
                      return tolower(a) == tolower(b);
                    });
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
  using geometry::internal::kMargin;
  using geometry::internal::kMaterialGroup;
  using geometry::internal::kRelaxationTime;
  using geometry::internal::kRezHint;

  // Both being true is disallowed -- so assert is_rigid NAND is_compliant.
  DRAKE_DEMAND(!(is_rigid && is_compliant));
  geometry::ProximityProperties properties;

  if (is_rigid) {
    properties.AddProperty(kHydroGroup, kComplianceType, HT::kRigid);
  } else if (is_compliant) {
    properties.AddProperty(kHydroGroup, kComplianceType, HT::kSoft);
  }

  {
    std::optional<double> rez_hint = read_double("drake:mesh_resolution_hint");
    if (rez_hint) {
      properties.AddProperty(kHydroGroup, kRezHint, *rez_hint);
    }
  }

  {
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
  }

  {
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

    geometry::AddContactMaterial(dissipation, stiffness, friction, &properties);
  }

  {
    std::optional<double> relaxation_time =
        read_double("drake:relaxation_time");
    if (relaxation_time.has_value()) {
      if (*relaxation_time < 0) {
        diagnostic.Error(
            fmt::format("The relaxation time can't be negative; given {}",
                        *relaxation_time));
      } else {
        properties.AddProperty(kMaterialGroup, kRelaxationTime,
                               *relaxation_time);
      }
    }
  }

  {
    std::optional<double> margin = read_double("drake:hydroelastic_margin");
    if (margin.has_value()) {
      if (*margin < 0) {
        diagnostic.Error(fmt::format(
            "The hydroelastic margin can't be negative; given {}", *margin));
      } else {
        properties.AddProperty(kHydroGroup, kMargin, *margin);
      }
    }
  }

  return properties;
}

const LinearBushingRollPitchYaw<double>* ParseLinearBushingRollPitchYaw(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const Frame<double>*(const char*)>& read_frame,
    MultibodyPlant<double>* plant) {
  const Frame<double>* frame_A = read_frame("drake:bushing_frameA");
  if (!frame_A) {
    return {};
  }
  const Frame<double>* frame_C = read_frame("drake:bushing_frameC");
  if (!frame_C) {
    return {};
  }

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

std::optional<MultibodyConstraintId> ParseBallConstraint(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const RigidBody<double>*(const char*)>& read_body,
    MultibodyPlant<double>* plant) {
  const RigidBody<double>* body_A = read_body("drake:ball_constraint_body_A");
  if (!body_A) {
    return {};
  }
  const RigidBody<double>* body_B = read_body("drake:ball_constraint_body_B");
  if (!body_B) {
    return {};
  }

  const Eigen::Vector3d p_AP = read_vector("drake:ball_constraint_p_AP");
  const Eigen::Vector3d p_BQ = read_vector("drake:ball_constraint_p_BQ");

  return plant->AddBallConstraint(*body_A, p_AP, *body_B, p_BQ);
}

const LinearSpringDamper<double>* ParseLinearSpringDamper(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const RigidBody<double>*(const char*)>& read_body,
    const std::function<std::optional<double>(const char*)>& read_double,
    MultibodyPlant<double>* plant) {
  const RigidBody<double>* body_A =
      read_body("drake:linear_spring_damper_body_A");
  if (!body_A) {
    return {};
  }
  const RigidBody<double>* body_B =
      read_body("drake:linear_spring_damper_body_B");
  if (!body_B) {
    return {};
  }

  const Eigen::Vector3d p_AP = read_vector("drake:linear_spring_damper_p_AP");
  const Eigen::Vector3d p_BQ = read_vector("drake:linear_spring_damper_p_BQ");

  const std::optional<double> free_length =
      read_double("drake:linear_spring_damper_free_length");
  if (!free_length) {
    return {};
  }
  const std::optional<double> stiffness =
      read_double("drake:linear_spring_damper_stiffness");
  if (!stiffness) {
    return {};
  }
  const std::optional<double> damping =
      read_double("drake:linear_spring_damper_damping");
  if (!damping) {
    return {};
  }

  return &plant->AddForceElement<LinearSpringDamper>(
      *body_A, p_AP, *body_B, p_BQ, *free_length, *stiffness, *damping);
}

std::optional<MultibodyConstraintId> ParseTendonConstraint(
    const DiagnosticPolicy& diagnostic, ModelInstanceIndex model_instance,
    const ElementNode& constraint_node,
    const std::function<std::optional<double>(const char*)>& read_double,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_child_element,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_sibling_element,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_string_attribute,
    const std::function<double(const ElementNode&, const char*)>&
        read_double_attribute,
    MultibodyPlant<double>* plant) {
  std::vector<JointIndex> joints = {};
  std::vector<double> a = {};
  for (auto joint_node =
           next_child_element(constraint_node, "drake:tendon_constraint_joint");
       std::holds_alternative<sdf::ElementPtr>(joint_node)
           ? std::get<sdf::ElementPtr>(joint_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(joint_node) != nullptr;
       joint_node =
           next_sibling_element(joint_node, "drake:tendon_constraint_joint")) {
    const std::string joint_name = read_string_attribute(joint_node, "name");
    if (!plant->HasJointNamed(joint_name, model_instance)) {
      diagnostic.Error(fmt::format(
          "<drake:tendon_constraint>: Joint '{}' specified for "
          "<drake:tendon_constraint_joint> does not exist in the model.",
          joint_name));
      return {};
    }

    const JointIndex joint_index =
        plant->GetJointByName(joint_name, model_instance).index();
    joints.push_back(joint_index);

    const double joint_a = read_double_attribute(joint_node, "a");
    a.push_back(joint_a);
  }

  const std::optional<double> offset =
      read_double("drake:tendon_constraint_offset");
  const std::optional<double> lower_limit =
      read_double("drake:tendon_constraint_lower_limit");
  const std::optional<double> upper_limit =
      read_double("drake:tendon_constraint_upper_limit");
  const std::optional<double> stiffness =
      read_double("drake:tendon_constraint_stiffness");
  const std::optional<double> damping =
      read_double("drake:tendon_constraint_damping");

  return plant->AddTendonConstraint(joints, a, offset, lower_limit, upper_limit,
                                    stiffness, damping);
}

namespace {
// See ParseCollisionFilterGroupCommon at header for documentation
void CollectCollisionFilterGroup(
    const DiagnosticPolicy& diagnostic, ModelInstanceIndex model_instance,
    const MultibodyPlant<double>& plant, const ElementNode& group_node,
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
  if (group_name.empty()) {
    return;
  }

  std::set<std::string> bodies;
  for (auto member_node = next_child_element(group_node, "drake:member");
       std::holds_alternative<sdf::ElementPtr>(member_node)
           ? std::get<sdf::ElementPtr>(member_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(member_node) != nullptr;
       member_node = next_sibling_element(member_node, "drake:member")) {
    const std::string body_name = read_tag_string(member_node, "link");
    if (body_name.empty()) {
      continue;
    }

    bodies.insert(body_name);
  }
  std::set<std::string> member_groups;
  for (auto member_node = next_child_element(group_node, "drake:member_group");
       std::holds_alternative<sdf::ElementPtr>(member_node)
           ? std::get<sdf::ElementPtr>(member_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(member_node) != nullptr;
       member_node = next_sibling_element(member_node, "drake:member_group")) {
    const std::string member_group_name = read_tag_string(member_node, "name");
    if (member_group_name.empty()) {
      continue;
    }

    member_groups.insert(member_group_name);
  }
  resolver->AddGroup(diagnostic, group_name, bodies, member_groups,
                     model_instance);

  for (auto ignore_node = next_child_element(
           group_node, "drake:ignored_collision_filter_group");
       std::holds_alternative<sdf::ElementPtr>(ignore_node)
           ? std::get<sdf::ElementPtr>(ignore_node) != nullptr
           : std::get<tinyxml2::XMLElement*>(ignore_node) != nullptr;
       ignore_node = next_sibling_element(
           ignore_node, "drake:ignored_collision_filter_group")) {
    const std::string target_name = read_tag_string(ignore_node, "name");
    if (target_name.empty()) {
      continue;
    }

    // These two group names are allowed to be identical, which means the
    // bodies inside this collision filter group should be collision excluded
    // among each other.
    resolver->AddPair(diagnostic, group_name, target_name, model_instance);
  }
}
}  // namespace

void ParseCollisionFilterGroupCommon(
    const DiagnosticPolicy& diagnostic, ModelInstanceIndex model_instance,
    const ElementNode& model_node, MultibodyPlant<double>* plant,
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
        diagnostic, model_instance, *plant, group_node, resolver,
        next_child_element, next_sibling_element, has_attribute,
        read_string_attribute, read_bool_attribute, read_tag_string);
  }
}

SpatialInertia<double> ParseSpatialInertia(const DiagnosticPolicy& diagnostic,
                                           const math::RigidTransformd& X_BBi,
                                           double mass,
                                           const InertiaInputs& inertia_Bi_Bi) {
  // If physical validity checks fail below, return a prepared plausible
  // inertia instead. Use a plausible guess for the mass, letting actual mass
  // validity checking happen later.
  const double plausible_dummy_mass =
      (std::isfinite(mass) && mass > 0.0) ? mass : 1.0;
  // Construct a dummy inertia for a solid sphere, with the density of water,
  // and radius deduced from the plausible mass.
  static constexpr double kPlausibleDensity{1000};  // Water is 1000 kg/m³.
  const double plausible_volume = plausible_dummy_mass / kPlausibleDensity;
  // Volume = (4/3)π(radius)³, so radius = ³√(3/(4π))(volume).
  const double plausible_radius =
      std::cbrt((3.0 / (4.0 * M_PI)) * plausible_volume);
  // Create Mdum_BBo_B, a plausible spatial inertia for B about Bo (B's origin).
  // To do this, create Mdum_BBcm (a plausible spatial inertia for B about
  // Bcm) as a solid sphere and then shift that spatial inertia from Bcm to Bo.
  // Note: It is unwise to directly create a solid sphere about Bo as this does
  // not guarantee that the spatial inertia about Bcm is valid. Bcm (B's center
  // of mass) is the ground-truth point for validity tests.
  const SpatialInertia<double> Mdum_BBcm =
      SpatialInertia<double>::SolidSphereWithDensity(kPlausibleDensity,
                                                     plausible_radius);
  const Vector3d& p_BoBcm_B = X_BBi.translation();
  const SpatialInertia<double> Mdum_BBo_B = Mdum_BBcm.Shift(-p_BoBcm_B);

  // Yes, catching exceptions violates the coding standard. It is done here to
  // capture math-aware exceptions into parse-time warnings, since non-physical
  // inertias are all too common, and the thrown messages are actually pretty
  // useful.
  //
  // The safety and correctness of the catch blocks here relies on the promises
  // of the *Inertia classes to satisfy the "basic exception guarantee".
  // See https://en.cppreference.com/w/cpp/language/exceptions
  RotationalInertia<double> I_BBcm_Bi;
  try {
    // Use the factory method here; it doesn't change its diagnostic behavior
    // between release and debug builds.
    I_BBcm_Bi = RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
        inertia_Bi_Bi.ixx, inertia_Bi_Bi.iyy, inertia_Bi_Bi.izz,
        inertia_Bi_Bi.ixy, inertia_Bi_Bi.ixz, inertia_Bi_Bi.iyz);
  } catch (const std::exception& e) {
    diagnostic.Warning(
        fmt::format("While parsing inertia matrix: {}", e.what()));
    return Mdum_BBo_B;
  }

  // If this is a massless body, return a zero SpatialInertia.
  if (mass == 0.0 && I_BBcm_Bi.get_moments().isZero() &&
      I_BBcm_Bi.get_products().isZero()) {
    return SpatialInertia<double>(0.0, Vector3d::Zero(),
                                  UnitInertia<double>{0.0, 0.0, 0.0});
  }
  // B and Bi are not necessarily aligned.
  const math::RotationMatrix<double>& R_BBi = X_BBi.rotation();

  // Re-express in frame B as needed.
  const RotationalInertia<double> I_BBcm_B = I_BBcm_Bi.ReExpress(R_BBi);

  try {
    return SpatialInertia<double>::MakeFromCentralInertia(mass, p_BoBcm_B,
                                                          I_BBcm_B);
  } catch (const std::exception& e) {
    diagnostic.Warning(
        fmt::format("While re-expressing as central inertia: {}", e.what()));
    return Mdum_BBo_B;
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
