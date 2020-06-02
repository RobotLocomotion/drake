#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <fmt/format.h>
#include <tinyxml2.h>

#include "drake/common/sorted_pair.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/detail_urdf_geometry.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

const char* kWorldName = "world";

SpatialInertia<double> ExtractSpatialInertiaAboutBoExpressedInB(
    XMLElement* node) {

  RigidTransformd X_BBi;

  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    X_BBi = OriginAttributesToTransform(origin);
  }

  double body_mass = 0;
  XMLElement* mass = node->FirstChildElement("mass");
  if (mass) {
    ParseScalarAttribute(mass, "value", &body_mass);
  }

  double ixx = 0;
  double ixy = 0;
  double ixz = 0;
  double iyy = 0;
  double iyz = 0;
  double izz = 0;

  XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    ParseScalarAttribute(inertia, "ixx", &ixx);
    ParseScalarAttribute(inertia, "ixy", &ixy);
    ParseScalarAttribute(inertia, "ixz", &ixz);
    ParseScalarAttribute(inertia, "iyy", &iyy);
    ParseScalarAttribute(inertia, "iyz", &iyz);
    ParseScalarAttribute(inertia, "izz", &izz);
  }

  const RotationalInertia<double> I_BBcm_Bi(ixx, iyy, izz, ixy, ixz, iyz);

  // B and Bi are not necessarily aligned.
  const math::RotationMatrix<double> R_BBi(X_BBi.rotation());

  // Re-express in frame B as needed.
  const RotationalInertia<double> I_BBcm_B = I_BBcm_Bi.ReExpress(R_BBi);

  // Bi's origin is at the COM as documented in
  // http://wiki.ros.org/urdf/XML/link#Elements
  const Vector3d p_BoBcm_B = X_BBi.translation();

  return SpatialInertia<double>::MakeFromCentralInertia(
      body_mass, p_BoBcm_B, I_BBcm_B);
}

void ParseBody(const multibody::PackageMap& package_map,
               const std::string& root_dir,
               ModelInstanceIndex model_instance,
               XMLElement* node,
               MaterialMap* materials,
               MultibodyPlant<double>* plant) {
  std::string drake_ignore;
  if (ParseStringAttribute(node, "drake_ignore", &drake_ignore) &&
      drake_ignore == std::string("true")) {
    return;
  }

  std::string body_name;
  if (!ParseStringAttribute(node, "name", &body_name)) {
    throw std::runtime_error(
        "ERROR: link tag is missing name attribute.");
  }

  if (body_name == kWorldName) {
    return;
  }

  SpatialInertia<double> M_BBo_B;
  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (!inertial_node) {
    M_BBo_B = SpatialInertia<double>(
        0, Vector3d::Zero(), UnitInertia<double>(0, 0, 0));
  } else {
    M_BBo_B = ExtractSpatialInertiaAboutBoExpressedInB(inertial_node);
  }

  // Add a rigid body to model each link.
  const RigidBody<double>& body =
      plant->AddRigidBody(body_name, model_instance, M_BBo_B);

  if (plant->geometry_source_is_registered()) {
    for (XMLElement* visual_node = node->FirstChildElement("visual");
         visual_node;
         visual_node = visual_node->NextSiblingElement("visual")) {
      geometry::GeometryInstance geometry_instance =
          ParseVisual(body_name, package_map, root_dir, visual_node, materials);
      // The parsing should *always* produce an IllustrationProperties
      // instance, even if it is empty.
      DRAKE_DEMAND(geometry_instance.illustration_properties() != nullptr);
      plant->RegisterVisualGeometry(
          body, geometry_instance.pose(), geometry_instance.shape(),
          geometry_instance.name(),
          *geometry_instance.illustration_properties());
    }

    for (XMLElement* collision_node = node->FirstChildElement("collision");
         collision_node;
         collision_node = collision_node->NextSiblingElement("collision")) {
      geometry::GeometryInstance geometry_instance =
          ParseCollision(body_name, package_map, root_dir, collision_node);
      DRAKE_DEMAND(geometry_instance.proximity_properties());
      plant->RegisterCollisionGeometry(
          body, geometry_instance.pose(), geometry_instance.shape(),
          geometry_instance.name(),
          std::move(*geometry_instance.mutable_proximity_properties()));
    }
  }
}

// Parse the collision filter group tag information into the collision filter
// groups and a set of pairs between which the collisions will be excluded.
// @pre plant.geometry_source_is_registered() is `true`.
void RegisterCollisionFilterGroup(
    ModelInstanceIndex model_instance,
    const MultibodyPlant<double>& plant, XMLElement* node,
    std::map<std::string, geometry::GeometrySet>* collision_filter_groups,
    std::set<SortedPair<std::string>>* collision_filter_pairs) {
  DRAKE_DEMAND(plant.geometry_source_is_registered());
  std::string drake_ignore;
  if (ParseStringAttribute(node, "ignore", &drake_ignore) &&
      drake_ignore == std::string("true")) {
    return;
  }

  std::string group_name;
  if (!ParseStringAttribute(node, "name", &group_name)) {
    throw std::runtime_error("ERROR: group tag is missing name attribute.");
  }

  geometry::GeometrySet collision_filter_geometry_set;
  for (XMLElement* member_node = node->FirstChildElement("drake:member");
       member_node;
       member_node = member_node->NextSiblingElement("drake:member")) {
    const char* body_name = member_node->Attribute("link");
    if (!body_name) {
      throw std::runtime_error(
          fmt::format("'{}':'{}':'{}': Collision filter group '{}' provides a "
                      "member tag without specifying the \"link\" attribute.",
                      __FILE__, __func__, node->GetLineNum(), group_name));
    }
    const auto& body = plant.GetBodyByName(body_name, model_instance);
    collision_filter_geometry_set.Add(
        plant.GetBodyFrameIdOrThrow(body.index()));
  }
  collision_filter_groups->insert({group_name, collision_filter_geometry_set});

  for (XMLElement* ignore_node =
           node->FirstChildElement("drake:ignored_collision_filter_group");
       ignore_node; ignore_node = ignore_node->NextSiblingElement(
                        "drake:ignored_collision_filter_group")) {
    const char* target_name = ignore_node->Attribute("name");
    if (!target_name) {
      throw std::runtime_error(fmt::format(
          "'{}':'{}':'{}': Collision filter group provides a tag specifying a "
          "group to ignore without specifying the \"name\" attribute.",
          __FILE__, __func__, node->GetLineNum()));
    }
    // These two group names are allowed to be identical, which means the bodies
    // inside this collision filter group should be collision excluded among
    // each other.
    collision_filter_pairs->insert({group_name, target_name});
  }
}

// @pre plant->geometry_source_is_registered() is `true`.
void ParseCollisionFilterGroup(ModelInstanceIndex model_instance,
                               XMLElement* node,
                               MultibodyPlant<double>* plant) {
  DRAKE_DEMAND(plant->geometry_source_is_registered());
  std::map<std::string, geometry::GeometrySet> collision_filter_groups;
  std::set<SortedPair<std::string>> collision_filter_pairs;
  for (XMLElement* group_node =
           node->FirstChildElement("drake:collision_filter_group");
       group_node; group_node = group_node->NextSiblingElement(
                       "drake:collision_filter_group")) {
    RegisterCollisionFilterGroup(model_instance, *plant, group_node,
                                 &collision_filter_groups,
                                 &collision_filter_pairs);
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

// Parses a joint URDF specification to obtain the names of the joint, parent
// link, child link, and the joint type. An exception is thrown if any of these
// names cannot be determined.
//
// @param[in] node The XML node parsing the URDF joint description.
// @param[out] name A reference to a string where the name of the joint
// should be saved.
// @param[out] type A reference to a string where the joint type should be
// saved.
// @param[out] parent_link_name A reference to a string where the name of the
// parent link should be saved.
// @param[out] child_link_name A reference to a string where the name of the
// child link should be saved.
void ParseJointKeyParams(XMLElement* node,
                         std::string* name,
                         std::string* type,
                         std::string* parent_link_name,
                         std::string* child_link_name) {
  if (!ParseStringAttribute(node, "name", name)) {
    throw std::runtime_error(
        "ERROR: joint tag is missing name attribute");
  }

  if (!ParseStringAttribute(node, "type", type)) {
    throw std::runtime_error(
        "ERROR: joint " + *name + " is missing type attribute");
  }

  // Obtains the name of the joint's parent link.
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) {
    throw std::runtime_error(
        "ERROR: joint " + *name + " doesn't have a parent node!");
  }
  if (!ParseStringAttribute(parent_node, "link", parent_link_name)) {
    throw std::runtime_error(
        "ERROR: joint " + *name + "'s parent does not have a link attribute!");
  }

  // Obtains the name of the joint's child link.
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node) {
    throw std::runtime_error(
        "ERROR: joint " + *name + " doesn't have a child node");
  }
  if (!ParseStringAttribute(child_node, "link", child_link_name)) {
    throw std::runtime_error(
        "ERROR: joint " + *name + "'s child does not have a link attribute!");
  }
}

void ParseJointLimits(XMLElement* node, double* lower, double* upper,
                      double* velocity, double* effort) {
  *lower = -std::numeric_limits<double>::infinity();
  *upper = std::numeric_limits<double>::infinity();
  *velocity = std::numeric_limits<double>::infinity();
  *effort = std::numeric_limits<double>::infinity();

  XMLElement* limit_node = node->FirstChildElement("limit");
  if (limit_node) {
    ParseScalarAttribute(limit_node, "lower", lower);
    ParseScalarAttribute(limit_node, "upper", upper);
    ParseScalarAttribute(limit_node, "velocity", velocity);
    ParseScalarAttribute(limit_node, "effort", effort);
  }
}

void ParseJointDynamics(const std::string& joint_name,
                        XMLElement* node, double* damping) {
  *damping = 0.0;
  double coulomb_friction = 0.0;
  double coulomb_window = std::numeric_limits<double>::epsilon();

  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (dynamics_node) {
    ParseScalarAttribute(dynamics_node, "damping", damping);
    if (ParseScalarAttribute(dynamics_node, "friction", &coulomb_friction) &&
        coulomb_friction != 0.0) {
      drake::log()->warn("Joint {} specifies non-zero friction, which is "
                         "not supported by MultibodyPlant", joint_name);
    }
    if (ParseScalarAttribute(dynamics_node, "coulomb_window",
                             &coulomb_window) &&
        coulomb_window != std::numeric_limits<double>::epsilon()) {
      drake::log()->warn("Joint {} specifies non-zero coulomb_window, which is "
                         "not supported by MultibodyPlant", joint_name);
    }
  }
}

const Body<double>& GetBodyForElement(
    const std::string& element_name,
    const std::string& link_name,
    ModelInstanceIndex model_instance,
    MultibodyPlant<double>* plant) {
  if (link_name == kWorldName) {
    return plant->world_body();
  }

  if (!plant->HasBodyNamed(link_name, model_instance)) {
    throw std::runtime_error(
        "ERROR: Could not find link named\"" +
        link_name + "\" with model instance ID " +
        std::to_string(model_instance) + " for element " + element_name + ".");
  }
  return plant->GetBodyByName(link_name, model_instance);
}

void ParseJoint(ModelInstanceIndex model_instance,
                std::map<std::string, double>* joint_effort_limits,
                XMLElement* node,
                MultibodyPlant<double>* plant) {
  std::string drake_ignore;
  if (ParseStringAttribute(node, "drake_ignore", &drake_ignore) &&
      drake_ignore == std::string("true")) {
    return;
  }

  // Parses the parent and child link names.
  std::string name, type, parent_name, child_name;
  ParseJointKeyParams(node, &name, &type, &parent_name, &child_name);

  const Body<double>& parent_body = GetBodyForElement(
      name, parent_name, model_instance, plant);
  const Body<double>& child_body = GetBodyForElement(
      name, child_name, model_instance, plant);

  RigidTransformd X_PJ;
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    X_PJ = OriginAttributesToTransform(origin);
  }

  Vector3d axis(1, 0, 0);
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && type.compare("fixed") != 0 &&
      type.compare("floating") != 0 && type.compare("ball") != 0) {
    ParseVectorAttribute(axis_node, "xyz", &axis);
    if (axis.norm() < 1e-8) {
      throw std::runtime_error(
          "ERROR: Joint " + name + "axis is zero.  Don't do that.");
    }
    axis.normalize();
  }

  // Joint properties -- these are only used by some joint types.

  // Dynamic properties
  double damping = 0;

  // Limits
  double upper = std::numeric_limits<double>::infinity();
  double lower = -std::numeric_limits<double>::infinity();
  double velocity = std::numeric_limits<double>::infinity();

  // In MultibodyPlant, the effort limit is a property of the actuator, which
  // isn't created until the transmission element is parsed.  Stash a value
  // for all joints when parsing the joint element so that we can look it up
  // later if/when an actuator is created.
  double effort = std::numeric_limits<double>::infinity();


  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    ParseJointLimits(node, &lower, &upper, &velocity, &effort);
    ParseJointDynamics(name, node, &damping);
    const JointIndex index = plant->AddJoint<RevoluteJoint>(
        name, parent_body, X_PJ,
        child_body, std::nullopt, axis, lower, upper, damping).index();
    Joint<double>& joint = plant->get_mutable_joint(index);
    joint.set_velocity_limits(Vector1d(-velocity), Vector1d(velocity));
  } else if (type.compare("fixed") == 0) {
    plant->AddJoint<WeldJoint>(name, parent_body, X_PJ,
                               child_body, std::nullopt,
                               RigidTransformd::Identity());
  } else if (type.compare("prismatic") == 0) {
    ParseJointLimits(node, &lower, &upper, &velocity, &effort);
    ParseJointDynamics(name, node, &damping);
    const JointIndex index = plant->AddJoint<PrismaticJoint>(
        name, parent_body, X_PJ,
        child_body, std::nullopt, axis, lower, upper, damping).index();
    Joint<double>& joint = plant->get_mutable_joint(index);
    joint.set_velocity_limits(Vector1d(-velocity), Vector1d(velocity));
  } else if (type.compare("floating") == 0) {
    drake::log()->warn("Joint {} specified as type floating which is not "
                       "supported by MultibodyPlant.  Leaving {} as a "
                       "free body.", name, child_name);
  } else if (type.compare("ball") == 0) {
    drake::log()->warn(
        "Warning: ball joint is not an official part of the URDF standard.");
    throw std::runtime_error("Joint " + name + " specified as type ball which "
                             "is not supported by MultibodyPlant.");
  } else {
    throw std::runtime_error(
        "ERROR: Joint " + name + " has unrecognized type: " + type);
  }

  joint_effort_limits->emplace(name, effort);
}

void ParseTransmission(
    ModelInstanceIndex model_instance,
    const std::map<std::string, double>& joint_effort_limits,
    XMLElement* node,
    MultibodyPlant<double>* plant) {
  // Determines the transmission type.
  std::string type;
  XMLElement* type_node = node->FirstChildElement("type");
  if (type_node) {
    type = type_node->GetText();
  } else {
    // Old URDF format, kept for convenience
    if (!ParseStringAttribute(node, "type", &type)) {
      throw std::runtime_error(
          std::string(__FILE__) + ": " + __func__ +
          "ERROR: Transmission element is missing a type.");
    }
  }

  // Checks if the transmission type is not SimpleTransmission. If it is not,
  // print a warning and then abort this method call since only simple
  // transmissions are supported at this time.
  if (type.find("SimpleTransmission") == std::string::npos) {
    drake::log()->warn(
        "Only SimpleTransmissions are supported right now.  This element "
        "will be skipped.");
    return;
  }

  // Determines the actuator's name.
  XMLElement* actuator_node = node->FirstChildElement("actuator");
  if (!actuator_node) {
    throw std::runtime_error(
        "ERROR: Transmission is missing an actuator element.");
  }

  std::string actuator_name;
  if (!ParseStringAttribute(actuator_node, "name", &actuator_name)) {
    throw std::runtime_error(
        "ERROR: Transmission is missing an actuator name.");
  }

  // Determines the name of the joint to which the actuator is attached.
  XMLElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node) {
    throw std::runtime_error(
        "ERROR: Transmission is missing a joint element.");
  }

  std::string joint_name;
  if (!ParseStringAttribute(joint_node, "name", &joint_name)) {
    throw std::runtime_error(
        "ERROR: Transmission is missing a joint name.");
  }

  if (!plant->HasJointNamed(joint_name, model_instance)) {
    throw std::runtime_error(
        "ERROR: Transmission specifies joint " + joint_name +
        " which does not exist.");
  }
  const Joint<double>& joint = plant->GetJointByName(
      joint_name, model_instance);

  // Checks if the actuator is attached to a fixed joint. If so, abort this
  // method call.
  if (joint.num_positions() == 0) {
    drake::log()->warn(
        "WARNING: Skipping transmission since it's attached to "
        "a fixed joint \"" + joint_name + "\".");
    return;
  }

  const auto effort_iter = joint_effort_limits.find(joint_name);
  DRAKE_DEMAND(effort_iter != joint_effort_limits.end());
  if (effort_iter->second < 0) {
    throw std::runtime_error(
        "ERROR: Transmission specifies joint " + joint_name +
        " which has a negative effort limit.");
  }

  if (effort_iter->second <= 0) {
    drake::log()->warn(
        "WARNING: Skipping transmission since it's attached to "
        "joint \"" + joint_name + "\" which has a zero "
        "effort limit {}.", effort_iter->second);
    return;
  }

  plant->AddJointActuator(actuator_name, joint, effort_iter->second);
}

void ParseFrame(ModelInstanceIndex model_instance,
                XMLElement* node,
                MultibodyPlant<double>* plant) {
  std::string name;
  if (!ParseStringAttribute(node, "name", &name)) {
    throw std::runtime_error("ERROR parsing frame name.");
  }

  std::string body_name;
  if (!ParseStringAttribute(node, "link", &body_name)) {
    throw std::runtime_error(
        "ERROR: missing link name for frame " + name + ".");
  }

  const Body<double>& body =
      GetBodyForElement(name, body_name, model_instance, plant);

  RigidTransformd X_BF = OriginAttributesToTransform(node);
  plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      name, body.body_frame(), X_BF));
}

void ParseBushing(XMLElement* node, MultibodyPlant<double>* plant) {
  // Functor to read a child element with a vector valued `value` attribute
  // Throws an error if unable to find the tag or if the value attribute is
  // improperly formed.
  auto read_vector = [node](const char* element_name) -> Eigen::Vector3d {
    const XMLElement* value_node = node->FirstChildElement(element_name);
    if (value_node != nullptr) {
      Eigen::Vector3d value;
      if (ParseVectorAttribute(value_node, "value", &value)) {
        return value;
      } else {
        throw std::runtime_error(
            fmt::format("Unable to read the 'value' attribute for the <{}> "
                        "tag on line {}",
                        element_name, value_node->GetLineNum()));
      }
    } else {
      throw std::runtime_error(
          fmt::format("Unable to find the <{}> "
                      "tag on line {}",
                      element_name, node->GetLineNum()));
    }
  };

  // Functor to read a child element with a string valued `name` attribute.
  // Throws an error if unable to find the tag of if the name attribute is
  // improperly formed.
  auto read_frame = [node,
                     plant](const char* element_name) -> const Frame<double>& {
    XMLElement* value_node = node->FirstChildElement(element_name);

    if (value_node != nullptr) {
      std::string frame_name;
      if (ParseStringAttribute(value_node, "name", &frame_name)) {
        if (!plant->HasFrameNamed(frame_name)) {
          throw std::runtime_error(fmt::format(
              "Frame: {} specified for <{}> does not exist in the model.",
              frame_name, element_name));
        }
        return plant->GetFrameByName(frame_name);

      } else {
        throw std::runtime_error(
            fmt::format("Unable to read the 'name' attribute for the <{}> "
                        "tag on line {}",
                        element_name, value_node->GetLineNum()));
      }

    } else {
      throw std::runtime_error(
          fmt::format("Unable to find the <{}> tag on line {}", element_name,
                      node->GetLineNum()));
    }
  };

  ParseLinearBushingRollPitchYaw(read_vector, read_frame, plant);
}

ModelInstanceIndex ParseUrdf(
    const std::string& model_name_in,
    const multibody::PackageMap& package_map,
    const std::string& root_dir,
    XMLDocument* xml_doc,
    MultibodyPlant<double>* plant) {

  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error("ERROR: URDF does not contain a robot tag.");
  }

  std::string model_name = model_name_in;
  if (model_name.empty() && !ParseStringAttribute(node, "name", &model_name)) {
      throw std::runtime_error(
          "ERROR: Your robot must have a name attribute or a model name "
          "must be specified.");
  }

  // Parses the model's material elements. Throws an exception if there's a
  // material name clash regardless of whether the associated RGBA values are
  // the same.
  MaterialMap materials;
  for (XMLElement* material_node = node->FirstChildElement("material");
       material_node;
       material_node = material_node->NextSiblingElement("material")) {
    ParseMaterial(material_node, true /* name_required */, package_map,
                  root_dir, &materials);
  }

  const ModelInstanceIndex model_instance =
      plant->AddModelInstance(model_name);

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link");
       link_node;
       link_node = link_node->NextSiblingElement("link")) {
    ParseBody(package_map, root_dir, model_instance, link_node,
              &materials, plant);
  }

  // Parses the collision filter groups only if the scene graph is registered.
  if (plant->geometry_source_is_registered()) {
    ParseCollisionFilterGroup(model_instance, node, plant);
  }

  // Joint effort limits are stored with joints, but used when creating the
  // actuator (which is done when parsing the transmission).
  std::map<std::string, double> joint_effort_limits;

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    ParseJoint(model_instance, &joint_effort_limits, joint_node, plant);
  }

  // Parses the model's transmission elements.
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission")) {
    ParseTransmission(model_instance, joint_effort_limits,
                      transmission_node, plant);
  }

  if (node->FirstChildElement("loop_joint")) {
    throw std::runtime_error(
        "ERROR: loop joints are not supported in MultibodyPlant");
  }

  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame")) {
    ParseFrame(model_instance, frame_node, plant);
  }

  // Parses the model's custom Drake bushing tags.
  for (XMLElement* bushing_node =
           node->FirstChildElement("drake:linear_bushing_rpy");
       bushing_node; bushing_node = bushing_node->NextSiblingElement(
                         "drake:linear_bushing_rpy")) {
    ParseBushing(bushing_node, plant);
  }

  return model_instance;
}

}  // namespace

ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const std::string& model_name_in,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  const std::string full_path = GetFullPath(file_name);

  // Opens the URDF file and feeds it into the XML parser.
  XMLDocument xml_doc;
  xml_doc.LoadFile(full_path.c_str());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error("Failed to parse XML in file " + full_path +
                             "\n" + xml_doc.ErrorName());
  }

  // Uses the directory holding the URDF to be the root directory
  // in which to search for files referenced within the URDF file.
  std::string root_dir = ".";
  size_t found = full_path.find_last_of("/\\");
  if (found != std::string::npos) {
    root_dir = full_path.substr(0, found);
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  return ParseUrdf(model_name_in, package_map, root_dir,
                   &xml_doc, plant);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
