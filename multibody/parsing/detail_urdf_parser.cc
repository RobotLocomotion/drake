#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <limits>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "drake/math/rotation_matrix.h"
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
namespace detail {

using Eigen::Isometry3d;
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

  Isometry3d X_BBi = Isometry3d::Identity();

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
  const math::RotationMatrix<double> R_BBi(X_BBi.linear());

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
               MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph) {
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
          body, RigidTransformd(geometry_instance.pose()),
          geometry_instance.shape(), geometry_instance.name(),
          *geometry_instance.illustration_properties(), scene_graph);
    }

    for (XMLElement* collision_node = node->FirstChildElement("collision");
         collision_node;
         collision_node = collision_node->NextSiblingElement("collision")) {
      CoulombFriction<double> friction;
      geometry::GeometryInstance geometry_instance =
          ParseCollision(body_name, package_map, root_dir, collision_node,
                         &friction);
      plant->RegisterCollisionGeometry(
          body, RigidTransformd(geometry_instance.pose()),
          geometry_instance.shape(), geometry_instance.name(), friction,
          scene_graph);
    }
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
                      double* velocity) {
  *lower = -std::numeric_limits<double>::infinity();
  *upper = std::numeric_limits<double>::infinity();
  *velocity = std::numeric_limits<double>::infinity();

  XMLElement* limit_node = node->FirstChildElement("limit");
  if (limit_node) {
    ParseScalarAttribute(limit_node, "lower", lower);
    ParseScalarAttribute(limit_node, "upper", upper);
    ParseScalarAttribute(limit_node, "velocity", velocity);
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

  Isometry3d X_PJ = Isometry3d::Identity();
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

  // These are only used by some joint types.
  double upper = 0;
  double lower = 0;
  double damping = 0;
  double velocity = 0;

  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    ParseJointLimits(node, &lower, &upper, &velocity);
    ParseJointDynamics(name, node, &damping);
    const JointIndex index = plant->AddJoint<RevoluteJoint>(
        name, parent_body, X_PJ,
        child_body, nullopt, axis, lower, upper, damping).index();
    Joint<double>& joint = plant->get_mutable_joint(index);
    joint.set_velocity_limits(Vector1d(-velocity), Vector1d(velocity));
  } else if (type.compare("fixed") == 0) {
    plant->AddJoint<WeldJoint>(name, parent_body, X_PJ,
                               child_body, nullopt,
                               Isometry3d::Identity());
  } else if (type.compare("prismatic") == 0) {
    ParseJointLimits(node, &lower, &upper, &velocity);
    ParseJointDynamics(name, node, &damping);
    const JointIndex index = plant->AddJoint<PrismaticJoint>(
        name, parent_body, X_PJ,
        child_body, nullopt, axis, lower, upper, damping).index();
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
}

void ParseTransmission(ModelInstanceIndex model_instance,
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

  plant->AddJointActuator(actuator_name, joint);
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

  Isometry3d X_BF = OriginAttributesToTransform(node);
  plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      name, body.body_frame(), X_BF));
}

ModelInstanceIndex ParseUrdf(
    const std::string& model_name_in,
    const multibody::PackageMap& package_map,
    const std::string& root_dir,
    XMLDocument* xml_doc,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {

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
    ParseMaterial(material_node, &materials);
  }

  const ModelInstanceIndex model_instance =
      plant->AddModelInstance(model_name);

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link");
       link_node;
       link_node = link_node->NextSiblingElement("link")) {
    ParseBody(package_map, root_dir, model_instance, link_node,
              &materials, plant, scene_graph);
  }

  // TODO(sam.creasey) Parse collision filter groups.
  if (node->FirstChildElement("collision_filter_group")) {
    drake::log()->warn("Skipping collision_filter_group elements");
  }

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    ParseJoint(model_instance, joint_node, plant);
  }

  // Parses the model's transmission elements.
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission")) {
    ParseTransmission(model_instance, transmission_node, plant);
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
                   &xml_doc, plant, scene_graph);
}

}  // namespace detail
}  // namespace multibody
}  // namespace drake
