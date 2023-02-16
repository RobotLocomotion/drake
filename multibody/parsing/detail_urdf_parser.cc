#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <drake_vendor/tinyxml2.h>
#include <fmt/format.h>

#include "drake/common/sorted_pair.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_make_model_name.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/detail_tinyxml2_diagnostic.h"
#include "drake/multibody/parsing/detail_urdf_geometry.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using tinyxml2::XMLNode;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

using JointEffortLimits = std::map<std::string, double>;

// Helper class to share infrastructure among parsing methods.
class UrdfParser {
 public:
  // Note that @p data_source, @p xml_doc, and @p w are aliased for the
  // lifetime of this object. Their lifetimes must exceed that of the created
  // object.
  UrdfParser(
      const DataSource* data_source,
      const std::string& model_name,
      const std::optional<std::string>& parent_model_name,
      const std::string& root_dir,
      XMLDocument* xml_doc,
      const ParsingWorkspace& w)
      : model_name_(model_name),
        parent_model_name_(parent_model_name),
        root_dir_(root_dir),
        xml_doc_(xml_doc),
        w_(w),
        diagnostic_(&w.diagnostic, data_source) {
    DRAKE_DEMAND(data_source != nullptr);
    DRAKE_DEMAND(xml_doc != nullptr);
  }

  // @return a model instance index, if one was created during parsing.
  // @throw std::exception on parse error.
  // @note: see AddModelFromUrdf for a full account of diagnostics, error
  // reporting, and return values.
  std::optional<ModelInstanceIndex> Parse();
  void ParseBushing(XMLElement* node);
  void ParseFrame(XMLElement* node);
  void ParseTransmission(const JointEffortLimits& joint_effort_limits,
                         XMLElement* node);
  void ParseJoint(JointEffortLimits* joint_effort_limits, XMLElement* node);
  const Body<double>* GetBodyForElement(const std::string& element_name,
                                        const std::string& link_name);
  void ParseJointDynamics(XMLElement* node, double* damping);
  void ParseJointLimits(XMLElement* node, double* lower, double* upper,
                        double* velocity, double* acceleration, double* effort);
  void ParseJointKeyParams(XMLElement* node,
                           std::string* name,
                           std::string* type,
                           std::string* parent_link_name,
                           std::string* child_link_name);
  void ParseScrewJointThreadPitch(XMLElement* node, double* screw_thread_pitch);
  void ParseCollisionFilterGroup(XMLElement* node);
  void ParseBody(XMLElement* node, MaterialMap* materials);
  SpatialInertia<double> ExtractSpatialInertiaAboutBoExpressedInB(
      XMLElement* node);

  // A work-alike for internal::ParseScalarAttribute() that uses the local
  // diagnostic policy.
  bool ParseScalarAttribute(
    const tinyxml2::XMLElement* node,
    const char* attribute_name, double* val) {
    return internal::ParseScalarAttribute(
        node, attribute_name, val,
        diagnostic_.MakePolicyForNode(node));
  }
  void ParseMechanicalReduction(const XMLElement& node);


  void Warning(const XMLNode& location, std::string message) const {
    diagnostic_.Warning(location, std::move(message));
  }

  void Error(const XMLNode& location, std::string message) const {
    diagnostic_.Error(location, std::move(message));
  }

  // Warn about documented URDF elements ignored by Drake.
  void WarnUnsupportedElement(const XMLElement& node, const std::string& tag) {
    diagnostic_.WarnUnsupportedElement(node, tag);
  }

  // Warn about documented URDF attributes ignored by Drake.
  void WarnUnsupportedAttribute(const XMLElement& node,
                                const std::string& attribute) {
    diagnostic_.WarnUnsupportedAttribute(node, attribute);
  }

 private:
  const std::string model_name_;
  const std::optional<std::string> parent_model_name_;
  const std::string root_dir_;
  XMLDocument* const xml_doc_;
  const ParsingWorkspace& w_;
  TinyXml2Diagnostic diagnostic_;

  ModelInstanceIndex model_instance_{};
};

const char* kWorldName = "world";

SpatialInertia<double> UrdfParser::ExtractSpatialInertiaAboutBoExpressedInB(
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

  // If this is a massless body, return a zero SpatialInertia.
  if (body_mass == 0. && I_BBcm_Bi.get_moments().isZero() &&
      I_BBcm_Bi.get_products().isZero()) {
    return SpatialInertia<double>(body_mass, {0., 0., 0.}, {0., 0., 0});
  }
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

void UrdfParser::ParseBody(XMLElement* node, MaterialMap* materials) {
  // TODO(rpoyner-tri): legacy undocumented tag: remove, fix?
  std::string drake_ignore;
  if (ParseStringAttribute(node, "drake_ignore", &drake_ignore) &&
      drake_ignore == std::string("true")) {
    return;
  }
  // Seen in the ROS urdfdom XSD Schema.
  // See https://github.com/ros/urdfdom/blob/dbecca0/xsd/urdf.xsd
  WarnUnsupportedAttribute(*node, "type");

  std::string body_name;
  if (!ParseStringAttribute(node, "name", &body_name)) {
    Error(*node, "link tag is missing name attribute.");
    return;
  }

  const RigidBody<double>* body_pointer{};
  if (body_name == kWorldName) {
    // TODO(SeanCurtis-TRI): We have no documentation about what our parsers
    //  support and not. The fact that we allow this behavior should be
    //  discoverable *somewhere*. But this function is in an anonymous
    //  namespace; where would the documentation go that supports this
    //  implementation?
    body_pointer = &w_.plant->world_body();
    if (node->FirstChildElement("inertial") != nullptr) {
      Warning(*node, "A URDF file declared the \"world\" link and then"
              " attempted to assign mass properties (via the <inertial> tag)."
              " Only geometries, <collision> and <visual>, can be assigned to"
              " the world link. The <inertial> tag is being ignored.");
    }
  } else {
    SpatialInertia<double> M_BBo_B;
    XMLElement* inertial_node = node->FirstChildElement("inertial");
    if (!inertial_node) {
      M_BBo_B = SpatialInertia<double>(0, Vector3d::Zero(),
                                       UnitInertia<double>(0, 0, 0));
    } else {
      M_BBo_B = ExtractSpatialInertiaAboutBoExpressedInB(inertial_node);
    }

    // Add a rigid body to model each link.
    body_pointer = &w_.plant->AddRigidBody(body_name, model_instance_, M_BBo_B);
  }

  if (w_.plant->geometry_source_is_registered()) {
    const RigidBody<double>& body = *body_pointer;
    std::unordered_set<std::string> geometry_names;

    for (XMLElement* visual_node = node->FirstChildElement("visual");
         visual_node;
         visual_node = visual_node->NextSiblingElement("visual")) {
      std::optional<geometry::GeometryInstance> geometry_instance =
          ParseVisual(diagnostic_, body_name, w_.package_map, root_dir_,
                      visual_node, materials, &geometry_names);
      if (!geometry_instance) { continue; }
      // The parsing should *always* produce an IllustrationProperties
      // instance, even if it is empty.
      DRAKE_DEMAND(geometry_instance->illustration_properties() != nullptr);
      w_.plant->RegisterVisualGeometry(
          body, geometry_instance->pose(), geometry_instance->shape(),
          geometry_instance->name(),
          *geometry_instance->illustration_properties());
    }

    for (XMLElement* collision_node = node->FirstChildElement("collision");
         collision_node;
         collision_node = collision_node->NextSiblingElement("collision")) {
      std::optional<geometry::GeometryInstance> geometry_instance =
          ParseCollision(diagnostic_, body_name, w_.package_map, root_dir_,
                         collision_node, &geometry_names);
      if (!geometry_instance) { continue; }
      DRAKE_DEMAND(geometry_instance->proximity_properties() != nullptr);
      w_.plant->RegisterCollisionGeometry(
          body, geometry_instance->pose(), geometry_instance->shape(),
          geometry_instance->name(),
          std::move(*geometry_instance->mutable_proximity_properties()));
    }
  }
}

void UrdfParser::ParseCollisionFilterGroup(XMLElement* node) {
  auto next_child_element = [](const ElementNode& data_element,
                               const char* element_name) {
    return std::get<XMLElement*>(data_element)
        ->FirstChildElement(element_name);
  };
  auto next_sibling_element = [](const ElementNode& data_element,
                                 const char* element_name) {
    return std::get<XMLElement*>(data_element)
        ->NextSiblingElement(element_name);
  };
  auto has_attribute = [](const ElementNode& data_element,
                          const char* attribute_name) {
    std::string attribute_value;
    return ParseStringAttribute(std::get<XMLElement*>(data_element),
                                attribute_name, &attribute_value);
  };
  auto get_string_attribute = [this](const ElementNode& data_element,
                                     const char* attribute_name) {
    std::string attribute_value;
    XMLElement* anode = std::get<XMLElement*>(data_element);
    if (!ParseStringAttribute(anode, attribute_name, &attribute_value)) {
      Error(*anode, fmt::format("The tag <{}> does not specify the required"
                                " attribute \"{}\".", anode->Value(),
                                attribute_name));
      // Fall through to return empty string.
    }
    return attribute_value;
  };
  auto get_bool_attribute = [](const ElementNode& data_element,
                               const char* attribute_name) {
    std::string attribute_value;
    ParseStringAttribute(std::get<XMLElement*>(data_element),
                         attribute_name, &attribute_value);
    return attribute_value == std::string("true") ? true : false;
  };
  ParseCollisionFilterGroupCommon(
      diagnostic_.MakePolicyForNode(node), model_instance_, node, w_.plant,
      w_.collision_resolver, next_child_element, next_sibling_element,
      has_attribute, get_string_attribute, get_bool_attribute,
      get_string_attribute);
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
void UrdfParser::ParseJointKeyParams(XMLElement* node,
                                     std::string* name,
                                     std::string* type,
                                     std::string* parent_link_name,
                                     std::string* child_link_name) {
  if (!ParseStringAttribute(node, "name", name)) {
    Error(*node, "joint tag is missing name attribute");
    return;
  }

  if (!ParseStringAttribute(node, "type", type)) {
    Error(*node, fmt::format("joint '{}' is missing type attribute", *name));
    return;
  }

  // Obtains the name of the joint's parent link.
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) {
    Error(*node, fmt::format("joint '{}' doesn't have a parent node!", *name));
    return;
  }
  if (!ParseStringAttribute(parent_node, "link", parent_link_name)) {
    Error(*parent_node, fmt::format("joint {}'s parent does not have a link"
                                    " attribute!", *name));
    return;
  }

  // Obtains the name of the joint's child link.
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node) {
    Error(*node, fmt::format("joint '{}' doesn't have a child node!", *name));
    return;
  }
  if (!ParseStringAttribute(child_node, "link", child_link_name)) {
    Error(*child_node, fmt::format("joint {}'s child does not have a link"
                                   " attribute!", *name));
    return;
  }
}

void UrdfParser::ParseJointLimits(
    XMLElement* node, double* lower, double* upper,
    double* velocity, double* acceleration, double* effort) {
  *lower = -std::numeric_limits<double>::infinity();
  *upper = std::numeric_limits<double>::infinity();
  *velocity = std::numeric_limits<double>::infinity();
  *acceleration = std::numeric_limits<double>::infinity();
  *effort = std::numeric_limits<double>::infinity();

  XMLElement* limit_node = node->FirstChildElement("limit");
  if (limit_node) {
    ParseScalarAttribute(limit_node, "lower", lower);
    ParseScalarAttribute(limit_node, "upper", upper);
    ParseScalarAttribute(limit_node, "velocity", velocity);
    ParseScalarAttribute(limit_node, "drake:acceleration", acceleration);
    ParseScalarAttribute(limit_node, "effort", effort);
  }
}

void UrdfParser::ParseJointDynamics(XMLElement* node, double* damping) {
  *damping = 0.0;
  double coulomb_friction = 0.0;
  double coulomb_window = std::numeric_limits<double>::epsilon();

  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (dynamics_node) {
    ParseScalarAttribute(dynamics_node, "damping", damping);
    if (ParseScalarAttribute(dynamics_node, "friction", &coulomb_friction) &&
        coulomb_friction != 0.0) {
      Warning(*dynamics_node, "A joint has specified a non-zero value for the"
              " 'friction' attribute of a joint/dynamics tag. MultibodyPlant"
              " does not currently support non-zero joint friction.");
    }
    if (ParseScalarAttribute(
            dynamics_node, "coulomb_window", &coulomb_window)) {
      Warning(*dynamics_node, "A joint has specified a value for the"
              " 'coulomb_window' attribute of a <joint> tag. Drake no longer"
              " makes use of that attribute; all instances will be ignored.");
    }
  }
}

void UrdfParser::ParseScrewJointThreadPitch(XMLElement* node,
                                            double* screw_thread_pitch) {
  // Always set a value for the output-only argument, even if parsing fails.
  *screw_thread_pitch = 0.0;
  XMLElement* screw_thread_pitch_node =
      node->FirstChildElement("drake:screw_thread_pitch");
  if (screw_thread_pitch_node) {
    if (!ParseScalarAttribute(screw_thread_pitch_node, "value",
                              screw_thread_pitch)) {
      Error(*screw_thread_pitch_node, "A screw joint has a"
            " <drake:screw_thread_pitch> tag that is missing the 'value'"
            " attribute.");
      return;
    }
  } else {
      Error(*node, "A screw joint is missing the <drake:screw_thread_pitch>"
            " tag.");
      return;
  }
}

const Body<double>* UrdfParser::GetBodyForElement(
    const std::string& element_name,
    const std::string& link_name) {
  auto plant = w_.plant;
  if (link_name == kWorldName) {
    return &plant->world_body();
  }

  if (!plant->HasBodyNamed(link_name, model_instance_)) {
    Error(*xml_doc_, fmt::format("Could not find link named '{}' with model"
                                 " instance ID {} for element '{}'.", link_name,
                                 model_instance_, element_name));
    return nullptr;
  }
  return &plant->GetBodyByName(link_name, model_instance_);
}

void UrdfParser::ParseJoint(
    JointEffortLimits* joint_effort_limits,
    XMLElement* node) {
  // TODO(rpoyner-tri): legacy undocumented tag: remove, fix?
  std::string drake_ignore;
  if (ParseStringAttribute(node, "drake_ignore", &drake_ignore) &&
      drake_ignore == std::string("true")) {
    return;
  }
  WarnUnsupportedElement(*node, "calibration");
  WarnUnsupportedElement(*node, "safety_controller");

  // Parses the parent and child link names.
  std::string name, type, parent_name, child_name;
  ParseJointKeyParams(node, &name, &type, &parent_name, &child_name);
  if (name.empty() || type.empty() ||
      parent_name.empty() || child_name.empty()) {
    return;
  }

  const Body<double>* parent_body = GetBodyForElement(
      name, parent_name);
  if (parent_body == nullptr) { return; }
  const Body<double>* child_body = GetBodyForElement(
      name, child_name);
  if (child_body == nullptr) { return; }

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
      Error(*axis_node, fmt::format("Joint '{}' axis is zero.  Don't do that.",
                                    name));
      return;
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
  double acceleration = std::numeric_limits<double>::infinity();

  // In MultibodyPlant, the effort limit is a property of the actuator, which
  // isn't created until the transmission element is parsed.  Stash a value
  // for all joints when parsing the joint element so that we can look it up
  // later if/when an actuator is created.
  double effort = std::numeric_limits<double>::infinity();

  auto throw_on_custom_joint =
      [node, name, type, this](bool want_custom_joint) {
    const std::string node_name(node->Name());
    const bool is_custom_joint = node_name == "drake:joint";
    if (want_custom_joint && !is_custom_joint) {
      Error(*node, fmt::format("Joint {} of type {} is a custom joint type, and"
                               " should be a <drake:joint>", name, type));
      return;
    } else if (!want_custom_joint && is_custom_joint) {
      Error(*node, fmt::format("Joint {} of type {} is a standard joint type,"
                               " and should be a <joint>", name, type));
      return;
    }
  };

  auto plant = w_.plant;
  std::optional<JointIndex> index{};
  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    throw_on_custom_joint(false);
    ParseJointLimits(node, &lower, &upper, &velocity, &acceleration, &effort);
    ParseJointDynamics(node, &damping);
    index = plant->AddJoint<RevoluteJoint>(
        name, *parent_body, X_PJ,
        *child_body, std::nullopt, axis, lower, upper, damping).index();
    Joint<double>& joint = plant->get_mutable_joint(*index);
    joint.set_velocity_limits(Vector1d(-velocity), Vector1d(velocity));
    joint.set_acceleration_limits(
        Vector1d(-acceleration), Vector1d(acceleration));
  } else if (type.compare("fixed") == 0) {
    throw_on_custom_joint(false);
    index = plant->AddJoint<WeldJoint>(
        name, *parent_body, X_PJ, *child_body, std::nullopt,
        RigidTransformd::Identity()).index();
  } else if (type.compare("prismatic") == 0) {
    throw_on_custom_joint(false);
    ParseJointLimits(node, &lower, &upper, &velocity, &acceleration, &effort);
    ParseJointDynamics(node, &damping);
    index = plant->AddJoint<PrismaticJoint>(
        name, *parent_body, X_PJ, *child_body, std::nullopt, axis, lower,
        upper, damping).index();
    Joint<double>& joint = plant->get_mutable_joint(*index);
    joint.set_velocity_limits(Vector1d(-velocity), Vector1d(velocity));
    joint.set_acceleration_limits(
        Vector1d(-acceleration), Vector1d(acceleration));
  } else if (type.compare("floating") == 0) {
    throw_on_custom_joint(false);
    Warning(*node, fmt::format("Joint '{}' specified as type floating which is"
                               " not supported by MultibodyPlant.  Leaving '{}'"
                               " as a free body.", name, child_name));
  } else if (type.compare("ball") == 0) {
    throw_on_custom_joint(true);
    ParseJointDynamics(node, &damping);
    index = plant->AddJoint<BallRpyJoint>(
      name, *parent_body, X_PJ, *child_body, std::nullopt, damping).index();
  } else if (type.compare("planar") == 0) {
    // Permit both the standard 'joint' and custom 'drake:joint' spellings
    // here. The standard spelling was actually always correct, but Drake only
    // supported the custom spelling for quite some time, and some model files
    // are likely spelled that way. See #18730.
    Vector3d damping_vec(0, 0, 0);
    XMLElement* dynamics_node = node->FirstChildElement("dynamics");
    if (dynamics_node) {
      ParseVectorAttribute(dynamics_node, "damping", &damping_vec);
    }
    index = plant->AddJoint<PlanarJoint>(
      name, *parent_body, X_PJ, *child_body, std::nullopt, damping_vec).index();
  } else if (type.compare("screw") == 0) {
    throw_on_custom_joint(true);
    ParseJointDynamics(node, &damping);
    double screw_thread_pitch;
    ParseScrewJointThreadPitch(node, &screw_thread_pitch);
    index = plant->AddJoint<ScrewJoint>(
      name, *parent_body, X_PJ, *child_body, std::nullopt, axis,
      screw_thread_pitch, damping).index();
  } else if (type.compare("universal") == 0) {
    throw_on_custom_joint(true);
    ParseJointDynamics(node, &damping);
    index = plant->AddJoint<UniversalJoint>(
      name, *parent_body, X_PJ, *child_body, std::nullopt, damping).index();
  } else {
    Error(*node, fmt::format("Joint '{}' has unrecognized type: '{}'",
                             name, type));
    return;
  }

  joint_effort_limits->emplace(name, effort);

  XMLElement* mimic_node = node->FirstChildElement("mimic");
  if (mimic_node) {
    if (!plant->is_discrete() ||
        plant->get_discrete_contact_solver() != DiscreteContactSolver::kSap) {
      Warning(
          *mimic_node,
          fmt::format("Joint '{}' specifies a mimic element that will be "
                      "ignored. Mimic elements are currently only supported by "
                      "MultibodyPlant with a discrete time step and using "
                      "DiscreteContactSolver::kSap.",
                      name));
    } else {
      std::string joint_to_mimic;
      double gear_ratio{1.0};
      double offset{0.0};
      if (!ParseStringAttribute(mimic_node, "joint", &joint_to_mimic)) {
        Error(*mimic_node,
              fmt::format("Joint '{}' mimic element is missing the "
                          "required 'joint' attribute.",
                          name));
        return;
      }
      if (!plant->HasJointNamed(joint_to_mimic, model_instance_)) {
        Error(*mimic_node,
              fmt::format("Joint '{}' mimic element specifies joint '{}' which"
                          " does not exist.",
                          name, joint_to_mimic));
        return;
      }
      ParseScalarAttribute(mimic_node, "multiplier", &gear_ratio);
      ParseScalarAttribute(mimic_node, "offset", &offset);

      if (!index) {
        // This can currently happen if we have a "floating" joint, which does
        // not produce the actual QuaternionFloatingJoint above.
        Warning(*mimic_node,
                fmt::format("Drake only supports the mimic element for "
                            "single-dof joints. The mimic element in joint "
                            "'{}' will be ignored.",
                            name));
      } else {
        const Joint<double>& joint0 = plant->get_joint(*index);
        const Joint<double>& joint1 =
            plant->GetJointByName(joint_to_mimic, model_instance_);
        if (joint1.num_velocities() != joint0.num_velocities()) {
          Error(*mimic_node,
                fmt::format("Joint '{}' which has {} DOF cannot mimic "
                            "joint '{}' which has {} DOF.",
                            name, joint0.num_velocities(), joint_to_mimic,
                            joint1.num_velocities()));
          return;
        }
        if (joint0.num_velocities() != 1) {
          // The URDF documentation is ambiguous as to whether multi-dof joints
          // are supported by the mimic tag. So we only raise a warning, not an
          // error.
          Warning(*mimic_node,
                  fmt::format("Drake only supports the mimic element for "
                              "single-dof joints. The joint '{}' (with {} "
                              "dofs) is attempting to mimic joint '{}' (with "
                              "{} dofs). The mimic element will be ignored.",
                              name, joint0.num_velocities(), joint_to_mimic,
                              joint1.num_velocities()));
        } else {
          plant->AddCouplerConstraint(joint0, joint1, gear_ratio, offset);
        }
      }
    }
  }
}

void UrdfParser::ParseMechanicalReduction(const XMLElement& node) {
  const XMLElement* child = node.FirstChildElement("mechanicalReduction");
  if (!child) { return; }
  const char* text = child->GetText();
  if (!text) { return; }
  std::vector<double> values = ConvertToVector<double>(text);
  if (values.size() == 1 && values[0] == 1) { return; }
  Warning(*child, fmt::format(
              "A '{}' element contains a mechanicalReduction element with a"
              " value '{}' other than the default of 1. MultibodyPlant does"
              " not currently support non-default mechanical reductions.",
              node.Name(), text));
}

void UrdfParser::ParseTransmission(
    const JointEffortLimits& joint_effort_limits,
    XMLElement* node) {
  WarnUnsupportedElement(*node, "leftActuator");
  WarnUnsupportedElement(*node, "rightActuator");
  WarnUnsupportedElement(*node, "flexJoint");
  WarnUnsupportedElement(*node, "rollJoint");
  WarnUnsupportedElement(*node, "gap_joint");
  WarnUnsupportedElement(*node, "passive_joint");
  WarnUnsupportedElement(*node, "use_simulated_gripper_joint");
  ParseMechanicalReduction(*node);

  // Determines the transmission type.
  std::string type;
  XMLElement* type_node = node->FirstChildElement("type");
  if (type_node) {
    type = type_node->GetText();
  } else {
    // Old URDF format, kept for convenience
    if (!ParseStringAttribute(node, "type", &type)) {
      Error(*node, "Transmission element is missing a type.");
      return;
    }
  }

  // Checks if the transmission type is not SimpleTransmission. If it is not,
  // print a warning and then abort this method call since only simple
  // transmissions are supported at this time.
  if (type.find("SimpleTransmission") == std::string::npos) {
    Warning(*node, "A <transmission> has a type that isn't"
            " 'SimpleTransmission'. Drake only supports 'SimpleTransmission';"
            " all other transmission types will be ignored.");
    return;
  }

  // Determines the actuator's name.
  XMLElement* actuator_node = node->FirstChildElement("actuator");
  if (!actuator_node) {
    Error(*node, "Transmission is missing an actuator element.");
    return;
  }
  ParseMechanicalReduction(*actuator_node);
  // `actuator/hardwareInterface` child tags are silently ignored.

  std::string actuator_name;
  if (!ParseStringAttribute(actuator_node, "name", &actuator_name)) {
    Error(*actuator_node, "Transmission is missing an actuator name.");
    return;
  }

  // Determines the name of the joint to which the actuator is attached.
  XMLElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node) {
    Error(*node, "Transmission is missing a joint element.");
    return;
  }
  // `joint/hardwareInterface` child tags are silently ignored.


  std::string joint_name;
  if (!ParseStringAttribute(joint_node, "name", &joint_name)) {
    Error(*joint_node, "Transmission is missing a joint name.");
    return;
  }

  auto plant = w_.plant;
  if (!plant->HasJointNamed(joint_name, model_instance_)) {
    Error(*joint_node, fmt::format("Transmission specifies joint '{}' which"
                                   " does not exist.", joint_name));
    return;
  }
  const Joint<double>& joint = plant->GetJointByName(
      joint_name, model_instance_);

  // Checks if the actuator is attached to a fixed joint. If so, abort this
  // method call.
  if (joint.num_positions() == 0) {
    Warning(*joint_node, fmt::format("Skipping transmission since it's attached"
                                     " to a fixed joint \"{}\".", joint_name));
    return;
  }

  const auto effort_iter = joint_effort_limits.find(joint_name);
  DRAKE_DEMAND(effort_iter != joint_effort_limits.end());
  if (effort_iter->second < 0) {
    Error(*joint_node, fmt::format("Transmission specifies joint '{}' which has"
                                   " a negative effort limit.", joint_name));
    return;
  }

  if (effort_iter->second <= 0) {
    Warning(*joint_node, fmt::format("Skipping transmission since it's attached"
                                     " to joint \"{}\" which has a zero effort"
                                     " limit {}.", joint_name,
                                     effort_iter->second));
    return;
  }

  const JointActuator<double>& actuator =
      plant->AddJointActuator(actuator_name, joint, effort_iter->second);

  // Parse and add the optional drake:rotor_inertia parameter.
  XMLElement* rotor_inertia_node =
      actuator_node->FirstChildElement("drake:rotor_inertia");
  if (rotor_inertia_node) {
    double rotor_inertia;
    if (!ParseScalarAttribute(rotor_inertia_node, "value", &rotor_inertia)) {
      Error(*rotor_inertia_node,
            fmt::format("joint actuator {}'s drake:rotor_inertia does not have"
                        " a \"value\" attribute!", actuator_name));
      return;
    }
    plant->get_mutable_joint_actuator(actuator.index())
        .set_default_rotor_inertia(rotor_inertia);
  }

  // Parse and add the optional drake:gear_ratio parameter.
  XMLElement* gear_ratio_node =
      actuator_node->FirstChildElement("drake:gear_ratio");
  if (gear_ratio_node) {
    double gear_ratio;
    if (!ParseScalarAttribute(gear_ratio_node, "value", &gear_ratio)) {
      Error(*gear_ratio_node,
            fmt::format("joint actuator {}'s drake:gear_ratio does not have a"
                        " \"value\" attribute!", actuator_name));
      return;
    }
    plant->get_mutable_joint_actuator(actuator.index())
        .set_default_gear_ratio(gear_ratio);
  }
}

void UrdfParser::ParseFrame(XMLElement* node) {
  std::string name;
  if (!ParseStringAttribute(node, "name", &name)) {
    Error(*node, "parsing frame name.");
    return;
  }

  std::string body_name;
  if (!ParseStringAttribute(node, "link", &body_name)) {
    Error(*node, fmt::format("missing link name for frame {}.", name));
    return;
  }

  const Body<double>* body = GetBodyForElement(
      name, body_name);
  if (body == nullptr) { return; }

  RigidTransformd X_BF = OriginAttributesToTransform(node);
  w_.plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      name, body->body_frame(), X_BF));
}

void UrdfParser::ParseBushing(XMLElement* node) {
  // Functor to read a child element with a vector valued `value` attribute.
  // Returns a zero vector if unable to find the tag or if the value attribute
  // is improperly formed.
  auto read_vector = [node, this](const char* element_name) -> Eigen::Vector3d {
    const XMLElement* value_node = node->FirstChildElement(element_name);
    if (value_node != nullptr) {
      Eigen::Vector3d value;
      if (ParseVectorAttribute(value_node, "value", &value)) {
        return value;
      } else {
        Error(*node, fmt::format("Unable to read the 'value' attribute for the"
                                 " <{}> tag", element_name));
        return Eigen::Vector3d::Zero();
      }
    } else {
      Error(*node, fmt::format("Unable to find the <{}> tag", element_name));
      return Eigen::Vector3d::Zero();
    }
  };

  // Functor to read a child element with a string-valued `name` attribute.
  // Returns nullptr if unable to find the tag, if the name attribute is
  // improperly formed, or if it does not refer to a frame already in the
  // model.
  auto read_frame = [node, this](const char* element_name)
                    -> const Frame<double>* {
    XMLElement* value_node = node->FirstChildElement(element_name);

    if (value_node != nullptr) {
      std::string frame_name;
      auto plant = w_.plant;
      if (ParseStringAttribute(value_node, "name", &frame_name)) {
        if (!plant->HasFrameNamed(frame_name, model_instance_)) {
          Error(*value_node, fmt::format("Frame: {} specified for <{}> does not"
                                         " exist in the model.", frame_name,
                                         element_name));
          return {};
        }
        return &plant->GetFrameByName(frame_name, model_instance_);
      } else {
        Error(*value_node, fmt::format("Unable to read the 'name' attribute for"
                                       " the <{}> tag", element_name));
        return {};
      }

    } else {
      Error(*node, fmt::format("Unable to find the <{}> tag", element_name));
      return {};
    }
  };

  ParseLinearBushingRollPitchYaw(read_vector, read_frame, w_.plant);
}

std::optional<ModelInstanceIndex> UrdfParser::Parse() {
  XMLElement* node = xml_doc_->FirstChildElement("robot");
  if (!node) {
    Error(*xml_doc_, "URDF does not contain a robot tag.");
    return {};
  }
  // See https://github.com/ros/urdfdom/blob/dbecca0/urdf_parser/src/model.cpp#L124-L131
  WarnUnsupportedAttribute(*node, "version");
  // <gazebo> child tags are silently ignored.

  std::string model_name = model_name_;
  if (model_name.empty() && !ParseStringAttribute(node, "name", &model_name)) {
    Error(*node, "Your robot must have a name attribute or a model name "
          "must be specified.");
    return {};
  }

  model_name = MakeModelName(model_name, parent_model_name_, w_);
  model_instance_ = w_.plant->AddModelInstance(model_name);

  // Parses the model's material elements. Throws an exception if there's a
  // material name clash regardless of whether the associated RGBA values are
  // the same.
  MaterialMap materials;
  for (XMLElement* material_node = node->FirstChildElement("material");
       material_node;
       material_node = material_node->NextSiblingElement("material")) {
    ParseMaterial(diagnostic_, material_node, true /* name_required */,
                  w_.package_map, root_dir_, &materials);
  }

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link");
       link_node;
       link_node = link_node->NextSiblingElement("link")) {
    ParseBody(link_node, &materials);
  }

  // Parses the collision filter groups only if the scene graph is registered.
  if (w_.plant->geometry_source_is_registered()) {
    ParseCollisionFilterGroup(node);
  }

  // Joint effort limits are stored with joints, but used when creating the
  // actuator (which is done when parsing the transmission).
  JointEffortLimits joint_effort_limits;

  // Parses the model's joint elements.  While it may not be strictly required
  // to be true in MultibodyPlant, generally the JointIndex for any given
  // joint follows the declaration order in the model (and users probably
  // should avoid caring about the ordering of JointIndex), we still parse the
  // joints in model order regardless of the element type so that the results
  // are consistent with an SDF version of the same model.
  for (XMLElement* joint_node = node->FirstChildElement(); joint_node;
       joint_node = joint_node->NextSiblingElement()) {
    const std::string node_name(joint_node->Name());
    if (node_name == "joint" || node_name == "drake:joint") {
      ParseJoint(&joint_effort_limits, joint_node);
    }
  }

  // Parses the model's transmission elements.
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission")) {
    ParseTransmission(joint_effort_limits, transmission_node);
  }

  if (node->FirstChildElement("loop_joint")) {
    Error(*node, "loop joints are not supported in MultibodyPlant");
    return model_instance_;
  }

  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame")) {
    ParseFrame(frame_node);
  }

  // Parses the model's custom Drake bushing tags.
  for (XMLElement* bushing_node =
           node->FirstChildElement("drake:linear_bushing_rpy");
       bushing_node; bushing_node = bushing_node->NextSiblingElement(
                         "drake:linear_bushing_rpy")) {
    ParseBushing(bushing_node);
  }

  return model_instance_;
}

}  // namespace

std::optional<ModelInstanceIndex> AddModelFromUrdf(
    const DataSource& data_source,
    const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  MultibodyPlant<double>* plant = workspace.plant;
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());
  TinyXml2Diagnostic diag(&workspace.diagnostic, &data_source);

  // Opens the URDF file and feeds it into the XML parser.
  XMLDocument xml_doc;
  if (data_source.IsFilename()) {
    xml_doc.LoadFile(data_source.filename().c_str());
    if (xml_doc.ErrorID()) {
      diag.Error(xml_doc, fmt::format("Failed to parse XML file: {}",
                                      xml_doc.ErrorName()));
      return std::nullopt;
    }
  } else {
    xml_doc.Parse(data_source.contents().c_str());
    if (xml_doc.ErrorID()) {
      diag.Error(xml_doc, fmt::format("Failed to parse XML string: {}",
                                      xml_doc.ErrorName()));
      return std::nullopt;
    }
  }

  UrdfParser parser(&data_source, model_name_in, parent_model_name,
                    data_source.GetRootDir(), &xml_doc, workspace);
  return parser.Parse();
}

UrdfParserWrapper::UrdfParserWrapper() {}

UrdfParserWrapper::~UrdfParserWrapper() {}

std::optional<ModelInstanceIndex> UrdfParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddModelFromUrdf(data_source, model_name, parent_model_name,
                          workspace);
}

std::vector<ModelInstanceIndex> UrdfParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  auto result = AddModel(data_source, {}, parent_model_name, workspace);
  if (result.has_value()) {
    return {*result};
  }
  return {};
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
