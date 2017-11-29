#include "drake/multibody/parsers/urdf_parser.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/drake_joints.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/material_map.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/xml_util.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"

namespace drake {
namespace parsers {
namespace urdf {

using std::allocate_shared;
using std::cerr;
using std::endl;
using std::max;
using std::numeric_limits;
using std::runtime_error;
using std::ostream;
using std::string;
using std::stringstream;
using std::unique_ptr;
using std::vector;

using Eigen::Isometry3d;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using drake::parsers::ModelInstanceIdTable;
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kRollPitchYaw;

namespace {

void ParseInertial(RigidBody<double>* body, XMLElement* node) {
  Isometry3d T = Isometry3d::Identity();

  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T);

  XMLElement* mass = node->FirstChildElement("mass");
  if (mass) {
    double body_mass = 0;
    parseScalarAttribute(mass, "value", body_mass);
    body->set_mass(body_mass);
  }

  Eigen::Vector3d com(T(0, 3), T(1, 3), T(2, 3));
  body->set_center_of_mass(com);

  drake::SquareTwistMatrix<double> I = drake::SquareTwistMatrix<double>::Zero();
  I.block(3, 3, 3, 3) << body->get_mass() * Matrix3d::Identity();

  XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    parseScalarAttribute(inertia, "ixx", I(0, 0));
    parseScalarAttribute(inertia, "ixy", I(0, 1));
    I(1, 0) = I(0, 1);
    parseScalarAttribute(inertia, "ixz", I(0, 2));
    I(2, 0) = I(0, 2);
    parseScalarAttribute(inertia, "iyy", I(1, 1));
    parseScalarAttribute(inertia, "iyz", I(1, 2));
    I(2, 1) = I(1, 2);
    parseScalarAttribute(inertia, "izz", I(2, 2));
  }

  body->set_spatial_inertia(transformSpatialInertia(T, I));
}

// Adds a material to the supplied @materials map. Currently, only simple colors
// are supported.
//
// TODO(liang.fok) Add support for texture-based materials. See:
// https://github.com/RobotLocomotion/drake/issues/2588.
//
// @param[in] material_name A human-understandable name of the material.
//
// @param[in] color_rgba The red-green-blue-alpha color values of the material.
// The range of values is [0, 1].
//
// @param[in] abort_if_name_clash If true, this method will abort if
// @p material_name is already in @p materials regardless of whether the RGBA
// values are the same. If false, this method will abort if
// @p material_name is already in @p materials and the infinity norm of the
// difference is greater than 1e-10.
//
// @param[out] materials A pointer to the map in which to store the material.
// This cannot be nullptr.
void AddMaterialToMaterialMap(const string& material_name,
                              const Vector4d& color_rgba,
                              bool abort_if_name_clash,
                              MaterialMap* materials) {
  // Verifies that parameter materials is not nullptr.
  DRAKE_DEMAND(materials);

  // Determines if the material is already in the map.
  auto material_iter = materials->find(material_name);
  if (material_iter != materials->end()) {
    // The material is already in the map. Checks whether the old material is
    // the same as the new material.  The range of values in the RGBA vectors
    // is [0, 1].
    const auto& existing_color = material_iter->second;
    if (abort_if_name_clash || (color_rgba != existing_color)) {
      // The materials map already has the material_name key but the color
      // associated with it is different.
      stringstream error_buff;
      error_buff << string(__FILE__) + ": " + __func__ + ": ERROR: "
                 << "Material \"" + material_name + "\" was previously "
                 << "defined.\n"
                 << "  - existing RGBA values: " << existing_color.transpose()
                 << std::endl
                 << "  - new RGBA values: " << color_rgba.transpose()
                 << std::endl;
      throw std::runtime_error(error_buff.str());
    }
  } else {
    // Adds the new color to the materials map.
    (*materials)[material_name] = color_rgba;
  }
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void ParseMaterial(XMLElement* node, MaterialMap& materials) {
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0) {
    throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Material tag is missing a name.");
  }
  string name(attr);

  Vector4d rgba = Vector4d::Zero();  // Defaults to black.

  XMLElement* color_node = node->FirstChildElement("color");

  if (color_node) {
    if (!parseVectorAttribute(color_node, "rgba", rgba)) {
      throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
          "Color tag is missing rgba attribute.");
    }
    AddMaterialToMaterialMap(name, rgba, true /* abort_if_name_clash */,
        &materials);
  } else {
    // If no color was specified and the material is not in the materials map,
    // check if the material is texture-based. If it is, print a warning, use
    // default color (black), and then return.
    //
    // Otherwise, throw an exception.
    //
    // TODO(liang.fok): Update this logic once texture-based materials are
    // supported. See: https://github.com/RobotLocomotion/drake/issues/2588.
    if (materials.find(name) == materials.end()) {
      XMLElement* texture_node = node->FirstChildElement("texture");

      if (texture_node) {
        std::cerr
            << string(__FILE__) + ": " + __func__ + ": WARNING: Material \""
            << name << "\" is a texture. Textures are currently not supported. "
            << "For more information, see: "
            << "https://github.com/RobotLocomotion/drake/issues/2588. "
               "Defaulting to use the black color for this material."
            << endl;
        AddMaterialToMaterialMap(name, rgba, true /* abort_if_name_clash */,
            &materials);
      } else {
        throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": "
            "ERROR: Material\"" + name + "\" not previously defined. Therefore "
            "a color must be specified.");
      }

      return;
    }
  }
}

bool ParseGeometry(XMLElement* node, const PackageMap& package_map,
                   const string& root_dir,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   DrakeShapes::Element& element) {
  // DEBUG
  // cout << "ParseGeometry: START" << endl;
  // END_DEBUG
  const char* attr;
  XMLElement* shape_node;
  if ((shape_node = node->FirstChildElement("box"))) {
    double x = 0, y = 0, z = 0;
    attr = shape_node->Attribute("size");
    if (attr) {
      stringstream s(attr);
      s >> x >> y >> z;
    } else {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR parsing box "
          "element size" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Box(Vector3d(x, y, z)));
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    double r = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR parsing sphere "
          "element radius" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Sphere(max(DrakeShapes::MIN_RADIUS, r)));
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR parsing cylinder "
          "element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR parsing cylinder "
         "element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Cylinder(r, l));
  } else if ((shape_node = node->FirstChildElement("capsule"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR parsing capsule "
          "element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR: Failed to parse "
          "capsule element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    attr = shape_node->Attribute("filename");
    if (!attr) {
      cerr << string(__FILE__) + ": " + __func__ + ": ERROR: Mesh element has "
          "no filename tag" << endl;
      return false;
    }
    string filename(attr);

    // This method will return an empty string if the file is not found or
    // resolved within a ROS package.
    string resolved_filename =
        ResolveFilename(filename, package_map, root_dir);

    if (resolved_filename.empty()) {
      throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
          "Mesh file name could not be resolved from the provided uri \"" +
          filename + "\".");
    }
    DrakeShapes::Mesh mesh(filename, resolved_filename);

    // Obtains the scale of the mesh if it exists.
    if (shape_node->Attribute("scale") != nullptr)
      ParseThreeVectorAttribute(shape_node, "scale", &mesh.scale_);

    element.setGeometry(mesh);
  } else {
    cerr << string(__FILE__) + ": " + __func__ + ": Warning: geometry element "
      "has an unknown type and will be ignored." << endl;
  }
  // DEBUG
  // cout << "ParseGeometry: END" << endl;
  // END_DEBUG
  return true;
}

// Parses the URDF visual specification of a link. Currently, only colors are
// supported.
//
// TODO(liang.fok) Add support for textures. See:
// https://github.com/RobotLocomotion/drake/issues/2588
//
// For color visualizations that are named, this method adds the name and color
// tuple into the materials map.
//
// A warning is printed to std::cerr if a material is not set for the rigid
// body's visualization.
void ParseVisual(RigidBody<double>* body, XMLElement* node,
                 MaterialMap* materials, const PackageMap& package_map,
                 const string& root_dir) {
  // Ensures there is a geometry child element. Since this is a required
  // element, throws an exception if a geometry element does not exist.
  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: Link " +
        body->get_name() + " has a visual element without geometry.");
  }

  // Obtains the reference frame of the visualization relative to the reference
  // frame of the rigid body that is being visualized. It defaults to identity
  // if no transform is specified.
  Isometry3d T_element_to_link = Isometry3d::Identity();
  {
    XMLElement* origin = node->FirstChildElement("origin");
    if (origin) originAttributesToTransform(origin, T_element_to_link);
  }
  DrakeShapes::VisualElement element(T_element_to_link);

  // Parses the geometry specifications of the visualization.
  if (!ParseGeometry(geometry_node, package_map, root_dir, element)) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: Failed "
        "to parse visual element in link " + body->get_name() + ".");
  }

  // Parses the material specification of the visualization. Note that we cannot
  // reuse the logic within ParseMaterial() here because the context is
  // different. Whereas ParseMaterial() parses material specifications that
  // are children elements of the "robot" element, the material specification
  // being parsed here are children of a "visual" element. One key difference is
  // the XML here may not specify a "name" attribute. Because of this difference
  // in context, we need specialized logic here to determine the material
  // visualization of a link.
  XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    // Checks and remembers whether a "color" child element exists. If so,
    // parses the color value.
    bool color_specified = false;
    Vector4d rgba;
    {
      XMLElement* color_node = material_node->FirstChildElement("color");
      if (color_node) {
        if (!parseVectorAttribute(color_node, "rgba", rgba)) {
          throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
              "Failed to parse color of material for model \"" +
              body->get_model_name() + "\", link \"" + body->get_name() +
              "\".");
        }
        color_specified = true;
      }
    }

    // Checks and remembers whether a "name" attribute exists. If so, parses the
    // name value.
    string material_name;
    bool name_specified = false;
    {
      const char* attr = material_node->Attribute("name");

      if (attr != nullptr && strlen(attr) != 0) {
        material_name = string(attr);
        name_specified = true;
      }
    }

    // Adds the material to the materials map if both the name and color are
    // specified. This is so that link elements that reside later in the URDF
    // can reference this material in their visualization elements. Note that
    // this capability is not specified by the official URDF specification (see:
    // http://wiki.ros.org/urdf/XML/link), but is needed by certain URDFs
    // released by companies and organizations like Robotiq and ROS Industrial
    // (for example, see this URDF by Robotiq: http://bit.ly/28P0pmo).
    if (color_specified && name_specified) {
      // The `abort_if_name_clash` parameter is passed a value of `false` to
      // allow the same material to be defined across multiple links as long as
      // they correspond to the same RGBA value. This can happen, for example,
      // in URDFs that are automatically generated using `xacro` since `xacro`
      // may produce a URDF from multiple `.xacro` files. Through testing, we
      // determined that the Gazebo simulator supports loading URDFs containing
      // duplicate material specifications as long as the duplicates are
      // distributed across multiple `<link>` elements and are not at the
      // `<robot>` level.
      AddMaterialToMaterialMap(material_name, rgba,
          false /* abort_if_name_clash */, materials);
    }

    // Sets the material's color.
    bool material_set = false;
    {
      // If the color is specified as a child element of the current material
      // node, use that color. It takes precedence over any material saved in
      // the material map.
      if (color_specified) {
        element.setMaterial(rgba);
        material_set = true;
      } else {
        // No color specified. Checks if the material is already in the
        // materials map.
        if (name_specified) {
          auto material_iter = materials->find(material_name);
          if (material_iter != materials->end()) {
            // The material is in the map. Sets the material of the visual
            // element based on the value in the map.
            element.setMaterial(material_iter->second);
            material_set = true;
          }
        }
      }
    }

    // Throws a std::runtime_error if the material was not set for this
    // visualization.
    //
    // TODO(liang.fok): Update this logic once texture-based materials are
    // supported. See: https://github.com/RobotLocomotion/drake/issues/2588.
    if (!material_set) {
      stringstream error_buff;
      error_buff
          << string(__FILE__) + ": " + __func__ + ": WARNING: Visual element "
          << "has a material whose color could not be determined.\n"
          << "  - model name: " << body->get_model_name() << "\n"
          << "  - body name: " << body->get_name() << "\n"
          << "  - material name: " << material_name << "\n";
      throw std::runtime_error(error_buff.str());
    }
  }

  if (element.hasGeometry()) body->AddVisualElement(element);
}

void ParseCollision(RigidBody<double>* body, XMLElement* node,
                    RigidBodyTree<double>* tree,
                    const PackageMap& package_map, const string& root_dir) {
  Isometry3d T_element_to_link = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T_element_to_link);

  const char* attr;
  string group_name;

  attr = node->Attribute("group");
  if (attr) {
    group_name = attr;
  } else {
    group_name = "default";
  }

  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: Link " +
        body->get_name() + " has a collision element without geometry");
  }

  drake::multibody::collision::Element element(T_element_to_link, body);
  if (!ParseGeometry(geometry_node, package_map, root_dir, element)) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: Failed "
        "to parse collision element in link " + body->get_name() + ".");
  }

  element.set_compliant_material(ParseCollisionCompliance(node));

  if (element.hasGeometry()) {
    tree->addCollisionElement(element, *body, group_name);
  }
}

bool ParseBody(RigidBodyTree<double>* tree, string robot_name, XMLElement* node,
               MaterialMap* materials, const PackageMap& package_map,
               const string& root_dir, int model_instance_id, int* index) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return false;

  RigidBody<double>* body{nullptr};
  std::unique_ptr<RigidBody<double>> owned_body(body = new RigidBody<double>());
  body->set_model_name(robot_name);
  body->set_model_instance_id(model_instance_id);

  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": "
        "ERROR: link tag is missing name attribute.");
  }

  // World links are handled by ParseWorldJoint().
  body->set_name(attr);
  if (body->get_name() == string(RigidBodyTreeConstants::kWorldName))
    return false;

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) ParseInertial(body, inertial_node);

  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    ParseVisual(body, visual_node, materials, package_map, root_dir);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    ParseCollision(body, collision_node, tree, package_map, root_dir);
  }

  tree->add_rigid_body(std::move(owned_body));
  *index = body->get_body_index();
  return true;
}

template <typename JointType>
void SetLimits(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* limit_node = node->FirstChildElement("limit");
  if (fjoint != nullptr && limit_node) {
    double lower = -numeric_limits<double>::infinity(),
           upper = numeric_limits<double>::infinity();
    parseScalarAttribute(limit_node, "lower", lower);
    parseScalarAttribute(limit_node, "upper", upper);
    fjoint->setJointLimits(lower, upper);
  }
}

template <typename JointType>
void SetDynamics(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (fjoint != nullptr && dynamics_node) {
    double damping = 0.0, coulomb_friction = 0.0;
    double coulomb_window = std::numeric_limits<double>::epsilon();
    parseScalarAttribute(dynamics_node, "damping", damping);
    parseScalarAttribute(dynamics_node, "friction", coulomb_friction);
    parseScalarAttribute(dynamics_node, "coulomb_window", coulomb_window);
    fjoint->setDynamics(damping, coulomb_friction, coulomb_window);
  }
}

/**
 * Parses the URDF collision filter group specification. Attempts to add
 * collision filter groups (with their member lists and ignore lists) to the
 * tree specification.  Inconsistent definitions will lead to thrown
 * exceptions.
 *
 * @param tree                  The rigid body tree containing the bodies to
 *                              which the filters will be applied.
 * @param node                  The XML node containing the filter details.
 * @param model_instance_id     The id of the current model instance.
 */
void ParseCollisionFilterGroup(RigidBodyTree<double>* tree, XMLElement* node,
                               int model_instance_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return;

  // TODO(SeanCurtis-TRI): After upgrading to newest tinyxml, add line numbers
  // to error messages.
  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Collision filter group specification missing name attribute.");
  }
  string group_name(attr);

  tree->DefineCollisionFilterGroup(group_name);

  for (XMLElement* member_node = node->FirstChildElement("member"); member_node;
       member_node = member_node->NextSiblingElement("member")) {
    const char* link_name = member_node->Attribute("link");
    if (!link_name) {
      throw runtime_error(string(__FILE__) + ": " + __func__ + ": Collision "
          "filter group " + group_name + " provides a member tag without "
          "specifying the \"link\" attribute.");
    }
    tree->AddCollisionFilterGroupMember(group_name, link_name,
                                        model_instance_id);
  }

  for (XMLElement* ignore_node =
           node->FirstChildElement("ignored_collision_filter_group");
       ignore_node; ignore_node = ignore_node->NextSiblingElement(
                        "ignored_collision_filter_group")) {
    const char* target_name = ignore_node->Attribute("collision_filter_group");
    if (!target_name) {
      throw runtime_error(
          string(__FILE__) + ": " + __func__ + ": Collision filter group "
          "provides a tag specifying a group to ignore without specifying the "
          "\"collision_filter_group\" attribute.");
    }
    tree->AddCollisionFilterIgnoreTarget(group_name, target_name);
  }
}

/**
 * Parses a joint URDF specification to obtain the names of the joint, parent
 * link, child link, and the joint type. An exception is thrown if any of these
 * names cannot be determined.
 *
 * @param[in] node The XML node parsing the URDF joint description.
 * @param[out] name A reference to a string where the name of the joint
 * should be saved.
 * @param[out] type A reference to a string where the joint type should be
 * saved.
 * @param[out] parent_link_name A reference to a string where the name of the
 * parent link should be saved.
 * @param[out] child_link_name A reference to a string where the name of the
 * child link should be saved.
 */
void ParseJointKeyParams(XMLElement* node,
                         // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                         string& name,
                         // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                         string& type,
                         // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                         string& parent_link_name,
                         // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                         string& child_link_name) {
  // Obtains the joint's name.
  const char* attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: joint "
        "tag is missing name attribute");
  }
  name = string(attr);

  // Obtains the joint's type.
  attr = node->Attribute("type");
  if (!attr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        name + " is missing type attribute");
  }
  type = string(attr);

  // Obtains the name of the joint's parent link.
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        name + " doesn't have a parent node!");
  }
  attr = parent_node->Attribute("link");
  if (!attr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        name + "'s parent does not have a link attribute!");
  }
  parent_link_name = string(attr);

  // Obtains the name of the joint's child link.
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        name + " doesn't have a child node");
  }
  attr = child_node->Attribute("link");
  if (!attr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        name + "'s child does not have a link attribute");
  }
  child_link_name = string(attr);
}

void ParseJoint(RigidBodyTree<double>* tree, XMLElement* node,
                int model_instance_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return;

  // Parses the parent and child link names.
  string name, type, parent_name, child_name;
  ParseJointKeyParams(node, name, type, parent_name, child_name);

  // Checks if this joint connects to the world and, if so, terminates this
  // method call. This is because joints that connect to the world are processed
  // separately.
  if (parent_name == string(RigidBodyTreeConstants::kWorldName)) return;

  int parent_index = tree->FindBodyIndex(parent_name, model_instance_id);
  if (parent_index < 0) {
    throw runtime_error(
        string(__FILE__) + ": " + __func__ + ": ERROR: Could not find "
        "parent link named \"" + parent_name + "\" with model instance ID " +
        std::to_string(model_instance_id) + ".");
  }

  int child_index = tree->FindBodyIndex(child_name, model_instance_id);
  if (child_index < 0) {
    throw runtime_error(
        string(__FILE__) + ": " + __func__ + ": ERROR: Could not find "
        "child link named \"" + child_name + "\" with model instance ID " +
        std::to_string(model_instance_id) + ".");
  }

  Isometry3d transform_to_parent_body = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    originAttributesToTransform(origin, transform_to_parent_body);
  }

  Vector3d axis(1, 0, 0);
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && type.compare("fixed") != 0 &&
      type.compare("floating") != 0 && type.compare("ball") != 0) {
    parseVectorAttribute(axis_node, "xyz", axis);
    if (axis.norm() < 1e-8) {
      throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: axis "
          "is zero.  don't do that");
    }
    axis.normalize();
  }

  // now construct the actual joint (based on its type)
  DrakeJoint* joint = nullptr;

  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    FixedAxisOneDoFJoint<RevoluteJoint>* fjoint =
        new RevoluteJoint(name, transform_to_parent_body, axis);
    SetDynamics(node, fjoint);
    SetLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("fixed") == 0) {
    joint = new FixedJoint(name, transform_to_parent_body);
  } else if (type.compare("prismatic") == 0) {
    FixedAxisOneDoFJoint<PrismaticJoint>* fjoint =
        new PrismaticJoint(name, transform_to_parent_body, axis);
    SetDynamics(node, fjoint);
    SetLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("floating") == 0) {
    joint = new RollPitchYawFloatingJoint(name, transform_to_parent_body);
  } else if (type.compare("ball") == 0) {
    joint = new QuaternionBallJoint(name, transform_to_parent_body);
    cerr << string(__FILE__) + ": " + __func__ + ": Warning: ball joint "
      "is not an official part of the URDF standard." << endl;
  } else {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Unrecognized joint type: " + type);
  }

  unique_ptr<DrakeJoint> joint_unique_ptr(joint);
  tree->bodies[child_index]->setJoint(move(joint_unique_ptr));
  tree->bodies[child_index]->set_parent(tree->bodies[parent_index].get());
}

/* Searches through the URDF document looking for the effort limits of a joint
 * named @p joint_name. If the joint is not found, throws an
 * `std::runtime_error` exception. If the limits of the joint are not specified,
 * this method does nothing. If the effort limits are specified, this method
 * saves them in \p min_effort and \p max_effort.
 *
 * @param[in] robot_node The XML node for the robot description. This must
 * contain child joint elements through which to search.
 *
 * @param[in] joint_name The name of the joint to find. If no joint with such a
 * name exists, throw an exception.
 *
 * @param[out] min_effort A pointer to where the minimum effort should be saved.
 * If the effort limits are not specified, this value is not modified.
 *
 * @param[out] max_effort A pointer to where the minimum effort should be saved.
 * If the effort limits are not specified, this value is not modified.
 *
 * @throws std::runtime_error if the name of the actuator's joint cannot be
 * determined or if the named joint could not be found.
 */
void GetActuatorEffortLimit(XMLElement* robot_node, const string& joint_name,
                            double* min_effort, double* max_effort) {
  for (XMLElement* joint_node = robot_node->FirstChildElement("joint");
       joint_node; joint_node = joint_node->NextSiblingElement("joint")) {
    // Checks for the existence of an attribute named "drake_ignore". If such an
    // attribute exists and has a value of "true", ignores the current joint.
    {
      const char* attr = joint_node->Attribute("drake_ignore");
      if (attr && (std::strcmp(attr, "true") == 0)) {
        continue;
      }
    }

    // Obtains the joint's name.
    const char* attr = joint_node->Attribute("name");
    if (!attr) {
      throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
          "Joint tag is missing name attribute.");
    }
    string name = string(attr);

    // Checks if the current joint's name matches parameter joint_name.
    if (name == joint_name) {
      // Parses the minimum and maximum effort limits.
      XMLElement* limit_node = joint_node->FirstChildElement("limit");
      if (limit_node) {
        // Parses attribute "effort" if it exists.
        parseScalarAttribute(limit_node, "effort", *max_effort);
        (*min_effort) = -(*max_effort);

        // Attribute effort_min and effort_max take precedence over attribute
        // effort if they exist.
        parseScalarAttribute(limit_node, "effort_min", *min_effort);
        parseScalarAttribute(limit_node, "effort_max", *max_effort);
      }

      // Terminates this method call since the joint was found.
      return;
    }
  }

  // If this point in the code is reached, that means no joint named
  // joint_name was found in the URDF. Therefore throw an exception.
  throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
      "Unable to find joint \"" + joint_name + "\".");
}

void ParseTransmission(RigidBodyTree<double>* tree,
                       XMLElement* transmission_node, int model_instance_id) {
  // Determines the transmission type.
  const char* attr = nullptr;
  XMLElement* type_node = transmission_node->FirstChildElement("type");
  if (type_node) {
    attr = type_node->GetText();
  }

  if (!attr) {
    // Old URDF format, kept for convenience
    attr = transmission_node->Attribute("type");
    if (!attr) {
      throw std::logic_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
          "Transmission element is missing the type child.");
    }
  }
  string type(attr);

  // Checks if the transmission type is not SimpleTransmission. If it is not,
  // print a warning and then abort this method call since only simple
  // transmissions are supported at this time.
  if (type.find("SimpleTransmission") == string::npos) {
    cerr << string(__FILE__) + ": " + __func__ + ": WARNING: Only "
        "SimpleTransmissions are supported right now.  This element will be "
        "skipped." << endl;
    return;
  }

  // Determines the actuator's name.
  XMLElement* actuator_node = transmission_node->FirstChildElement("actuator");
  if (!actuator_node || !actuator_node->Attribute("name")) {
    throw std::logic_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Transmission is missing an actuator element.");
  }
  string actuator_name(actuator_node->Attribute("name"));

  // Determines the name of the joint to which the actuator is attached.
  XMLElement* joint_node = transmission_node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name")) {
    throw std::logic_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Transmission is missing a joint element.");
  }
  string joint_name(joint_node->Attribute("name"));

  // Checks if the actuator is attached to a fixed joint. If so, abort this
  // method call.
  int body_index =
      tree->FindIndexOfChildBodyOfJoint(joint_name, model_instance_id);

  if (tree->bodies[body_index]->getJoint().get_num_positions() == 0) {
    cerr << string(__FILE__) + ": " + __func__ + ": WARNING: Skipping "
         << "transmission since it's attached to a fixed joint \""
         << joint_name << "\"." << endl;
    return;
  }

  // Obtains the actuator's gain.
  XMLElement* reduction_node =
      transmission_node->FirstChildElement("mechanicalReduction");
  double gain = 1.0;
  if (reduction_node) parseScalarValue(reduction_node, gain);

  // Obtains the min and max effort from the joint.
  double effort_max = numeric_limits<double>::infinity();
  double effort_min = -numeric_limits<double>::infinity();

  if (XMLElement* element =
          dynamic_cast<XMLElement*>(transmission_node->Parent())) {
    GetActuatorEffortLimit(element, joint_name, &effort_min, &effort_max);
  } else {
    throw std::logic_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Expected a <robot> element as a parent of a <transmission> element "
        "for actuator \"" + actuator_name + "\" and joint \"" + joint_name +
        "\".");
  }

  // Creates the actuator and adds it to the rigid body tree.
  tree->actuators.push_back(RigidBodyActuator(actuator_name,
                                              tree->bodies[body_index].get(),
                                              gain, effort_min, effort_max));
}

void ParseLoop(RigidBodyTree<double>* tree, XMLElement* node,
               int model_instance_id) {
  Vector3d axis(1.0, 0.0, 0.0);

  if (!node || !node->Attribute("name")) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: loop is "
        "missing a name element");
  }
  string name(node->Attribute("name"));

  XMLElement* link_node = node->FirstChildElement("link1");
  std::shared_ptr<RigidBodyFrame<double>> frameA =
      MakeRigidBodyFrameFromUrdfNode(
          *tree, *link_node, link_node, name + "FrameA", model_instance_id);

  link_node = node->FirstChildElement("link2");
  std::shared_ptr<RigidBodyFrame<double>> frameB =
      MakeRigidBodyFrameFromUrdfNode(
          *tree, *link_node, link_node, name + "FrameB", model_instance_id);

  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && !parseVectorAttribute(axis_node, "xyz", axis)) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR parsing "
        "loop joint axis.");
  }

  tree->addFrame(frameA);
  tree->addFrame(frameB);
  RigidBodyLoop<double> l(frameA, frameB, axis);
  tree->loops.push_back(l);
}

void ParseFrame(RigidBodyTree<double>* tree, XMLElement* node,
                int model_instance_id) {
  const char* frame_name = node->Attribute("name");
  if (!frame_name) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR parsing "
        "Drake frame name.");
  }

  std::shared_ptr<RigidBodyFrame<double>> frame =
      MakeRigidBodyFrameFromUrdfNode(
          *tree, *node, node, frame_name, model_instance_id);
  tree->addFrame(frame);
}

/* Searches for a joint that connects the URDF model to a body called
 * RigidBodyTree::kWorldName. If it finds such a joint, it updates
 * @p weld_to_frame with the offset specified by the joint.
 *
 * An exception is thrown if no such joint is found, or if multiple
 * world-connecting joints are found.
 *
 * Multiple world-connecting joints cannot exist in a single URDF file because
 * each URDF file describes one model using a tree of bodies connected by
 * joints. Thus, the only way for a URDF to contain multiple world-connecting
 * joints is if the URDF describes more than one model. This is a violation of
 * the one-model-per-URDF rule.
 *
 * @param[in] node A pointer to the XML node that is parsing the URDF model.
 *
 * @param[out] floating_base_type A reference to where the floating_base_type
 * should be saved.
 *
 * @param[out] weld_to_frame The parameter to modify. If this parameter is
 * `nullptr`, a new `RigidBodyFrame` is constructed and stored in the shared
 * pointer.
 */
void ParseWorldJoint(XMLElement* node,
                     // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                     FloatingBaseType& floating_base_type,
                     // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                     std::shared_ptr<RigidBodyFrame<double>>& weld_to_frame) {
  bool found_world_joint = false;

  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    const char* attr = joint_node->Attribute("drake_ignore");
    if (attr && (std::strcmp(attr, "true") == 0)) continue;

    // Parses the names of the joint, joint type, parent link, and child link.
    string joint_name, joint_type, parent_name, child_name;
    ParseJointKeyParams(joint_node, joint_name, joint_type, parent_name,
                        child_name);

    if (parent_name == string(RigidBodyTreeConstants::kWorldName)) {
      // Ensures only one joint connects the model to the world.
      if (found_world_joint) {
        throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
            "Model contains multiple joints that connect to world!");
      }
      found_world_joint = true;

      // The world-connecting joint was found. The following code updates the
      // weld_to_frame parameter based on the joint's offset, and the
      // floating_base_type parameter based on the joint's type.
      Isometry3d transform_to_parent_body = Isometry3d::Identity();
      XMLElement* origin = joint_node->FirstChildElement("origin");
      if (origin) {
        originAttributesToTransform(origin, transform_to_parent_body);
      }

      // Creates a new rigid body frame if the weld_to_frame parameter contains
      // a nullptr.
      if (weld_to_frame == nullptr)
        weld_to_frame.reset(new RigidBodyFrame<double>());

      weld_to_frame->set_name(string(RigidBodyTreeConstants::kWorldName));
      weld_to_frame->set_transform_to_body(
          weld_to_frame->get_transform_to_body() * transform_to_parent_body);

      if (joint_type == "fixed") {
        floating_base_type = FloatingBaseType::kFixed;
      } else if (joint_type == "continuous") {
        floating_base_type = FloatingBaseType::kQuaternion;
      }

      // Throws an exception if the joint connecting the model to the world
      // includes an axis specification. This is a very strange situation that
      // may not be physically possible in the real world.
      if (node->FirstChildElement("axis")) {
        throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
            "Drake's URDF parser does not support an axis specification for "
            "the joint that connects the model to the world.");
      }
    }
  }
}

ModelInstanceIdTable ParseModel(RigidBodyTree<double>* tree, XMLElement* node,
                                const PackageMap& package_map,
                                const string& root_dir,
                                const FloatingBaseType floating_base_type,
                                std::shared_ptr<RigidBodyFrame<double>>
                                    weld_to_frame) {
  if (!node->Attribute("name")) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: Your "
        "robot must have a name attribute.");
  }

  // Obtains the model name and, if model_instance_table exists, ensures no such
  // model exists in the model_instance_id_table. Throws an exception if a model
  // of the same name already exists in the table.
  string model_name = node->Attribute("name");

  // Instantiates a ModelInstanceIdTable.
  ModelInstanceIdTable model_instance_id_table;

  // Obtains and adds a new model instance ID into the table.
  int model_instance_id = tree->add_model_instance();
  model_instance_id_table[model_name] = model_instance_id;

  // Parses the model's material elements. Throws an exception if there's a
  // material name clash regardless of whether the associated RGBA values are
  // the same.
  MaterialMap materials;
  for (XMLElement* material_node = node->FirstChildElement("material");
       material_node;
       material_node = material_node->NextSiblingElement("material")) {
    ParseMaterial(material_node, materials);
  }

  // Makes a copy of parameter floating_base_type. This is necessary since the
  // actual type may be specified by the URDF itself when the URDF contains a
  // world link and a joint connecting to the world link. By default,
  // actual_floating_base_type equals parameter floating_base_type.
  FloatingBaseType actual_floating_base_type = floating_base_type;

  // Maintains a list of links that were added to the rigid body tree.
  // This is iterated over by AddFloatingJoint() to determine where to attach
  // floating joints.
  std::vector<int> link_indices;

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link")) {
    int index;
    if (ParseBody(tree, model_name, link_node, &materials, package_map,
                  root_dir, model_instance_id, &index)) {
      link_indices.push_back(index);
    } else {
      // Determines whether the link was not parsed because it is a world link.
      const char* name_attr = link_node->Attribute("name");
      if (!name_attr) {
        throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
          "link tag is missing name attribute");
      }

      if (string(name_attr) == string(RigidBodyTreeConstants::kWorldName)) {
        // Since a world link was specified within the URDF, there must be
        // a  joint that connects the world to the robot's root node. The
        // following method parses the information contained within this
        // joint. The weld_to_frame transform, if not a nullptr, is updated
        // with the transform specified by this joint. If it is a nullptr,
        // a shared_ptr<RigidBodyFrame> is created and initialized to this
        // transform.
        ParseWorldJoint(node, actual_floating_base_type, weld_to_frame);
      }
    }
  }

  // DEBUG
  // else {
  // cout << "Parsed link" << endl;
  // cout << "model->bodies.size() = " << model->bodies.size() << endl;
  // cout << "model->num_bodies = " << model->num_bodies << endl;
  //}
  // END_DEBUG

  // Parses the collision filter groups.
  for (XMLElement* group_node =
           node->FirstChildElement("collision_filter_group");
       group_node;
       group_node = group_node->NextSiblingElement("collision_filter_group")) {
    ParseCollisionFilterGroup(tree, group_node, model_instance_id);
  }

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint"))
    ParseJoint(tree, joint_node, model_instance_id);

  // Parses the model's transmission elements.
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission"))
    ParseTransmission(tree, transmission_node, model_instance_id);

  // Parses the model's loop joint elements.
  for (XMLElement* loop_node = node->FirstChildElement("loop_joint"); loop_node;
       loop_node = loop_node->NextSiblingElement("loop_joint"))
    ParseLoop(tree, loop_node, model_instance_id);

  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame"))
    ParseFrame(tree, frame_node, model_instance_id);

  // Adds the floating joint(s) that connect the newly added robot model to the
  // rest of the rigid body tree.
  AddFloatingJoint(actual_floating_base_type, link_indices, weld_to_frame,
                   nullptr /* pose_map */, tree);

  return model_instance_id_table;
}

ModelInstanceIdTable ParseUrdf(XMLDocument* xml_doc,
    const PackageMap& package_map, const string& root_dir,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "URDF does not contain a robot tag.");
  }

  ModelInstanceIdTable model_instance_id_table = ParseModel(
      tree, node, package_map, root_dir, floating_base_type, weld_to_frame);

  tree->compile();

  return model_instance_id_table;
}

}  // namespace

ModelInstanceIdTable AddModelInstanceFromUrdfStringWithRpyJointToWorld(
    const string& urdf_string, RigidBodyTree<double>* tree) {
  const PackageMap package_map;
  return
      AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages(
          urdf_string, package_map, tree);
}

// TODO(liang.fok) Remove this deprecated method prior to Release 1.0.
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const string& urdf_string, RigidBodyTree<double>* tree) {
  return AddModelInstanceFromUrdfStringWithRpyJointToWorld(urdf_string, tree);
}

ModelInstanceIdTable
AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages(
    const string& urdf_string, const PackageMap& package_map,
    RigidBodyTree<double>* tree) {
  const string root_dir = ".";
  return AddModelInstanceFromUrdfStringSearchingInRosPackages(
      urdf_string, package_map, root_dir, kRollPitchYaw,
      nullptr /* weld_to_frame */, tree);
}

// TODO(liang.fok) Remove this deprecated method prior to Release 1.0.
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const string& urdf_string, const PackageMap& package_map,
    RigidBodyTree<double>* tree) {
  return
      AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages(
          urdf_string, package_map, tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const string& urdf_string,
    const PackageMap& package_map,
    const string& root_dir,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  return AddModelInstanceFromUrdfStringSearchingInRosPackages(
      urdf_string, package_map, root_dir, floating_base_type,
      weld_to_frame, tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const string& urdf_string, const string& root_dir,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  const PackageMap package_map;
  return AddModelInstanceFromUrdfStringSearchingInRosPackages(
      urdf_string, package_map, root_dir, floating_base_type,
      weld_to_frame, tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfStringSearchingInRosPackages(
    const string& urdf_string, const PackageMap& package_map,
    const string& root_dir, const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  XMLDocument xml_doc;
  xml_doc.Parse(urdf_string.c_str());
  return ParseUrdf(&xml_doc, package_map, root_dir, floating_base_type,
                   weld_to_frame, tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfFileWithRpyJointToWorld(
    const string& filename, RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree && "You must provide a valid RigidBodyTree pointer.");
  const string full_path_filename = GetFullPath(filename);
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_path_filename);
  return AddModelInstanceFromUrdfFileSearchingInRosPackages(
      full_path_filename, package_map, kRollPitchYaw,
      nullptr /* weld_to_frame */, tree);
}

// TODO(liang.fok) Remove this deprecated method prior to Release 1.0.
ModelInstanceIdTable AddModelInstanceFromUrdfFile(const string& filename,
                                                  RigidBodyTree<double>* tree) {
  return AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename, tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfFileToWorld(
    const string& filename, const FloatingBaseType floating_base_type,
    RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree && "You must provide a valid RigidBodyTree pointer.");
  const string full_path_filename = GetFullPath(filename);
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_path_filename);
  return AddModelInstanceFromUrdfFileSearchingInRosPackages(
      full_path_filename, package_map, floating_base_type,
      nullptr /*weld_to_frame*/, tree);
}

// TODO(liang.fok) Remove this deprecated method prior to Release 1.0.
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const string& filename, const FloatingBaseType floating_base_type,
    RigidBodyTree<double>* tree) {
  return AddModelInstanceFromUrdfFileToWorld(filename, floating_base_type,
                                             tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const string& filename, const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree && "You must provide a valid RigidBodyTree pointer.");
  const string full_path_filename = GetFullPath(filename);
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_path_filename);
  return AddModelInstanceFromUrdfFileSearchingInRosPackages(
      full_path_filename, package_map, floating_base_type, weld_to_frame, tree);
}

ModelInstanceIdTable AddModelInstanceFromUrdfFileSearchingInRosPackages(
    const string& filename, const PackageMap& package_map,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree && "You must provide a valid RigidBodyTree pointer.");

  // Opens the URDF file and feeds it into the XML parser.
  XMLDocument xml_doc;
  xml_doc.LoadFile(filename.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error(string(__FILE__) + ": " + __func__ + ": Failed to "
        "parse XML in file " + filename + "\n" + xml_doc.ErrorName());
  }

  // Uses the directory holding the URDF to be the root directory
  // in which to search for files referenced within the URDF file.
  string root_dir = ".";
  size_t found = filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = filename.substr(0, found);
  }

  return ParseUrdf(&xml_doc, package_map, root_dir, floating_base_type,
                   weld_to_frame, tree);
}

// TODO(liang.fok) Remove this deprecated method prior to Release 1.0.
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const string& filename, const PackageMap& package_map,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree) {
  return AddModelInstanceFromUrdfFileSearchingInRosPackages(
      filename, package_map, floating_base_type, weld_to_frame, tree);
}

std::shared_ptr<RigidBodyFrame<double>> MakeRigidBodyFrameFromUrdfNode(
    const RigidBodyTree<double>& tree, const tinyxml2::XMLElement& link,
    const tinyxml2::XMLElement* pose, const string& name,
    int model_instance_id) {
  string body_name = link.Attribute("link");
  RigidBody<double>* body =
      tree.FindBody(body_name, "" /* model_name */, model_instance_id);
  if (body == nullptr) {
    throw runtime_error(string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Couldn't find body \"" + body_name + "\" referenced in frame \"" +
        name + "\".");
  }

  Vector3d xyz = Vector3d::Zero();
  Vector3d rpy = Vector3d::Zero();
  if (pose) {
    parseVectorAttribute(pose, "xyz", xyz);
    parseVectorAttribute(pose, "rpy", rpy);
  }
  return allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), name, body, xyz, rpy);
}

}  // namespace urdf
}  // namespace parsers
}  // namespace drake
