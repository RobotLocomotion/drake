#include <fstream>
#include <sstream>
#include <string>

#include "drake/systems/plants/joints/DrakeJoints.h"
#include "drake/systems/plants/material_map.h"
#include "drake/systems/plants/rigid_body_tree_urdf.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/util/eigen_matrix_compare.h"

using Eigen::Isometry3d;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

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

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

// todo: rectify this with FindBodyIndex in the class (which makes more
// assumptions)
int findLinkIndex(RigidBodyTree* model, string link_name) {
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (link_name.compare(model->bodies[i]->name_) == 0) {
      index = i;
      break;
    }
  }
  return index;
}

// Finds the index of the link whose parent joint has a specified name.
// Throws a std::runtime_error if no such link can be found or if more than
// one link is found.
//
// TODO(liang.fok): Generalize this method to support a model_id.
//                  See: https://github.com/RobotLocomotion/drake/issues/2583
int FindBodyIndexByJointName(RigidBodyTree* model, string joint_name) {
  // Instantiates a local variable that stores the index of the rigid body whose
  // joint is the one we're searching for. It is initialized to an invalid index
  // so the failure mode of not finding any matching joint can be identified.
  // Valid index values are between zero and the number of rigid bodies in
  // the rigid body tree.
  int index = -1;

  // Searches through all of the bodies in the rigid body tree looking for the
  // joint with the specified name.
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (model->bodies[i]->hasParent() &&
        joint_name.compare(model->bodies[i]->getJoint().getName()) == 0) {
      if (index == -1) {
        index = i;
      } else {
        throw std::runtime_error(
            "RigidBodyTreeURDF.cpp: FindBodyIndexByJointName: ERROR: Multiple "
            "joints named \"" +
            joint_name + "\" found.");
      }
    }
  }

  // Verifies that the link was found. If not, throws an exception.
  if (index == -1) {
    throw std::runtime_error(
        "RigidBodyTreeURDF.cpp: FindBodyIndexByJointName: "
        "ERROR: Unable to find joint named \"" +
        joint_name + "\".");
  }

  return index;
}

void parseInertial(RigidBody* body, XMLElement* node, RigidBodyTree* model) {
  Isometry3d T = Isometry3d::Identity();

  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T);

  XMLElement* mass = node->FirstChildElement("mass");
  if (mass) parseScalarAttribute(mass, "value", body->mass);

  body->com << T(0, 3), T(1, 3), T(2, 3);

  Matrix<double, TWIST_SIZE, TWIST_SIZE> I =
      Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
  I.block(3, 3, 3, 3) << body->mass * Matrix3d::Identity();

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

  body->I = transformSpatialInertia(T, I);
}

// Adds a material to the supplied material map. If the material is already
// present, it checks whether the new values are the same as the old values. If
// they are the same, return normally. Otherwise print a warning to std::cerr.
//
// Currently, only simple colors are supported as the material.
//
// TODO(liang.fok) Add support for texture-based materials. See:
// https://github.com/RobotLocomotion/drake/issues/2588
//
// @param[in] material_name A human-understandable name of the material.
// @param[in] color_rgba The red-green-blue-alpha color values of the material.
// The range of values is [0, 1].
// @param[out] materials A pointer to the map in which to store the material.
// If this pointer is a null, a `std::logic_error` is thrown.
void AddMaterialToMaterialMap(const std::string& material_name,
                              const Vector4d& color_rgba,
                              MaterialMap* materials) {
  // Throws an exception if parameter materials is null.
  if (materials == nullptr) {
    throw std::logic_error(
        "RigidBodyTreeURDF.cpp: AddMaterialToMaterialMap: ERROR: materials is "
        "null, material_name = " +
        material_name + ".");
  }

  // Determines if the material is already in the map.
  auto material_iter = materials->find(material_name);
  if (material_iter != materials->end()) {
    // The material is already in the map. Checks whether the old material is
    // the same as the new material. Note that since the range of values in the
    // RGBA vectors is [0, 1], absolute and relative tolerance comparisons are
    // identical.
    auto& existing_color = material_iter->second;
    if (!drake::util::CompareMatrices(
            color_rgba, existing_color, 1e-10,
            drake::util::MatrixCompareType::absolute)) {
      // The materials map already has the material_name key but the color
      // associated with it is different.
      std::stringstream error_buff;
      error_buff << "RigidBodyTreeURDF.cpp: AddMaterialToMaterialMap(): "
                 << "Error: Material \"" + material_name + "\" was previously "
                 << "defined but was associated with different RGBA color "
                 << "values." << std::endl
                 << "  - existing RGBA values: " << existing_color.transpose()
                 << std::endl
                 << "  - new RGBA values: " << color_rgba.transpose()
                 << std::endl
                 << "Keeping the original RGBA values in the materials map."
                 << std::endl;
      throw std::runtime_error(error_buff.str());
    }
  } else {
    // Adds the new color to the materials map.
    (*materials)[material_name] = color_rgba;
  }
}

void ParseMaterial(XMLElement* node, MaterialMap& materials) {
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0) {
    throw std::runtime_error(
        "RigidBodyTreeURDF.cpp: ParseMaterial(): ERROR: "
        "Material tag is missing a name.");
  }
  string name(attr);

  Vector4d rgba = Vector4d::Zero();  // Defaults to black.

  XMLElement* color_node = node->FirstChildElement("color");

  if (color_node) {
    if (!parseVectorAttribute(color_node, "rgba", rgba)) {
      throw std::runtime_error(
          "RigidBodyTreeURDF.cpp: ParseMaterial(): ERROR: "
          "Color tag is missing rgba attribute.");
    }
    AddMaterialToMaterialMap(name, rgba, &materials);
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
            << "RigidBodyTreeURDF.cpp: ParseMaterial():  WARNING: Material \""
            << name << "\" is a texture. Textures are currently not supported. "
            << "For more information, see: "
            << "https://github.com/RobotLocomotion/drake/issues/2588. "
               "Defaulting to use the black color for this material."
            << endl;
        AddMaterialToMaterialMap(name, rgba, &materials);
      } else {
        throw std::runtime_error(
            "RigidBodyTreeURDF.cpp: ParseMaterial: ERROR: Material\"" + name +
            "\" not previously defined. Therefore a color must be specified.");
      }

      return;
    }
  }
}

bool parseGeometry(XMLElement* node, const PackageMap& package_map,
                   const string& root_dir, DrakeShapes::Element& element) {
  // DEBUG
  // cout << "parseGeometry: START" << endl;
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
      cerr << "ERROR parsing box element size" << endl;
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
      cerr << "ERROR parsing sphere element radius" << endl;
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
      cerr << "ERROR parsing cylinder element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << "ERROR parsing cylinder element length" << endl;
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
      cerr << "ERROR parsing capsule element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << "ERROR: Failed to parse capsule element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    attr = shape_node->Attribute("filename");
    if (!attr) {
      cerr << "ERROR mesh element has no filename tag" << endl;
      return false;
    }
    string filename(attr);
    string resolved_filename = resolveFilename(filename, package_map, root_dir);
    DrakeShapes::Mesh mesh(filename, resolved_filename);

    // Obtains the scale of the mesh if it exists.
    if (shape_node->Attribute("scale") != nullptr)
      ParseThreeVectorAttribute(shape_node, "scale", &mesh.scale);

    element.setGeometry(mesh);
  } else {
    cerr << "Warning: geometry element has an unknown type and will be ignored."
         << endl;
  }
  // DEBUG
  // cout << "parseGeometry: END" << endl;
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
void parseVisual(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                 MaterialMap* materials, const PackageMap& package_map,
                 const string& root_dir) {
  // Ensures there is a geometry child element. Since this is a required
  // element, throws an exception if a geometry element does not exist.
  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw runtime_error("ERROR: Link " + body->name_ +
                        " has a visual element without geometry.");
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
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse visual element in link " +
                        body->name_ + ".");

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
        Vector4d rgba;
        if (!parseVectorAttribute(color_node, "rgba", rgba)) {
          throw runtime_error(
              "ERROR: Failed to parse color of material for "
              "model \"" +
              body->model_name() + "\", link \"" + body->name() + "\".");
        }
        color_specified = true;
      }
    }

    // Checks and remembers whether a "name" attribute exists. If so, parses the
    // name value.
    std::string material_name;
    bool name_specified = false;
    {
      const char* attr = material_node->Attribute("name");

      if (attr != nullptr && strlen(attr) != 0) {
        material_name = std::string(attr);
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
    if (color_specified && name_specified)
      AddMaterialToMaterialMap(material_name, rgba, materials);

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
      std::stringstream error_buff;
      error_buff
          << "RigidBodyTreeURDF.cpp: parseVisual(): "
          << "WARNING: Visual element has a material whose color could not"
             "be determined."
          << std::endl
          << "  - model name: " << body->model_name() << std::endl
          << "  - body name: " << body->name() << std::endl
          << "  - material name: " << material_name << std::endl;
      throw std::runtime_error(error_buff.str());
    }
  }

  if (element.hasGeometry()) body->addVisualElement(element);
}

void parseCollision(RigidBody* body, XMLElement* node, RigidBodyTree* model,
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
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->name_ +
                        " has a collision element without geometry");

  RigidBody::CollisionElement element(T_element_to_link, body);
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse collision element in link " +
                        body->name_ + ".");

  if (element.hasGeometry()) {
    model->addCollisionElement(element, *body, group_name);
  }
}

bool parseLink(RigidBodyTree* model, std::string robot_name, XMLElement* node,
               MaterialMap* materials, const PackageMap& package_map,
               const string& root_dir, int* index) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return false;

  RigidBody* body{nullptr};
  std::unique_ptr<RigidBody> owned_body(body = new RigidBody());
  body->model_name_ = robot_name;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");

  // World links are handled by parseWorldJoint().
  body->name_ = attr;
  if (body->name_ == std::string(RigidBodyTree::kWorldLinkName)) return false;

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) parseInertial(body, inertial_node, model);

  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    parseVisual(body, visual_node, model, materials, package_map, root_dir);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    parseCollision(body, collision_node, model, package_map, root_dir);
  }

  model->add_rigid_body(std::move(owned_body));
  *index = body->body_index;
  return true;
}

template <typename JointType>
void setLimits(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
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
void setDynamics(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (fjoint != nullptr && dynamics_node) {
    double damping = 0.0, coulomb_friction = 0.0, coulomb_window = 0.0;
    parseScalarAttribute(dynamics_node, "damping", damping);
    parseScalarAttribute(dynamics_node, "friction", coulomb_friction);
    parseScalarAttribute(dynamics_node, "coulomb_window", coulomb_window);
    fjoint->setDynamics(damping, coulomb_friction, coulomb_window);
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
void parseJointKeyParams(XMLElement* node, std::string& name, std::string& type,
                         std::string& parent_link_name,
                         std::string& child_link_name) {
  // Obtains the joint's name.
  const char* attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: joint tag is missing name attribute");
  name = std::string(attr);

  // Obtains the joint's type.
  attr = node->Attribute("type");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " is missing type "
                        "attribute");
  type = std::string(attr);

  // Obtains the name of the joint's parent link.
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node)
    throw runtime_error("ERROR: joint " + name +
                        " doesn't have a parent node!");
  attr = parent_node->Attribute("link");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        "'s parent does not have a link attribute!");
  parent_link_name = std::string(attr);

  // Obtains the name of the joint's child link.
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node)
    throw runtime_error("ERROR: joint " + name + " doesn't have a child node");
  attr = child_node->Attribute("link");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        "'s child does not have a link attribute");
  child_link_name = std::string(attr);
}

void parseJoint(RigidBodyTree* model, XMLElement* node) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return;

  // Parses the parent and child link names.
  std::string name, type, parent_name, child_name;
  parseJointKeyParams(node, name, type, parent_name, child_name);

  // Checks if this joint connects to the world and, if so, terminates this
  // method call. This is because joints that connect to the world are processed
  // separately.
  if (parent_name == std::string(RigidBodyTree::kWorldLinkName)) return;

  int parent_index = findLinkIndex(model, parent_name);
  if (parent_index < 0)
    throw runtime_error("ERROR: could not find parent link named " +
                        parent_name);

  int child_index = findLinkIndex(model, child_name);
  if (child_index < 0)
    throw runtime_error("ERROR: could not find child link named " + child_name);

  Isometry3d transform_to_parent_body = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    originAttributesToTransform(origin, transform_to_parent_body);
  }

  Vector3d axis;
  axis << 1, 0, 0;
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && type.compare("fixed") != 0 &&
      type.compare("floating") != 0) {
    parseVectorAttribute(axis_node, "xyz", axis);
    if (axis.norm() < 1e-8)
      throw runtime_error("ERROR: axis is zero.  don't do that");
    axis.normalize();
  }

  // now construct the actual joint (based on it's type)
  DrakeJoint* joint = nullptr;

  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    FixedAxisOneDoFJoint<RevoluteJoint>* fjoint =
        new RevoluteJoint(name, transform_to_parent_body, axis);
    setDynamics(node, fjoint);
    setLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("fixed") == 0) {
    joint = new FixedJoint(name, transform_to_parent_body);
  } else if (type.compare("prismatic") == 0) {
    FixedAxisOneDoFJoint<PrismaticJoint>* fjoint =
        new PrismaticJoint(name, transform_to_parent_body, axis);
    setDynamics(node, fjoint);
    setLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("floating") == 0) {
    joint = new RollPitchYawFloatingJoint(name, transform_to_parent_body);
  } else {
    throw runtime_error("ERROR: Unrecognized joint type: " + type);
  }

  unique_ptr<DrakeJoint> joint_unique_ptr(joint);
  model->bodies[child_index]->setJoint(move(joint_unique_ptr));
  model->bodies[child_index]->parent = model->bodies[parent_index].get();
}

// Searches through the URDF document looking for the effort limits of a
// particular joint. If the joint is not found, throws an exception. If the
// limits of the joint are not specified, does nothing. If the effort limits are
// specified, saves the effort limits in \p min_effort and \p max_effort.
//
// @param[in] robot_node The XML node for the robot description. This must
// contain child joint elements through which to search.
// @param[in] joint_name The name of the joint to find. If no joint with such a
// name exists, throw an exception.
// @param[out] min_effort A pointer to where the minimum effort should be saved.
// If the effort limits are not specified, this value is not modified.
// @param[out] max_effort A pointer to where the minimum effort should be saved.
// If the effort limits are not specified, this value is not modified.
// @throws std::runtime_error if the name of the actuator's joint cannot be
// determined or if the named joint could not be found.
void GetActuatorEffortLimit(XMLElement* robot_node,
                            const std::string& joint_name, double* min_effort,
                            double* max_effort) {
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
      throw std::runtime_error(
          "RigidBodyTreeURDF.cpp: GetActuatorEffortLimit: ERROR: Joint tag is "
          "missing name attribute.");
    }
    std::string name = std::string(attr);

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
  throw std::runtime_error(
      "GetActuatorEffortLimit: ERROR: Unable to find joint \"" + joint_name +
      "\".");
}

void parseTransmission(RigidBodyTree* model, XMLElement* transmission_node) {
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
      throw std::logic_error(
          "RigidBodyTreeURDF.cpp: parseTransmission: ERROR: Transmission "
          "element is missing the type child.");
    }
  }
  string type(attr);

  // Checks if the transmission type is not SimpleTransmission. If it is not,
  // print a warning and then abort this method call since only simple
  // transmissions are supported at this time.
  if (type.find("SimpleTransmission") == string::npos) {
    cerr << "RigidBodyTreeURDF.cpp: parseTransmission: WARNING: Only "
            "SimpleTransmissions are supported right now.  This element will "
            "be skipped."
         << endl;
    return;
  }

  // Determines the actuator's name.
  XMLElement* actuator_node = transmission_node->FirstChildElement("actuator");
  if (!actuator_node || !actuator_node->Attribute("name")) {
    throw std::logic_error(
        "RigidBodyTreeURDF.cpp: parseTransmission: ERROR: "
        "Transmission is missing an actuator element.");
  }
  string actuator_name(actuator_node->Attribute("name"));

  // Determines the name of the joint to which the actuator is attached.
  XMLElement* joint_node = transmission_node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name")) {
    throw std::logic_error(
        "RigidBodyTreeURDF.cpp: parseTransmission: ERROR: "
        "Transmission is missing a joint element.");
  }
  string joint_name(joint_node->Attribute("name"));

  // Checks if the actuator is attached to a fixed joint. If so, abort this
  // method call.
  int body_index = FindBodyIndexByJointName(model, joint_name);

  if (model->bodies[body_index]->getJoint().getNumPositions() == 0) {
    cerr << "RigidBodyTreeURDF.cpp: parseTransmission: WARNING: Skipping "
            "transmission since it's attached to a fixed joint \""
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
    throw std::logic_error(
        "RigidBodyTreeURDF.cpp: parseTransmission: ERROR: Expected a <robot> "
        "element as a parent of a <transmission> element for actuator \"" +
        actuator_name + "\" and joint \"" + joint_name + "\".");
  }

  // Creates the actuator and adds it to the rigid body tree.
  model->actuators.push_back(RigidBodyActuator(actuator_name,
                                               model->bodies[body_index].get(),
                                               gain, effort_min, effort_max));
}

void parseLoop(RigidBodyTree* model, XMLElement* node) {
  Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  if (!node || !node->Attribute("name"))
    throw runtime_error("ERROR: loop is missing a name element");
  string name(node->Attribute("name"));

  XMLElement* link_node = node->FirstChildElement("link1");
  std::shared_ptr<RigidBodyFrame> frameA =
      drake::systems::MakeRigidBodyFrameFromURDFNode(
          *model, link_node, link_node, name + "FrameA");

  link_node = node->FirstChildElement("link2");
  std::shared_ptr<RigidBodyFrame> frameB =
      drake::systems::MakeRigidBodyFrameFromURDFNode(
          *model, link_node, link_node, name + "FrameB");

  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && !parseVectorAttribute(axis_node, "xyz", axis))
    throw runtime_error("ERROR parsing loop joint axis");

  model->addFrame(frameA);
  model->addFrame(frameB);
  RigidBodyLoop l(frameA, frameB, axis);
  model->loops.push_back(l);
}

void parseFrame(RigidBodyTree* model, XMLElement* node) {
  const char* frame_name = node->Attribute("name");
  if (!frame_name) throw runtime_error("ERROR parsing Drake frame name");

  std::shared_ptr<RigidBodyFrame> frame =
      drake::systems::MakeRigidBodyFrameFromURDFNode(*model, node, node,
                                                     frame_name);
  model->addFrame(frame);
}

/**
 * Searches for a joint that connects the URDF model to a link with a name equal
 * to the string defined by RigidBodyTree::kWorldLinkName. If it finds such a
 * joint, it updates the weld_to_frame parameter with the offset specified by
 * the joint.
 *
 * An exception is thrown if no such joint is found, or if multiple
 * world-connecting joints are found.
 *
 * Multiple world-connecting joints cannot exist in a single URDF file because
 * each URDF file describes one robot using a tree of links connected by joints.
 * Thus, the only way for a URDF to contain multiple world-connecting joints is
 * if the URDF describes more than one robot. This is a violation of the
 * one-robot-per-URDF rule.
 *
 * @param[in] node A pointer to the XML node that is parsing the URDF model.
 * @param[out] floating_base_type A reference to where the floating_base_type
 * should be saved.
 * @param[out] weld_to_frame The parameter to modify. If this parameter is
 * `nullptr`, a new `RigidBodyFrame` is constructed and stored in the shared
 * pointer.
 */
void parseWorldJoint(XMLElement* node,
                     DrakeJoint::FloatingBaseType& floating_base_type,
                     std::shared_ptr<RigidBodyFrame>& weld_to_frame) {
  bool found_world_joint = false;

  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    const char* attr = joint_node->Attribute("drake_ignore");
    if (attr && (std::strcmp(attr, "true") == 0)) continue;

    // Parses the names of the joint, joint type, parent link, and child link.
    std::string joint_name, joint_type, parent_name, child_name;
    parseJointKeyParams(joint_node, joint_name, joint_type, parent_name,
                        child_name);

    if (parent_name == std::string(RigidBodyTree::kWorldLinkName)) {
      // Ensures only one joint connects the model to the world.
      if (found_world_joint)
        throw runtime_error(
            "ERROR: Model contains multiple joints that connect to world!");
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
      if (weld_to_frame == nullptr) weld_to_frame.reset(new RigidBodyFrame());

      weld_to_frame->name = std::string(RigidBodyTree::kWorldLinkName);
      weld_to_frame->transform_to_body = transform_to_parent_body;

      if (joint_type == "fixed") {
        floating_base_type = DrakeJoint::FloatingBaseType::FIXED;
      } else if (joint_type == "continuous") {
        floating_base_type = DrakeJoint::FloatingBaseType::QUATERNION;
      }

      // Throws an exception if the joint connecting the model to the world
      // includes an axis specification. This is a very strange situation that
      // may not be physically possible in the real world.
      if (node->FirstChildElement("axis")) {
        throw runtime_error(
            "ERROR: Drake's URDF parser does not support an axis specification"
            "for the joint that connects the model to the world.");
      }
    }
  }
}

void parseRobot(RigidBodyTree* model, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const DrakeJoint::FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr) {
  if (!node->Attribute("name"))
    throw runtime_error("Error: your robot must have a name attribute");

  string robotname = node->Attribute("name");

  // Parses the model's material elements.
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
  DrakeJoint::FloatingBaseType actual_floating_base_type = floating_base_type;

  // Maintains a list of links that were added to the rigid body tree.
  // This is iterated over by method AddFloatingJoint() to determine where
  // to attach floating joints.
  std::vector<int> link_indices;

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link")) {
    int index;
    if (parseLink(model, robotname, link_node, &materials, package_map,
                  root_dir, &index)) {
      link_indices.push_back(index);
    } else {
      // Determines whether the link was not parsed because it is a world link.
      const char* name_attr = link_node->Attribute("name");
      if (!name_attr)
        throw runtime_error("ERROR: link tag is missing name attribute");

      if (std::string(name_attr) ==
          std::string(RigidBodyTree::kWorldLinkName)) {
        // A world link was specified within the URDF. The following code
        // verifies that parameter weld_to_frame is not specified. It throws an
        // exception if it is since the model being added is connected to the
        // world in two different ways. Otherwise, it extract the information
        // necessary create the virtual joint that connects the robot to the
        // world.
        if (weld_to_frame != nullptr) {
          throw runtime_error(
              "Both weld_to_frame and world link specified. "
              "Only one may be specified when instantiating "
              "a URDF model.");
        } else {
          // Since a world link was specified, there must be a joint that
          // connects the world to the robot's root note. The following
          // method call parses the information contained within this joint.
          parseWorldJoint(node, actual_floating_base_type, weld_to_frame);
        }
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

  // todo: parse collision filter groups

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint"))
    parseJoint(model, joint_node);

  // Parses the model's transmission elements.
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission"))
    parseTransmission(model, transmission_node);

  // Parses the model's loop joint elements.
  for (XMLElement* loop_node = node->FirstChildElement("loop_joint"); loop_node;
       loop_node = loop_node->NextSiblingElement("loop_joint"))
    parseLoop(model, loop_node);

  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame"))
    parseFrame(model, frame_node);

  // Adds the floating joint(s) that connect the newly added robot model to the
  // rest of the rigid body tree.
  model->AddFloatingJoint(actual_floating_base_type, link_indices,
                          weld_to_frame);
}

void parseURDF(RigidBodyTree* model, XMLDocument* xml_doc,
               PackageMap& package_map, const string& root_dir,
               const DrakeJoint::FloatingBaseType floating_base_type,
               std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr) {
  populatePackageMap(package_map);
  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error("ERROR: This urdf does not contain a robot tag");
  }

  parseRobot(model, node, package_map, root_dir, floating_base_type,
             weld_to_frame);

  model->compile();
}

}  // namespace

namespace drake {
namespace systems {

std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromURDFNode(
    const RigidBodyTree& model, const tinyxml2::XMLElement* link,
    const tinyxml2::XMLElement* pose, const std::string& name) {
  std::string body_name = link->Attribute("link");
  RigidBody* body = model.FindBody(body_name);
  if (body == nullptr) {
    throw runtime_error("ERROR: Couldn't find body \"" + body_name + "\""
                        " referenced in frame \"" + name + "\".");
  }

  Vector3d xyz = Vector3d::Zero(), rpy = Vector3d::Zero();
  if (pose) {
    parseVectorAttribute(pose, "xyz", xyz);
    parseVectorAttribute(pose, "rpy", rpy);
  }
  return allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), name, body, xyz, rpy);
}

}  // namespace systems
}  // namespace drake

void RigidBodyTree::addRobotFromURDFString(
    const string& xml_string, const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  PackageMap package_map;
  addRobotFromURDFString(xml_string, package_map, root_dir, floating_base_type,
                         weld_to_frame);
}

void RigidBodyTree::addRobotFromURDFString(
    const string& xml_string, PackageMap& package_map, const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  XMLDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  parseURDF(this, &xml_doc, package_map, root_dir, floating_base_type,
            weld_to_frame);
}

void RigidBodyTree::addRobotFromURDF(
    const string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  PackageMap package_map;
  addRobotFromURDF(urdf_filename, package_map, floating_base_type,
                   weld_to_frame);
}

void RigidBodyTree::addRobotFromURDF(
    const string& urdf_filename, PackageMap& package_map,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  XMLDocument xml_doc;
  xml_doc.LoadFile(urdf_filename.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename +
                             "\n" + xml_doc.ErrorName());
  }

  string root_dir = ".";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  parseURDF(this, &xml_doc, package_map, root_dir, floating_base_type,
            weld_to_frame);
}
