#include <fstream>
#include <sstream>
#include <string>

#include "drake/systems/plants/joints/DrakeJoints.h"
#include "drake/systems/plants/material_map.h"
#include "drake/systems/plants/rigid_body_tree_urdf.h"
#include "drake/systems/plants/xmlUtil.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

namespace {

// todo: rectify this with findLinkId in the class (which makes more
// assumptions)
int findLinkIndex(RigidBodyTree* model, string linkname) {
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (linkname.compare(model->bodies[i]->linkname) == 0) {
      index = i;
      break;
    }
  }
  return index;
}

int findLinkIndexByJointName(RigidBodyTree* model, string jointname) {
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (model->bodies[i]->hasParent() &&
        jointname.compare(model->bodies[i]->getJoint().getName()) == 0) {
      index = i;
      break;
    }
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

bool parseMaterial(XMLElement* node, MaterialMap& materials) {
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0) {
    cerr << "WARNING: material tag is missing a name" << endl;
    return false;
  }
  string name(attr);
  auto material_iter = materials.find(name);
  bool already_in_map = false;
  if (material_iter != materials.end()) {
    already_in_map = true;
  }

  Vector4d rgba;
  XMLElement* color_node = node->FirstChildElement("color");
  if (color_node) {
    if (!parseVectorAttribute(color_node, "rgba", rgba)) {
      cerr << "WARNING: color tag is missing rgba attribute" << endl;
      return false;
    }
    materials[name] = rgba;
  } else if (!already_in_map) {
    cerr << "WARNING: material \"" << name
         << "\" is not a simple color material (so is currently unsupported)"
         << endl;
    return false;
  }
  return true;
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

    attr = shape_node->Attribute("scale");
    if (attr) {
      stringstream s(attr);
      s >> mesh.scale;
    }

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

void parseVisual(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                 const MaterialMap& materials, const PackageMap& package_map,
                 const string& root_dir) {
  // DEBUG
  // cout << "parseVisual: START" << endl;
  // END_DEBUG
  Isometry3d T_element_to_link = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T_element_to_link);

  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a visual element without geometry.");

  DrakeShapes::VisualElement element(T_element_to_link);
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse visual element in link " +
                        body->linkname + ".");

  XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    const char* attr;
    attr = material_node->Attribute("name");
    if (attr && strlen(attr) > 0 && materials.find(attr) != materials.end()) {
      element.setMaterial(materials.at(attr));
    } else {
      XMLElement* color_node = material_node->FirstChildElement("color");
      if (color_node) {
        Vector4d rgba;
        if (!parseVectorAttribute(color_node, "rgba", rgba)) {
          cerr << "WARNING: Failed to parse color element rgba in visual"
               << endl;
        } else {
          element.setMaterial(rgba);
        }
      } else {
        cerr << "WARNING: visual element had a material with neither a name "
                "nor a nested color element"
             << endl;
      }
    }
  }

  if (element.hasGeometry()) {
    // DEBUG
    // cout << "parseVisual: Adding element to body" << endl;
    // END_DEBUG
    body->addVisualElement(element);
  }

  // DEBUG
  // cout << "parseVisual: END" << endl;
  // END_DEBUG
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
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a collision element without geometry");

  RigidBody::CollisionElement element(T_element_to_link, *body);
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse collision element in link " +
                        body->linkname + ".");

  if (element.hasGeometry()) {
    model->addCollisionElement(element, *body, group_name);
  }
}

bool parseLink(RigidBodyTree* model, std::string robot_name, XMLElement* node,
               const MaterialMap& materials, const PackageMap& package_map,
               const string& root_dir, int* index) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return false;

  std::unique_ptr<RigidBody> body(new RigidBody());
  body->model_name = robot_name;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");

  body->linkname = attr;
  if (body->linkname == "world")
    throw runtime_error(
        "ERROR: do not name a link 'world', it is a reserved name");

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) parseInertial(body.get(), inertial_node, model);

  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    parseVisual(body.get(), visual_node, model, materials, package_map, root_dir);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    parseCollision(body.get(), collision_node, model, package_map, root_dir);
  }

  model->add_rigid_body(std::move(body));
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

void parseJoint(RigidBodyTree* model, XMLElement* node) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: joint tag is missing name attribute");
  string name(attr);

  attr = node->Attribute("type");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " is missing the type attribute");
  string type(attr);

  // parse parent
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node)
    throw runtime_error("ERROR: joint " + name + " doesn't have a parent node");

  attr = parent_node->Attribute("link");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " parent does not have a link attribute");
  string parent_name(attr);

  int parent_index = findLinkIndex(model, parent_name);
  if (parent_index < 0)
    throw runtime_error("ERROR: could not find parent link named " +
                        parent_name);

  // parse child
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node)
    throw runtime_error("ERROR: joint " + name + " doesn't have a child node");
  attr = child_node->Attribute("link");
  if (!attr)
    throw runtime_error("ERROR: joint " + name +
                        " child does not have a link attribute");
  string child_name(attr);

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
  model->bodies[child_index]->parent = &model->body(parent_index);
}

void parseTransmission(RigidBodyTree* model, XMLElement* node) {
  const char* attr = nullptr;
  XMLElement* type_node = node->FirstChildElement("type");
  if (type_node) {
    attr = type_node->GetText();
  }

  if (!attr) {
    attr = node->Attribute("type");  // old URDF format, kept for convenience
    if (!attr)
      throw runtime_error(
          "ERROR: transmission element is missing the type child");
  }
  string type(attr);
  if (type.find("SimpleTransmission") == string::npos) {
    cerr << "WARNING: only SimpleTransmissions are supported right now.  this "
            "element will be skipped."
         << endl;
    return;
  }

  XMLElement* actuator_node = node->FirstChildElement("actuator");
  if (!actuator_node || !actuator_node->Attribute("name"))
    throw runtime_error("ERROR: transmission is missing an actuator element");
  string actuator_name(actuator_node->Attribute("name"));

  XMLElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name"))
    throw runtime_error("ERROR: transmission is missing a joint element");
  string joint_name(joint_node->Attribute("name"));

  int body_index = findLinkIndexByJointName(model, joint_name);

  if (model->bodies[body_index]->getJoint().getNumPositions() == 0) {
    cerr << "WARNING: Skipping transmission since it's attached to a fixed "
            "joint: "
         << joint_name << endl;
    return;
  }

  XMLElement* reduction_node = node->FirstChildElement("mechanicalReduction");
  double gain = 1.0;
  if (reduction_node) parseScalarValue(reduction_node, gain);

  XMLElement* limit_node = joint_node->FirstChildElement("limit");
  double effort_max = numeric_limits<double>::infinity();
  double effort_min = -numeric_limits<double>::infinity();
  if (limit_node) {
    parseScalarAttribute(limit_node, "effort", effort_max);
    effort_min = -effort_max;

    // effort_min and effort_max take precedence over effort if they exist
    parseScalarAttribute(limit_node, "effort_min", effort_min);
    parseScalarAttribute(limit_node, "effort_max", effort_max);
  }

  model->actuators.push_back(RigidBodyActuator(
      actuator_name, model->body(body_index), gain, effort_min, effort_max));
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

void parseRobot(RigidBodyTree* model, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const DrakeJoint::FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr) {
  if (!node->Attribute("name"))
    throw runtime_error("Error: your robot must have a name attribute");

  string robotname = node->Attribute("name");

  // Parses the model's material elements.
  MaterialMap materials;
  for (XMLElement* link_node = node->FirstChildElement("material"); link_node;
       link_node = link_node->NextSiblingElement("material"))
    parseMaterial(link_node, materials);  // accept failed material parsing

  // Maintains a list of links that were added to the rigid body tree.
  // This is iterated over by method AddFloatingJoint() to determine where
  // to attach floating joints.
  std::vector<int> link_indices;

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link")) {
    int index;
    if (parseLink(model, robotname, link_node, materials, package_map, root_dir,
                  &index)) {
      link_indices.push_back(index);
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
  model->AddFloatingJoint(floating_base_type, link_indices, weld_to_frame);
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
  std::string linkname = link->Attribute("link");
  RigidBody* body = model.findLink(linkname);
  if (body == nullptr) {
    throw runtime_error("couldn't find link " + linkname +
                        " referenced in frame " + name);
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
