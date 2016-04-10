#include <string>
#include <fstream>
#include <sstream>

#include "drake/systems/plants/RigidBodyTree.h"
#include "joints/DrakeJoints.h"

#include "xmlUtil.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

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

RigidBodyFrame::RigidBodyFrame(RigidBodyTree* tree, XMLElement* link_reference,
                               XMLElement* pose, std::string name)
    : name(name), frame_index(0) {
  string linkname = link_reference->Attribute("link");
  body = tree->findLink(linkname);
  if (!body)
    throw runtime_error("couldn't find link %s referenced in frame " + name);

  Vector3d xyz = Vector3d::Zero(), rpy = Vector3d::Zero();
  if (pose) {
    parseVectorAttribute(pose, "xyz", xyz);
    parseVectorAttribute(pose, "rpy", rpy);
  }
  transform_to_body.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

void parseInertial(shared_ptr<RigidBody> body, XMLElement* node,
                   RigidBodyTree* model) {
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

void parseVisual(shared_ptr<RigidBody> body, XMLElement* node,
                 RigidBodyTree* model, const MaterialMap& materials,
                 const PackageMap& package_map, const string& root_dir) {
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
                "nor a nested color element" << endl;
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

void parseCollision(shared_ptr<RigidBody> body, XMLElement* node,
                    RigidBodyTree* model, const PackageMap& package_map,
                    const string& root_dir) {
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
    ;
  }

  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a collision element without geometry");

  RigidBody::CollisionElement element(T_element_to_link, body);
  if (!parseGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse collision element in link " +
                        body->linkname + ".");

  if (element.hasGeometry()) {
    model->addCollisionElement(element, *body, group_name);
  }
}

void parseLink(RigidBodyTree* model, std::string robot_name, XMLElement* node,
               const MaterialMap& materials, const PackageMap& package_map,
               const string& root_dir) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  shared_ptr<RigidBody> body(new RigidBody());
  body->model_name = robot_name;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");

  body->linkname = attr;
  if (body->linkname == "world")
    throw runtime_error(
        "ERROR: do not name a link 'world', it is a reserved name");

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

  model->bodies.push_back(body);
  body->body_index = static_cast<int>(model->bodies.size()) - 1;
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
  model->bodies[child_index]->parent = model->bodies[parent_index];
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
            "element will be skipped." << endl;
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
            "joint: " << joint_name << endl;
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
      actuator_name, model->bodies[body_index], gain, effort_min, effort_max));
}

void parseLoop(RigidBodyTree* model, XMLElement* node) {
  Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  if (!node || !node->Attribute("name"))
    throw runtime_error("ERROR: loop is missing a name element");
  string name(node->Attribute("name"));

  XMLElement* link_node = node->FirstChildElement("link1");
  std::shared_ptr<RigidBodyFrame> frameA = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), model, link_node, link_node,
      name + "FrameA");

  link_node = node->FirstChildElement("link2");
  std::shared_ptr<RigidBodyFrame> frameB = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), model, link_node, link_node,
      name + "FrameB");

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

  std::shared_ptr<RigidBodyFrame> frame = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), model, node, node,
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

  // parse material elements
  MaterialMap materials;
  for (XMLElement* link_node = node->FirstChildElement("material"); link_node;
       link_node = link_node->NextSiblingElement("material"))
    parseMaterial(link_node, materials);  // accept failed material parsing

  // parse link elements
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link"))
    parseLink(model, robotname, link_node, materials, package_map, root_dir);

  // DEBUG
  // else {
  // cout << "Parsed link" << endl;
  // cout << "model->bodies.size() = " << model->bodies.size() << endl;
  // cout << "model->num_bodies = " << model->num_bodies << endl;
  //}
  // END_DEBUG

  // todo: parse collision filter groups

  // parse joints
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint"))
    parseJoint(model, joint_node);

  // parse transmission elements
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission"))
    parseTransmission(model, transmission_node);

  // parse loop joints
  for (XMLElement* loop_node = node->FirstChildElement("loop_joint"); loop_node;
       loop_node = loop_node->NextSiblingElement("loop_joint"))
    parseLoop(model, loop_node);

  // parse Drake frames
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame"))
    parseFrame(model, frame_node);

  std::string floating_joint_name;
  std::shared_ptr<RigidBody> weld_to_body;
  Isometry3d transform_to_body;
  if (!weld_to_frame) {
    // If no body was given for us to weld to, then weld to the world
    weld_to_body = model->bodies[0];
    floating_joint_name = "base";
    transform_to_body = Isometry3d::Identity();
  } else {
    weld_to_body = weld_to_frame->body;
    transform_to_body = weld_to_frame->transform_to_body;
    floating_joint_name = "weld";
  }

  for (unsigned int i = 1; i < model->bodies.size(); i++) {
    if (model->bodies[i]->parent == nullptr) {  // attach the root nodes to the
                                                // world with a floating base
                                                // joint
      model->bodies[i]->parent = weld_to_body;
      switch (floating_base_type) {
        case DrakeJoint::FIXED: {
          unique_ptr<DrakeJoint> joint(
              new FixedJoint(floating_joint_name, transform_to_body));
          model->bodies[i]->setJoint(move(joint));
        } break;
        case DrakeJoint::ROLLPITCHYAW: {
          unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              floating_joint_name, transform_to_body));
          model->bodies[i]->setJoint(move(joint));
        } break;
        case DrakeJoint::QUATERNION: {
          unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              floating_joint_name, transform_to_body));
          model->bodies[i]->setJoint(move(joint));
        } break;
        default:
          throw std::runtime_error("unknown floating base type");
      }
    }
  }
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
    std::shared_ptr<RigidBodyFrame> weld_to_frame) { //weld_to_frame defaults to nullptr in RigidBodyTree.h
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
