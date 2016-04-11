#include <fstream>
#include <sstream>
#include <string>

#include "spruce.hh"

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"
#include "joints/DrakeJoints.h"

#include "drake/Path.h"
#include "xmlUtil.h"

// from
// http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
#if defined(WIN32) || defined(WIN64)
#define POPEN _popen
#define PCLOSE _pclose
#else
#define POPEN popen
#define PCLOSE pclose
#endif

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

void parseSDFInertial(shared_ptr<RigidBody> body, XMLElement* node,
                      RigidBodyTree* model, PoseMap& pose_map,
                      const Isometry3d& T_link) {
  Isometry3d T = T_link;
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) poseValueToTransform(pose, pose_map, T, T_link);

  parseScalarValue(node, "mass", body->mass);

  body->com = T_link.inverse() * T.translation();

  Matrix<double, TWIST_SIZE, TWIST_SIZE> I =
      Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
  I.block(3, 3, 3, 3) << body->mass * Matrix3d::Identity();

  XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    parseScalarValue(inertia, "ixx", I(0, 0));
    parseScalarValue(inertia, "ixy", I(0, 1));
    I(1, 0) = I(0, 1);
    parseScalarValue(inertia, "ixz", I(0, 2));
    I(2, 0) = I(0, 2);
    parseScalarValue(inertia, "iyy", I(1, 1));
    parseScalarValue(inertia, "iyz", I(1, 2));
    I(2, 1) = I(1, 2);
    parseScalarValue(inertia, "izz", I(2, 2));
  }

  body->I = transformSpatialInertia(T_link.inverse() * T, I);
}

bool parseSDFGeometry(XMLElement* node, const PackageMap& package_map,
                      const string& root_dir, DrakeShapes::Element& element) {
  // DEBUG
  // cout << "parseGeometry: START" << endl;
  // END_DEBUG
  if (node->NoChildren()) {
    // This may not be true legal SDF, but is seen in practice.
    return true;
  }
  XMLElement* shape_node;
  if ((shape_node = node->FirstChildElement("box"))) {
    Vector3d xyz;
    if (!parseVectorValue(shape_node, "size", xyz)) {
      cerr << "ERROR parsing box element size" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Box(xyz));
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    double r = 0;
    if (!parseScalarValue(shape_node, "radius", r)) {
      cerr << "ERROR parsing sphere element radius" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Sphere(max(DrakeShapes::MIN_RADIUS, r)));
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    double r = 0, l = 0;
    if (!parseScalarValue(shape_node, "radius", r)) {
      cerr << "ERROR parsing cylinder element radius" << endl;
      return false;
    }

    if (!parseScalarValue(shape_node, "length", l)) {
      cerr << "ERROR parsing cylinder element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Cylinder(r, l));
  } else if ((shape_node = node->FirstChildElement("capsule"))) {
    double r = 0, l = 0;
    if (!parseScalarValue(shape_node, "radius", r)) {
      cerr << "ERROR parsing capsule element radius" << endl;
      return false;
    }

    if (!parseScalarValue(shape_node, "length", l)) {
      cerr << "ERROR: Failed to parse capsule element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    string uri;
    if (!parseStringValue(shape_node, "uri", uri)) {
      cerr << "ERROR mesh element has no uri tag" << endl;
      return false;
    }
    string resolved_filename = resolveFilename(uri, package_map, root_dir);
    DrakeShapes::Mesh mesh(uri, resolved_filename);

    parseScalarValue(shape_node, "scale", mesh.scale);
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

void parseSDFVisual(shared_ptr<RigidBody> body, XMLElement* node,
                    RigidBodyTree* model, const PackageMap& package_map,
                    const string& root_dir, PoseMap& pose_map,
                    const Isometry3d& transform_parent_to_model) {
  Isometry3d transform_to_model = transform_parent_to_model;
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose)
    poseValueToTransform(pose, pose_map, transform_to_model,
                         transform_parent_to_model);

  /*
    const char* attr = node->Attribute("name");
    if (!attr) throw runtime_error("ERROR: visual tag is missing name
    attribute");
    string name(attr);
  */
  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a visual element without geometry.");

  DrakeShapes::VisualElement element(transform_parent_to_model.inverse() *
                                     transform_to_model);
  if (!parseSDFGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse visual element in link " +
                        body->linkname + ".");

  XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    Vector4d rgba(0.7, 0.7, 0.7, 1.0);  // default color is a nice robot gray.
                                        // should only be used if diffuse is not
                                        // specified.
    parseVectorValue(material_node, "diffuse", rgba);
    element.setMaterial(rgba);
  }

  if (element.hasGeometry()) {
    // DEBUG
    // cout << "parseVisual: Adding element to body" << endl;
    // END_DEBUG
    body->addVisualElement(element);
  }
}

void parseSDFCollision(shared_ptr<RigidBody> body, XMLElement* node,
                       RigidBodyTree* model, const PackageMap& package_map,
                       const string& root_dir, PoseMap& pose_map,
                       const Isometry3d& transform_parent_to_model) {
  Isometry3d transform_to_model = transform_parent_to_model;
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose)
    poseValueToTransform(pose, pose_map, transform_to_model,
                         transform_parent_to_model);

  /*
    const char* attr = node->Attribute("name");
    if (!attr) throw runtime_error("ERROR: visual tag is missing name
    attribute");
    string name(attr);
  */
  string group_name("default");

  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw runtime_error("ERROR: Link " + body->linkname +
                        " has a collision element without geometry.");

  RigidBody::CollisionElement element(
      transform_parent_to_model.inverse() * transform_to_model, body);
  if (!parseSDFGeometry(geometry_node, package_map, root_dir, element))
    throw runtime_error("ERROR: Failed to parse collision element in link " +
                        body->linkname + ".");

  if (element.hasGeometry()) {
    model->addCollisionElement(element, *body, group_name);
  }
}

void parseSDFLink(RigidBodyTree* model, std::string model_name,
                  XMLElement* node, const PackageMap& package_map,
                  PoseMap& pose_map, const string& root_dir) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  shared_ptr<RigidBody> body(new RigidBody());
  body->model_name = model_name;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");
  body->linkname = attr;

  if (body->linkname == "world")
    throw runtime_error(
        "ERROR: do not name a link 'world', it is a reserved name");

  Isometry3d transform_to_model = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    poseValueToTransform(pose, pose_map, transform_to_model);
    pose_map.insert(
        std::pair<string, Isometry3d>(body->linkname, transform_to_model));
  }

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node)
    parseSDFInertial(body, inertial_node, model, pose_map, transform_to_model);

  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    parseSDFVisual(body, visual_node, model, package_map, root_dir, pose_map,
                   transform_to_model);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    parseSDFCollision(body, collision_node, model, package_map, root_dir,
                      pose_map, transform_to_model);
  }

  model->bodies.push_back(body);
  body->body_index = static_cast<int>(model->bodies.size()) - 1;
}

template <typename JointType>
void setSDFLimits(XMLElement* node, FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* limit_node = node->FirstChildElement("limit");
  if (fjoint != nullptr && limit_node) {
    double lower = -numeric_limits<double>::infinity(),
           upper = numeric_limits<double>::infinity();
    parseScalarValue(limit_node, "lower", lower);
    parseScalarValue(limit_node, "upper", upper);
    fjoint->setJointLimits(lower, upper);
  }
}

template <typename JointType>
void setSDFDynamics(RigidBodyTree* model, XMLElement* node,
                    FixedAxisOneDoFJoint<JointType>* fjoint) {
  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (fjoint != nullptr && dynamics_node) {
    double damping = 0.0, coulomb_friction = 0.0, coulomb_window = 0.0;
    parseScalarValue(dynamics_node, "damping", damping);
    parseScalarValue(dynamics_node, "friction", coulomb_friction);
    parseScalarValue(dynamics_node, "coulomb_window",
                     coulomb_window);  // note: not a part of the sdf spec
    fjoint->setDynamics(damping, coulomb_friction, coulomb_window);
  }
}

void parseSDFFrame(RigidBodyTree* model, std::string model_name,
                   XMLElement* node) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: frame tag is missing name attribute");
  string name(attr);

  // Parse the link
  string link_name;
  if (!parseStringValue(node, "link", link_name))
    throw runtime_error("ERROR: frame " + name + " doesn't have a link node");

  // The following will throw a std::runtime_error if the link doesn't exist.
  std::shared_ptr<RigidBody> link = model->findLink(link_name, model_name);

  // Get the frame's pose
  XMLElement* pose = node->FirstChildElement("pose");
  if (!pose)
    throw runtime_error("ERROR: frame \"" + name +
                        "\" is missing its pose tag");

  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();

  const char* strval = pose->FirstChild()->Value();
  if (strval) {
    std::stringstream s(strval);
    s >> xyz(0) >> xyz(1) >> xyz(2) >> rpy(0) >> rpy(1) >> rpy(2);
  }

  // Create the frame
  std::shared_ptr<RigidBodyFrame> frame = allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), name, link, xyz, rpy);

  model->addFrame(frame);
}

void parseSDFJoint(RigidBodyTree* model, std::string model_name,
                   XMLElement* node, PoseMap& pose_map) {
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
  string parent_name;
  if (!parseStringValue(node, "parent", parent_name))
    throw runtime_error("ERROR: joint " + name + " doesn't have a parent node");

  auto parent = model->findLink(parent_name, model_name);
  if (!parent)
    throw runtime_error("ERROR: could not find parent link named " +
                        parent_name);

  // parse child
  string child_name;
  if (!parseStringValue(node, "child", child_name))
    throw runtime_error("ERROR: joint " + name + " doesn't have a child node");

  auto child = model->findLink(child_name, model_name);
  if (!child)
    throw runtime_error("ERROR: could not find child link named " + child_name);

  Isometry3d transform_child_to_model = Isometry3d::Identity(),
             transform_parent_to_model = Isometry3d::Identity();

  // Obtain the child-to-model frame transformation.
  if (pose_map.find(child_name) != pose_map.end())
    transform_child_to_model = pose_map.at(child_name);

  // Obtain the parent-to-model frame transformation.
  if (pose_map.find(parent_name) != pose_map.end())
    transform_parent_to_model = pose_map.at(parent_name);

  // By default, a joint is defined in the child's coordinate frame.
  // This was determined by studying valid SDF files available at:
  // https://bitbucket.org/osrf/gazebo_models/src
  Isometry3d transform_to_model = transform_child_to_model;

  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    // Read the joint's pose using the child link's coordinate frame by default.
    poseValueToTransform(pose, pose_map, transform_to_model,
                         transform_child_to_model);
  }

  if (pose_map.find(child_name) == pose_map.end()) {
    // The child link is not in the pose map. Thus, this joint actually defines
    // the pose of a previously-unspecified link frame. Adds this link's
    // transform to the model coordinate frame to the pose map.
    pose_map.insert(pair<string, Isometry3d>(child_name, transform_to_model));
  }

  Vector3d axis;
  axis << 1, 0, 0;
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && type.compare("fixed") != 0 &&
      type.compare("floating") != 0) {
    parseVectorValue(axis_node, "xyz", axis);
    if (axis.norm() < 1e-8)
      throw runtime_error("ERROR: axis is zero.  don't do that");
    axis.normalize();
    double in_parent_model_frame;
    if (parseScalarValue(axis_node, "use_parent_model_frame",
                         in_parent_model_frame) &&
        in_parent_model_frame > 0.0) {
      // The joint's axis of rotation should be interpreted as being in the
      // model coordinate frame. To be compatible with Drake, the joint's axis
      // must be defined in the joint's coordinate frame. Since we are
      // transforming the frame of an axis and not a point, we only want to
      // use the linear (rotational) part of the transform_to_model matrix.
      axis = transform_to_model.linear().inverse() * axis;
    }
  }

  // Obtain the transform from the joint frame to the parent link's frame.
  Isometry3d transform_to_parent_body =
      transform_parent_to_model.inverse() * transform_to_model;

  if (child->hasParent()) {
    // ... then implement it as a loop joint.

    // Gets the loop point in the joint's reference frame. Since the SDF
    // standard specifies that the joint's reference frame is defined by the
    // child link's reference frame, the loop point in the joint's reference
    // frame is simply the pose of the joint.
    Eigen::Vector3d loop_point_child = Eigen::Vector3d::Zero();
    {
      const char* strval = pose->FirstChild()->Value();
      if (strval) {
        std::stringstream s(strval);
        s >> loop_point_child(0) >> loop_point_child(1) >> loop_point_child(2);
      } else {
        throw runtime_error("ERROR: Unable to construct loop joint \"" + name +
                            "\": could not parse loop point.");
      }
    }

    // Get the loop point in the parent's reference frame.
    Eigen::Vector3d loop_point_model =
        transform_child_to_model * loop_point_child;

    Eigen::Vector3d loop_point_parent =
        transform_parent_to_model.inverse() * loop_point_model;

    std::shared_ptr<RigidBodyFrame> frameA = allocate_shared<RigidBodyFrame>(
        Eigen::aligned_allocator<RigidBodyFrame>(), name + "FrameA", parent,
        loop_point_parent, Vector3d::Zero());

    std::shared_ptr<RigidBodyFrame> frameB = allocate_shared<RigidBodyFrame>(
        Eigen::aligned_allocator<RigidBodyFrame>(), name + "FrameB", child,
        loop_point_child, Vector3d::Zero());

    model->addFrame(frameA);
    model->addFrame(frameB);
    RigidBodyLoop l(frameA, frameB, axis);
    model->loops.push_back(l);

  } else {
    // Update the reference frames of the child link's inertia, visual,
    // and collision elements to be this joint's frame.
    child->ApplyTransformToJointFrame(transform_to_model.inverse() *
                                      transform_child_to_model);

    for (const auto& c : child->collision_element_ids) {
      if (!model->transformCollisionFrame(
              c, transform_to_model.inverse() * transform_child_to_model)) {
        std::stringstream ss;
        ss << "RigidBodyTreeSDF::parseSDFJoint: Collision element with ID " << c
           << " not found! Cannot update its local frame to be that of joint.";
        throw std::runtime_error(ss.str());
      }
    }

    // Update pose_map with child's new frame, which is now the same as this
    // joint's frame.
    auto it = pose_map.find(child_name);
    if (it != pose_map.end()) {
      it->second = transform_to_model;
    } else {
      throw runtime_error(
          "ERROR: Unable to update transform_to_model of link " + child_name);
    }

    // construct the actual joint (based on its type)
    DrakeJoint* joint = nullptr;

    if (type.compare("revolute") == 0 || type.compare("gearbox") == 0) {
      FixedAxisOneDoFJoint<RevoluteJoint>* fjoint =
          new RevoluteJoint(name, transform_to_parent_body, axis);
      if (axis_node) {
        setSDFDynamics(model, axis_node, fjoint);
        setSDFLimits(axis_node, fjoint);

        double effort_limit = std::numeric_limits<double>::infinity();

        XMLElement* limit_node = axis_node->FirstChildElement("limit");
        if (limit_node) parseScalarValue(limit_node, "effort", effort_limit);

        if (effort_limit != 0.0) {
          RigidBodyActuator actuator(name, child, 1.0, -effort_limit,
                                     effort_limit);
          model->actuators.push_back(actuator);
        }
      }

      joint = fjoint;

    } else if (type.compare("fixed") == 0) {
      joint = new FixedJoint(name, transform_to_parent_body);
    } else if (type.compare("prismatic") == 0) {
      FixedAxisOneDoFJoint<PrismaticJoint>* fjoint =
          new PrismaticJoint(name, transform_to_parent_body, axis);
      if (axis_node) {
        setSDFDynamics(model, axis_node, fjoint);
        setSDFLimits(axis_node, fjoint);
      }
      joint = fjoint;
      double effort_limit = std::numeric_limits<double>::infinity();
      XMLElement* limit_node = axis_node->FirstChildElement("limit");
      if (limit_node) parseScalarValue(limit_node, "effort", effort_limit);
      if (effort_limit != 0.0) {
        RigidBodyActuator actuator(name, child, 1.0, -effort_limit,
                                   effort_limit);
        model->actuators.push_back(actuator);
      }
    } else if (type.compare("floating") == 0) {
      joint = new RollPitchYawFloatingJoint(name, transform_to_parent_body);
    } else {
      throw runtime_error("ERROR: Unrecognized joint type: " + type);
    }

    unique_ptr<DrakeJoint> joint_unique_ptr(joint);
    child->setJoint(move(joint_unique_ptr));
    child->parent = parent;
  }
}

void parseModel(RigidBodyTree* model, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const DrakeJoint::FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  PoseMap pose_map;  // because sdf specifies almost everything in the global
                     // (actually model) coordinates instead of relative
                     // coordinates.  sigh...

  if (!node->Attribute("name"))
    throw runtime_error("Error: your model must have a name attribute");
  string model_name = node->Attribute("name");

  Isometry3d transform_to_world = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    poseValueToTransform(pose, pose_map, transform_to_world);
    if (transform_to_world.matrix() != Isometry3d::Identity().matrix() &&
        weld_to_frame != nullptr) {
      throw std::runtime_error(
          "SDF model double offset to world not yet supported");
    }
  }

  // parse link elements
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link"))
    parseSDFLink(model, model_name, link_node, package_map, pose_map, root_dir);

  // parse joints
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint"))
    parseSDFJoint(model, model_name, joint_node, pose_map);

  // parse frames
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame"))
    parseSDFFrame(model, model_name, frame_node);

  bool has_root_node = false;
  for (unsigned int i = 1; i < model->bodies.size(); i++) {
    if (model->bodies[i]->parent == nullptr) {  // attach the root nodes to the
                                                // world with a floating base
                                                // joint
      has_root_node = true;
      model->bodies[i]->parent = model->bodies[0];

      Isometry3d transform_to_model = Isometry3d::Identity();
      if (pose_map.find(model->bodies[i]->linkname) != pose_map.end())
        transform_to_model = pose_map.at(model->bodies[i]->linkname);

      switch (floating_base_type) {
        case DrakeJoint::FIXED: {
          unique_ptr<DrakeJoint> joint(
              new FixedJoint("base", transform_to_world * transform_to_model));
          model->bodies[i]->setJoint(move(joint));
        } break;
        case DrakeJoint::ROLLPITCHYAW: {
          unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              "base", transform_to_world * transform_to_model));
          model->bodies[i]->setJoint(move(joint));
        } break;
        case DrakeJoint::QUATERNION: {
          unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              "base", transform_to_world * transform_to_model));
          model->bodies[i]->setJoint(move(joint));
        } break;
        default:
          throw std::runtime_error("unknown floating base type");
      }
    }
  }
  if (!has_root_node)
    throw runtime_error(
        "Your model does not have a root link (every link has a joint "
        "connecting it to some other joint).  You're about to loop "
        "indefinitely in the compile() method.  Still need to handle this "
        "case");
  // could handle it by disconnecting one of the internal nodes, making that a
  // loop joint, and connecting the new free joint to the world
}

void parseWorld(RigidBodyTree* model, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const DrakeJoint::FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  for (XMLElement* model_node = node->FirstChildElement("model"); model_node;
       model_node = model_node->NextSiblingElement("model")) {
    parseModel(model, model_node, package_map, root_dir, floating_base_type,
               weld_to_frame);
  }
}

void parseSDF(RigidBodyTree* model, XMLDocument* xml_doc,
              PackageMap& package_map, const string& root_dir,
              const DrakeJoint::FloatingBaseType floating_base_type,
              std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  populatePackageMap(package_map);

  XMLElement* node = xml_doc->FirstChildElement("sdf");
  if (!node)
    throw std::runtime_error(
        "ERROR: This xml file does not contain an sdf tag");

  // If we have a world, load it.
  XMLElement* world_node = node->FirstChildElement("world");
  if (world_node) {
    // If we have more than one world, it is ambiguous which one the user
    // wishes.
    if (world_node->NextSiblingElement("world")) {
      throw runtime_error("ERROR: Multiple worlds in file, ambiguous.");
    }
    parseWorld(model, world_node, package_map, root_dir, floating_base_type,
               weld_to_frame);
  }

  // Load all models not in a world.
  for (XMLElement* model_node = node->FirstChildElement("model"); model_node;
       model_node = model_node->NextSiblingElement("model"))
    parseModel(model, model_node, package_map, root_dir, floating_base_type,
               weld_to_frame);

  model->compile();
}

void RigidBodyTree::addRobotFromSDF(
    const string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  PackageMap package_map;

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

  parseSDF(this, &xml_doc, package_map, root_dir, floating_base_type,
           weld_to_frame);
}
