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

void parseSDFInertial(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                      PoseMap& pose_map, const Isometry3d& T_link) {
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
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Failed to parse box element size."
           << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Box(xyz));
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    double r = 0;
    if (!parseScalarValue(shape_node, "radius", r)) {
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Failed to parse sphere element radius."
           << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Sphere(max(DrakeShapes::MIN_RADIUS, r)));
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    double r = 0, l = 0;
    if (!parseScalarValue(shape_node, "radius", r)) {
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Failed to parse cylinder element radius."
           << endl;
      return false;
    }

    if (!parseScalarValue(shape_node, "length", l)) {
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Failed to parse cylinder element length."
           << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Cylinder(r, l));
  } else if ((shape_node = node->FirstChildElement("capsule"))) {
    double r = 0, l = 0;
    if (!parseScalarValue(shape_node, "radius", r)) {
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Failed to parse capsule element radius."
           << endl;
      return false;
    }

    if (!parseScalarValue(shape_node, "length", l)) {
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Failed to parse capsule element length."
           << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    string uri;
    if (!parseStringValue(shape_node, "uri", uri)) {
      cerr << std::string(__FILE__) + ": " + __func__ +
                  ": ERROR: Mesh element has no uri tag."
           << endl;
      return false;
    }
    string resolved_filename = resolveFilename(uri, package_map, root_dir);
    DrakeShapes::Mesh mesh(uri, resolved_filename);

    if (shape_node->FirstChildElement("scale") != nullptr)
      ParseThreeVectorValue(shape_node, "scale", &mesh.scale);
    element.setGeometry(mesh);
  } else {
    cerr << std::string(__FILE__) + ": " + __func__ + ": WARNING: "
         << "Geometry element has an unknown type and will be ignored." << endl;
  }
  // DEBUG
  // cout << "parseGeometry: END" << endl;
  // END_DEBUG
  return true;
}

void parseSDFVisual(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                    const PackageMap& package_map, const string& root_dir,
                    PoseMap& pose_map,
                    const Isometry3d& transform_parent_to_model) {
  Isometry3d transform_to_model = transform_parent_to_model;
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    poseValueToTransform(pose, pose_map, transform_to_model,
                         transform_parent_to_model);
  }

  /*
    const char* attr = node->Attribute("name");
    if (!attr) throw runtime_error("ERROR: visual tag is missing name
    attribute");
    string name(attr);
  */
  XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Link " + body->name_ +
                        " has a visual element without a geometry.");
  }

  DrakeShapes::VisualElement element(transform_parent_to_model.inverse() *
                                     transform_to_model);
  if (!parseSDFGeometry(geometry_node, package_map, root_dir, element)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Failed to parse visual element in link " +
                        body->name_ + ".");
  }

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

void parseSDFCollision(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                       const PackageMap& package_map, const string& root_dir,
                       PoseMap& pose_map,
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
  if (!geometry_node) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Link " + body->name_ +
                        " has a collision element without a geometry.");
  }

  RigidBody::CollisionElement element(
      transform_parent_to_model.inverse() * transform_to_model, body);
  if (!parseSDFGeometry(geometry_node, package_map, root_dir, element)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Failed to parse collision element in link " +
                        body->name_ + ".");
  }

  if (element.hasGeometry())
    model->addCollisionElement(element, *body, group_name);
}

bool parseSDFLink(RigidBodyTree* model, std::string model_name,
                  XMLElement* node, const PackageMap& package_map,
                  PoseMap& pose_map, const string& root_dir, int* index,
                  int model_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return false;

  RigidBody* body{nullptr};
  std::unique_ptr<RigidBody> owned_body(body = new RigidBody());
  body->model_name_ = model_name;
  body->set_model_id(model_id);

  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Link tag is missing a name attribute.");
  }
  body->name_ = attr;

  if (body->name_ == std::string(RigidBodyTree::kWorldLinkName)) {
    throw runtime_error(
        std::string(__FILE__) + ": " + __func__ +
        ": ERROR: Do not name a link 'world' because it is a reserved name.");
  }

  Isometry3d transform_to_model = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    poseValueToTransform(pose, pose_map, transform_to_model);
    pose_map.insert(
        std::pair<string, Isometry3d>(body->name_, transform_to_model));
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

  model->add_rigid_body(std::move(owned_body));
  *index = body->body_index;
  return true;
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

void parseSDFFrame(RigidBodyTree* rigid_body_tree, XMLElement* node,
                   int model_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Frame tag is missing a name attribute.");
  }
  string name(attr);

  // Parse the link
  string link_name;
  if (!parseStringValue(node, "link", link_name)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Frame \"" + name +
                        "\" doesn't have a link node.");
  }

  // The following will throw a std::runtime_error if the link doesn't exist.
  RigidBody* link = rigid_body_tree->findLink(link_name, "", model_id);

  // Get the frame's pose
  XMLElement* pose = node->FirstChildElement("pose");
  if (!pose) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Frame \"" + name +
                        "\" is missing its pose tag.");
  }

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

  rigid_body_tree->addFrame(frame);
}

void parseSDFJoint(RigidBodyTree* model, std::string model_name,
                   XMLElement* node, PoseMap& pose_map, int model_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Joint tag is missing its name attribute.");
  }
  string name(attr);

  attr = node->Attribute("type");
  if (!attr) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Joint \"" + name +
                        "\" is missing a type attribute.");
  }
  string type(attr);

  // parse parent
  string parent_name;
  if (!parseStringValue(node, "parent", parent_name)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Joint \"" + name +
                        "\" doesn't have a parent node.");
  }

  auto parent = model->findLink(parent_name, "", model_id);
  if (!parent) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Failed to find a parent link named \"" +
                        parent_name + "\" for joint \"" + name + "\".");
  }

  // parse child
  string child_name;
  if (!parseStringValue(node, "child", child_name)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Joint \"" + name +
                        "\" doesn't have a child node.");
  }

  auto child = model->findLink(child_name, "", model_id);
  if (!child) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Failed to find a child link named " +
                        child_name + ".");
  }

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
    if (axis.norm() < 1e-8) {
      throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                          ": ERROR: No axis specified.");
    }
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
        throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                            ": ERROR: Failed to construct loop joint \"" +
                            name + "\".");
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
        ss << std::string(__FILE__) << ": " << __func__
           << ": ERROR: Collision element with ID " << c
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
          std::string(__FILE__) + ": " + __func__ +
          ": ERROR: Unable to update transform_to_model of link " + child_name +
          ".");
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
      throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                          ": ERROR: Unrecognized joint type: " + type + ".");
    }

    unique_ptr<DrakeJoint> joint_unique_ptr(joint);
    child->setJoint(move(joint_unique_ptr));
    child->parent = parent;
  }
}

/**
 * Parses a model and adds it to the rigid body tree.
 *
 * @param rigid_body_tree A pointer to the rigid body tree to which to add the
 * model.
 * @param node The XML node containing the model information.
 * @param package_map A map containing information about the ROS workspace
 * in which to search for meshes.
 * @param root_dir The root directory from which to search for mesh files.
 * @param floating_base_type The type of floating joint to use to weld the
 * newly added model to the rigid body tree.
 * @param weld_to_frame Specifies the initial pose of the newly added robot
 * relative to the link to which the robot is being welded.
 */
void parseModel(RigidBodyTree* rigid_body_tree, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const DrakeJoint::FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  // The pose_map is needed because SDF specifies almost everything in the
  // model's coordinate frame.
  PoseMap pose_map;

  if (!node->Attribute("name")) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: The model must have a name attribute.");
  }

  string model_name = node->Attribute("name");

  int model_id = rigid_body_tree->get_next_model_id();

  // Maintains a list of links that were added to the rigid body tree.
  // This is iterated over by method AddFloatingJoint() to determine where
  // to attach floating joints.
  std::vector<int> link_indices;

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link")) {
    int index;
    if (parseSDFLink(rigid_body_tree, model_name, link_node, package_map,
                     pose_map, root_dir, &index, model_id)) {
      link_indices.push_back(index);
    }
  }

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    parseSDFJoint(rigid_body_tree, model_name, joint_node, pose_map, model_id);
  }

  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame")) {
    parseSDFFrame(rigid_body_tree, frame_node, model_id);
  }

  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    // Sets a default value for weld_to_frame if none was set.
    // By default, the robot is welded to the world frame.
    if (weld_to_frame == nullptr) {
      weld_to_frame = std::allocate_shared<RigidBodyFrame>(
          Eigen::aligned_allocator<RigidBodyFrame>(),
          std::string(RigidBodyTree::kWorldLinkName),
          nullptr,  // Valid since the robot is attached to the world.
          Eigen::Isometry3d::Identity());
    }

    // Obtains the transform from the frame of the model's root link to the
    // frame of the model's world.
    Isometry3d transform_model_root_to_model_world = Isometry3d::Identity();
    poseValueToTransform(pose, pose_map, transform_model_root_to_model_world);

    // Implements dual-offset: one from model root to model world, another
    // from model world to Drake's world.
    weld_to_frame->transform_to_body =
        weld_to_frame->transform_to_body * transform_model_root_to_model_world;
  }

  // Adds the floating joint that connects the newly added robot model to the
  // rest of the rigid body tree.
  rigid_body_tree->AddFloatingJoint(floating_base_type, link_indices,
                                    weld_to_frame, &pose_map);
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
  if (!node) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ +
        ": ERROR: The XML file does not contain an sdf tag.");
  }

  // Loads the world if it is defined.
  XMLElement* world_node =
      node->FirstChildElement(RigidBodyTree::kWorldLinkName);
  if (world_node) {
    // If we have more than one world, it is ambiguous which one the user
    // wishes to use.
    if (world_node->NextSiblingElement(RigidBodyTree::kWorldLinkName)) {
      throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                          ": ERROR: Multiple worlds in one file.");
    }
    parseWorld(model, world_node, package_map, root_dir, floating_base_type,
               weld_to_frame);
  }

  // Load all models not in a world.
  for (XMLElement* model_node = node->FirstChildElement("model"); model_node;
       model_node = model_node->NextSiblingElement("model")) {
    parseModel(model, model_node, package_map, root_dir, floating_base_type,
               weld_to_frame);
  }

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
    throw std::runtime_error(std::string(__FILE__) + ": " + __func__ +
                             ": ERROR: Failed to parse XML in file " +
                             urdf_filename + "\n" + xml_doc.ErrorName() + ".");
  }

  string root_dir = ".";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  parseSDF(this, &xml_doc, package_map, root_dir, floating_base_type,
           weld_to_frame);
}
