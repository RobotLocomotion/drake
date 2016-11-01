#include "drake/systems/plants/parser_sdf.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "spruce.hh"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/plants/joints/DrakeJoints.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_common.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

// from
// http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
#if defined(WIN32) || defined(WIN64)
#define POPEN _popen
#define PCLOSE _pclose
#else
#define POPEN popen
#define PCLOSE pclose
#endif

namespace drake {
namespace parsers {
namespace sdf {
namespace {

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using std::allocate_shared;
using std::cerr;
using std::endl;
using std::max;
using std::move;
using std::numeric_limits;
using std::pair;
using std::runtime_error;
using std::string;
using std::unique_ptr;

using tinyxml2::XMLElement;
using tinyxml2::XMLDocument;

using drake::systems::plants::joints::FloatingBaseType;

void ParseSdfInertial(
    RigidBody* body, XMLElement* node, RigidBodyTree* model,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    PoseMap& pose_map,
    const Isometry3d& T_link) {
  Isometry3d T = T_link;
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) poseValueToTransform(pose, pose_map, T, T_link);

  double mass = {0};
  parseScalarValue(node, "mass", mass);
  body->set_mass(mass);

  Eigen::Vector3d com;
  com = T_link.inverse() * T.translation();
  body->set_center_of_mass(com);

  drake::SquareTwistMatrix<double> I = drake::SquareTwistMatrix<double>::Zero();
  I.block(3, 3, 3, 3) << body->get_mass() * Matrix3d::Identity();

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

  body->set_spatial_inertia(transformSpatialInertia(T_link.inverse() * T, I));
}

bool ParseSdfGeometry(XMLElement* node, const PackageMap& package_map,
                      const string& root_dir,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      DrakeShapes::Element& element) {
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

    // This method will return an empty string if the file is not found or
    // resolved within a ROS package.
    string resolved_filename = resolveFilename(uri, package_map, root_dir);

    if (resolved_filename.empty()) {
      throw runtime_error(std::string(__FILE__) + ": " + __func__ +
          ": ERROR: Mesh file name could not be resolved from the "
          "provided uri \"" + uri + "\".");
    }
    DrakeShapes::Mesh mesh(uri, resolved_filename);

    if (shape_node->FirstChildElement("scale") != nullptr)
      ParseThreeVectorValue(shape_node, "scale", &mesh.scale_);
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

void ParseSdfVisual(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                    const PackageMap& package_map, const string& root_dir,
                    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
                        ": ERROR: Link " + body->get_name() +
                        " has a visual element without a geometry.");
  }

  DrakeShapes::VisualElement element(transform_parent_to_model.inverse() *
                                     transform_to_model);

  if (!ParseSdfGeometry(geometry_node, package_map, root_dir, element)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Failed to parse visual element in link " +
                        body->get_name() + ".");
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
    body->AddVisualElement(element);
  }
}

void ParseSdfCollision(RigidBody* body, XMLElement* node, RigidBodyTree* model,
                       const PackageMap& package_map, const string& root_dir,
                       // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
                        ": ERROR: Link " + body->get_name() +
                        " has a collision element without a geometry.");
  }

  DrakeCollision::Element element(
      transform_parent_to_model.inverse() * transform_to_model, body);
  // By default all collision elements added to the world from an SDF file are
  // flagged as static.
  // We would also like to flag as static bodies connected to the world with a
  // FloatingBaseType::kFixed joint.
  // However this is not possible at this stage since joints were not parsed
  // yet.
  // Solutions to this problem would be:
  //  1. To load the model with DrakeCollision::Element's here but flag them as
  //     static later at a the compile stage. This means that Bullet objects are
  //     not created here (with addCollisionElement) but later on with the call
  //     to RBT::compile when all the connectivity information is available.
  //  2. Load collision elements on a separate pass after links and joints were
  //     already loaded.
  //  Issue 2661 was created to track this problem.
  // TODO(amcastro-tri): fix the above issue tracked by 2661. Similarly for
  // parseCollision in RigidBodyTreeURDF.cpp.
  if (body->get_name().compare(std::string(RigidBodyTree::kWorldName)) == 0)
    element.set_static();

  if (!ParseSdfGeometry(geometry_node, package_map, root_dir, element)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Failed to parse collision element in link " +
                        body->get_name() + ".");
  }

  if (element.hasGeometry())
    model->addCollisionElement(element, *body, group_name);
}

bool ParseSdfLink(RigidBodyTree* model, std::string model_name,
                  XMLElement* node, const PackageMap& package_map,
                  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                  PoseMap& pose_map,
                  const string& root_dir, int* index,
                  int model_instance_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return false;

  RigidBody* body{nullptr};
  std::unique_ptr<RigidBody> owned_body(body = new RigidBody());
  body->set_model_name(model_name);
  body->set_model_instance_id(model_instance_id);

  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Link tag is missing a name attribute.");
  }
  body->set_name(std::string(attr));

  if (body->get_name() == std::string(RigidBodyTree::kWorldName)) {
    throw runtime_error(
        std::string(__FILE__) + ": " + __func__ +
        ": ERROR: Do not name a link 'world' because it is a reserved name.");
  }

  Isometry3d transform_to_model = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    poseValueToTransform(pose, pose_map, transform_to_model);
    pose_map.insert(
        std::pair<string, Isometry3d>(body->get_name(), transform_to_model));
  }

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node)
    ParseSdfInertial(body, inertial_node, model, pose_map, transform_to_model);

  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    ParseSdfVisual(body, visual_node, model, package_map, root_dir, pose_map,
                   transform_to_model);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    ParseSdfCollision(body, collision_node, model, package_map, root_dir,
                      pose_map, transform_to_model);
  }

  model->add_rigid_body(std::move(owned_body));
  *index = body->get_body_index();
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

    double stiffness = fjoint->get_joint_limit_stiffness()(0);
    double dissipation = fjoint->get_joint_limit_dissipation()(0);
    parseScalarValue(limit_node, "stiffness", stiffness);
    parseScalarValue(limit_node, "dissipation", dissipation);
    fjoint->SetJointLimitDynamics(stiffness, dissipation);
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

void ParseSdfFrame(RigidBodyTree* rigid_body_tree, XMLElement* node,
                   int model_instance_id) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  attr = node->Attribute("name");
  if (!attr) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Frame tag is missing a name attribute.");
  }
  string name(attr);

  // Parses the body.
  string body_name;
  if (!parseStringValue(node, "link", body_name)) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: Frame \"" + name +
                        "\" doesn't have a link node.");
  }

  // The following will throw a std::runtime_error if the link doesn't exist.
  RigidBody* link = rigid_body_tree->FindBody(body_name, "", model_instance_id);

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

void ParseSdfJoint(RigidBodyTree* model, std::string model_name,
                   XMLElement* node,
                  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   PoseMap& pose_map,
                   int model_instance_id) {
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

  auto parent = model->FindBody(parent_name, "", model_instance_id);
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

  auto child = model->FindBody(child_name, "", model_instance_id);
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

  if (child->has_parent_body()) {
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

    // This log statement is required for users to work around #3673, and can
    // be removed when that issue is resolved.
    drake::log()->info("Made joint {} a loop joint.", name);
  } else {
    // Update the reference frames of the child link's inertia, visual,
    // and collision elements to be this joint's frame.
    child->ApplyTransformToJointFrame(transform_to_model.inverse() *
                                      transform_child_to_model);

    for (const auto& c : child->get_collision_element_ids()) {
      if (!model->transformCollisionFrame(
              c, transform_to_model.inverse() * transform_child_to_model)) {
        std::stringstream ss;
        ss << std::string(__FILE__) << ": " << __func__
           << ": ERROR: Collision element with ID " << c
           << " not found! Cannot update its local frame to be that of joint.";
        throw std::runtime_error(ss.str());
      }
      // This log statement is required for users to work around #3673, and
      // can be removed when that issue is resolved.
      drake::log()->info("Adding joint {} to the plant.", name);
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
    child->set_parent(parent);
  }
}

// Parses a model and adds it to the rigid body tree. Note that the
// `ModelInstanceIdTable` is an output parameter rather than a return value
// since this method may be called multiple times, once for each model in an
// SDF. Each time this method is called, a new entry is added to the table.
// It's guaranteed that there will be no model name collisions since the SDF
// standard enforces a rule that each model within a single SDF description
// be uniquely named. Regardless, in an abundance of caution, this method
// includes code that checks for collisions and throws a `std::runtime_error` if
// such a collision occurs. This is useful for failing gracefully when provided
// a malformed SDF.
//
// @param[out] tree A pointer to the rigid body tree to which to add the model.
//
// @param[in] node The XML node containing the model information.
//
// @param[in] package_map A map containing information about the ROS workspace
// in which to search for meshes.
//
// @param[in] root_dir The root directory from which to search for mesh files.
//
// @param[in] floating_base_type The type of floating joint to use to weld the
// newly added model to the rigid body tree.
//
// @param[in] weld_to_frame Specifies the initial pose of the newly added robot
// relative to the link to which the robot is being welded.
//
// @param[out] model_instance_id_table A pointer to a map storing model
// names and their instance IDs. This parameter may not be `nullptr`. A
// `std::runtime_error` is thrown if an instance is created of a model whose
// name is already in this table.
void ParseModel(RigidBodyTree* tree, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame,
                ModelInstanceIdTable* model_instance_id_table) {
  // Aborts if any of the output parameter pointers are invalid.
  DRAKE_DEMAND(tree);
  DRAKE_DEMAND(node);

  // The pose_map is needed because SDF specifies almost everything in the
  // model's coordinate frame.
  PoseMap pose_map;

  if (!node->Attribute("name")) {
    throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                        ": ERROR: The model must have a name attribute.");
  }

  // Obtains the model name and, if model_instance_table exists, ensures no such
  // model exists in the model_instance_id_table. Throws an exception if a model
  // of the same name already exists in the table.
  string model_name = node->Attribute("name");
  if (model_instance_id_table != nullptr &&
      model_instance_id_table->find(model_name) !=
          model_instance_id_table->end()) {
    throw std::runtime_error("Model named \"" + model_name + "\" already "
        "exists in model_instance_id_table.");
  }

  // Obtains and adds a new model instance ID into model_instance_id_table.
  int model_instance_id = tree->add_model_instance();
  if (model_instance_id_table != nullptr) {
    (*model_instance_id_table)[model_name] = model_instance_id;
  }

  // Maintains a list of links that were added to the rigid body tree.
  // This is iterated over by AddFloatingJoint() to determine where to attach
  // floating joints.
  std::vector<int> link_indices;

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link")) {
    int index;
    if (ParseSdfLink(tree, model_name, link_node, package_map,
                     pose_map, root_dir, &index, model_instance_id)) {
      link_indices.push_back(index);
    }
  }

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    ParseSdfJoint(tree, model_name, joint_node, pose_map, model_instance_id);
  }

  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame")) {
    ParseSdfFrame(tree, frame_node, model_instance_id);
  }

  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    // Sets a default value for weld_to_frame if none was set.
    // By default, the robot is welded to the world frame.
    if (weld_to_frame == nullptr) {
      weld_to_frame = std::allocate_shared<RigidBodyFrame>(
          Eigen::aligned_allocator<RigidBodyFrame>(),
          std::string(RigidBodyTree::kWorldName),
          nullptr,  // Valid since the robot is attached to the world.
          Eigen::Isometry3d::Identity());
    }

    // Obtains the transform from the frame of the model's root link to the
    // frame of the model's world.
    Isometry3d transform_model_root_to_model_world = Isometry3d::Identity();
    poseValueToTransform(pose, pose_map, transform_model_root_to_model_world);

    // Implements dual-offset: one from model root to model world, another
    // from model world to Drake's world.
    weld_to_frame->set_transform_to_body(
        weld_to_frame->get_transform_to_body() *
            transform_model_root_to_model_world);
  }

  // Adds the floating joint that connects the newly added robot model to the
  // rest of the rigid body tree.
  AddFloatingJoint(floating_base_type, link_indices, weld_to_frame,
                   &pose_map, tree);
}

void ParseWorld(RigidBodyTree* model, XMLElement* node,
                const PackageMap& package_map, const string& root_dir,
                const FloatingBaseType floating_base_type,
                std::shared_ptr<RigidBodyFrame> weld_to_frame,
                ModelInstanceIdTable* model_instance_id_table) {
  for (XMLElement* model_node = node->FirstChildElement("model"); model_node;
       model_node = model_node->NextSiblingElement("model")) {
    ParseModel(model, model_node, package_map, root_dir, floating_base_type,
               weld_to_frame, model_instance_id_table);
  }
}

ModelInstanceIdTable ParseSdf(
    RigidBodyTree* model, XMLDocument* xml_doc,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    PackageMap& package_map,
    const string& root_dir,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  populatePackageMap(package_map);

  XMLElement* node = xml_doc->FirstChildElement("sdf");
  if (!node) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ +
        ": ERROR: The XML file does not contain an sdf tag.");
  }

  ModelInstanceIdTable model_instance_id_table;

  // Loads the world if it is defined.
  XMLElement* world_node =
      node->FirstChildElement(RigidBodyTree::kWorldName);
  if (world_node) {
    // If we have more than one world, it is ambiguous which one the user
    // wishes to use.
    if (world_node->NextSiblingElement(RigidBodyTree::kWorldName)) {
      throw runtime_error(std::string(__FILE__) + ": " + __func__ +
                          ": ERROR: Multiple worlds in one file.");
    }
    ParseWorld(model, world_node, package_map, root_dir, floating_base_type,
               weld_to_frame, &model_instance_id_table);
  }

  // Load all models not in a world.
  for (XMLElement* model_node = node->FirstChildElement("model"); model_node;
       model_node = model_node->NextSiblingElement("model")) {
    ParseModel(model, model_node, package_map, root_dir, floating_base_type,
               weld_to_frame, &model_instance_id_table);
  }

  model->compile();

  return model_instance_id_table;
}

}  // namespace

ModelInstanceIdTable AddModelInstancesFromSdfFileInWorldFrame(
    const string& filename,
    const FloatingBaseType floating_base_type,
    RigidBodyTree* tree) {
  // Ensures the output parameter pointers are valid.
  DRAKE_DEMAND(tree);
  return AddModelInstancesFromSdfFile(filename, floating_base_type,
      nullptr /* weld_to_frame */, tree);
}

ModelInstanceIdTable AddModelInstancesFromSdfFile(
    const string& filename,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree) {
  // Ensures the output parameter pointers are valid.
  DRAKE_DEMAND(tree);

  PackageMap package_map;

  XMLDocument xml_doc;
  xml_doc.LoadFile(filename.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error(std::string(__FILE__) + ": " + __func__ +
                             ": ERROR: Failed to parse XML in file " +
                             filename + "\n" + xml_doc.ErrorName() + ".");
  }

  string root_dir = ".";
  size_t found = filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = filename.substr(0, found);
  }

  return ParseSdf(tree, &xml_doc, package_map, root_dir, floating_base_type,
           weld_to_frame);
}

ModelInstanceIdTable AddModelInstancesFromSdfString(
    const string& sdf_string,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree) {
  // Ensures the output parameter pointers are valid.
  DRAKE_DEMAND(tree);

  PackageMap package_map;

  XMLDocument xml_doc;
  xml_doc.Parse(sdf_string.c_str());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error(std::string(__FILE__) + ": " + __func__ +
                             ": ERROR: Failed to parse XML in SDF description" +
                             xml_doc.ErrorName() + ".");
  }

  string root_dir = ".";

  return ParseSdf(tree, &xml_doc, package_map, root_dir, floating_base_type,
      weld_to_frame);
}

}  // namespace sdf
}  // namespace parsers
}  // namespace drake

