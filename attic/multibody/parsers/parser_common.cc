#include "drake/multibody/parsers/parser_common.h"

#include <string>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/parsers/xml_util.h"

using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace parsers {

using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kExperimentalMultibodyPlantStyle;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;
using drake::systems::CompliantMaterial;
using tinyxml2::XMLElement;

const char* const FloatingJointConstants::kFloatingJointName = "base";
const char* const FloatingJointConstants::kWeldJointName = "weld";

int AddFloatingJoint(
    const FloatingBaseType floating_base_type,
    const vector<int>& body_indices,
    const std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    const PoseMap* pose_map,
    RigidBodyTree<double>* tree) {
  string floating_joint_name;
  RigidBody<double>* weld_to_body{nullptr};
  Eigen::Isometry3d transform_to_world;

  if (weld_to_frame == nullptr) {
    // If weld_to_frame is not specified, weld the newly added model(s) to the
    // world with zero offset.
    weld_to_body = tree->get_bodies()[0].get();
    floating_joint_name = FloatingJointConstants::kFloatingJointName;
    transform_to_world = Eigen::Isometry3d::Identity();
  } else {
    // If weld_to_frame is specified and the model is being welded to the world,
    // ensure the "body" variable within weld_to_frame is nullptr. Then, only
    // use the transform_to_body variable within weld_to_frame to initialize
    // the robot at the desired location in the world.
    if (weld_to_frame->get_name() ==
        string(RigidBodyTreeConstants::kWorldName)) {
      // N.B. Because of the design of `RigidBodyTree`, the world can either be
      // indicated by the unique world body of tree or a `nullptr`. We should
      // check both of these.
      if (!weld_to_frame->has_as_rigid_body(nullptr) &&
          !weld_to_frame->has_as_rigid_body(&tree->world())) {
        throw runtime_error(
            "AddFloatingJoint: Attempted to weld robot to the world while "
            "specifying a non-world body link!");
      }
      weld_to_body = tree->get_bodies()[0].get();  // the world's body
      floating_joint_name = FloatingJointConstants::kFloatingJointName;
    } else {
      weld_to_body = weld_to_frame->get_mutable_rigid_body();
      floating_joint_name = FloatingJointConstants::kWeldJointName;
    }
    transform_to_world = weld_to_frame->get_transform_to_body();
  }

  int num_floating_joints_added = 0;

  for (auto i : body_indices) {
    if (tree->get_bodies()[i]->get_parent() == nullptr) {
      // The following code connects the parent-less link to the rigid body tree
      // using a floating joint.
      tree->get_bodies()[i]->set_parent(weld_to_body);

      Eigen::Isometry3d transform_to_model = Eigen::Isometry3d::Identity();
      if (pose_map != nullptr &&
          pose_map->find(tree->get_bodies()[i]->get_name()) != pose_map->end())
        transform_to_model = pose_map->at(tree->get_bodies()[i]->get_name());

      switch (floating_base_type) {
        case kFixed: {
          unique_ptr<DrakeJoint> joint(new FixedJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->get_bodies()[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case kRollPitchYaw: {
          unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->get_bodies()[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case kQuaternion:
        case kExperimentalMultibodyPlantStyle: {
          unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->get_bodies()[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        default:
          throw runtime_error("Unknown floating base type.");
      }
    }
  }

  if (floating_base_type != kExperimentalMultibodyPlantStyle &&
      num_floating_joints_added == 0) {
    // TODO(liang.fok) Handle this by disconnecting one of the internal nodes,
    // replacing the disconnected joint with a loop joint, and then connecting
    // the newly freed body (i.e., the body without a joint) to the world.
    throw runtime_error(
        "No root bodies found. Every body referenced by the supplied list of "
        "body indices has a joint connecting it to some other body.  This will "
        "result in RigidBodyTree::compile() looping indefinitely. This "
        "scenario currently not supported in Drake.");
  }

  return num_floating_joints_added;
}

CompliantMaterial ParseCollisionCompliance(XMLElement* node) {
  CompliantMaterial material;
  // TODO(SeanCurtis-TRI): Incomplete validation. This parsing allows redundant
  // declarations of material. Confirm proper validation via an XSD.
  XMLElement* compliant_node = node->FirstChildElement("drake_compliance");
  if (compliant_node) {
    double static_friction{-1};
    double dynamic_friction{-1};
    // Encode the friction values read via bit-masking; 0 = none, 1 = static,
    // 2 = dynamic, 3 = both.
    int friction_values_read = 0;
    for (XMLElement* child = compliant_node->FirstChildElement(); child;
         child = child->NextSiblingElement()) {
      const std::string name(child->Value());
      if (!child->FirstChild()) {
        throw std::runtime_error("Compliant parameter specified (\"" + name +
                                 "\") without any numerical value");
      }
      const double value = StringToDouble(child->FirstChild()->Value());
      if (name == "youngs_modulus") {
        material.set_youngs_modulus(value);
      } else if (name == "dissipation") {
        material.set_dissipation(value);
      } else if (name == "static_friction") {
        static_friction = value;
        friction_values_read |= 1;
      } else if (name == "dynamic_friction") {
        dynamic_friction = value;
        friction_values_read |= 2;
      } else {
        drake::log()->warn("Unrecognized element in drake_compliance: '{}'",
                           name);
      }
    }
    // Handle friction
    if (friction_values_read) {
      if (friction_values_read < 3) {
        throw std::runtime_error(
            "When specifying coefficient of friction, "
                "both static and dynamic coefficients must be defined");
      }
      material.set_friction(static_friction, dynamic_friction);
    }
  }
  return material;
}

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
              "provides a tag specifying a group to ignore without specifying "
              "the \"collision_filter_group\" attribute.");
    }
    tree->AddCollisionFilterIgnoreTarget(group_name, target_name);
  }
}

}  // namespace parsers
}  // namespace drake
