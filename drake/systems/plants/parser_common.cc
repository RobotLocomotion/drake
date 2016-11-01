#include "drake/systems/plants/parser_common.h"

#include <string>

#include "drake/systems/plants/joints/DrakeJoint.h"
#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"
#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"
#include "drake/systems/plants/joints/FixedJoint.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace parsers {

using drake::systems::plants::joints::FloatingBaseType;
using drake::systems::plants::joints::kFixed;
using drake::systems::plants::joints::kRollPitchYaw;
using drake::systems::plants::joints::kQuaternion;

int AddFloatingJoint(
    const FloatingBaseType floating_base_type,
    const std::vector<int>& body_indices,
    const std::shared_ptr<RigidBodyFrame> weld_to_frame,
    const PoseMap* pose_map,
    RigidBodyTree* tree) {
  std::string floating_joint_name;
  RigidBody* weld_to_body{nullptr};
  Eigen::Isometry3d transform_to_world;

  if (weld_to_frame == nullptr) {
    // If weld_to_frame is not specified, weld the newly added model(s) to the
    // world with zero offset.
    weld_to_body = tree->bodies[0].get();
    floating_joint_name = "base";
    transform_to_world = Eigen::Isometry3d::Identity();
  } else {
    // If weld_to_frame is specified and the model is being welded to the world,
    // ensure the "body" variable within weld_to_frame is nullptr. Then, only
    // use the transform_to_body variable within weld_to_frame to initialize
    // the robot at the desired location in the world.
    if (weld_to_frame->get_name()
          == std::string(RigidBodyTree::kWorldName)) {
      if (!weld_to_frame->has_as_rigid_body(nullptr)) {
        throw std::runtime_error(
            "RigidBodyTree::AddFloatingJoint: "
            "Attempted to weld robot to the world while specifying a body "
            "link!");
      }
      weld_to_body = tree->bodies[0].get();  // the world's body
      floating_joint_name = "base";
    } else {
      weld_to_body = weld_to_frame->get_mutable_rigid_body();
      floating_joint_name = "weld";
    }
    transform_to_world = weld_to_frame->get_transform_to_body();
  }

  int num_floating_joints_added = 0;

  for (auto i : body_indices) {
    if (tree->bodies[i]->get_parent() == nullptr) {
      // The following code connects the parent-less link to the rigid body tree
      // using a floating joint.
      tree->bodies[i]->set_parent(weld_to_body);

      Eigen::Isometry3d transform_to_model = Eigen::Isometry3d::Identity();
      if (pose_map != nullptr &&
          pose_map->find(tree->bodies[i]->get_name()) != pose_map->end())
        transform_to_model = pose_map->at(tree->bodies[i]->get_name());

      switch (floating_base_type) {
        case kFixed: {
          std::unique_ptr<DrakeJoint> joint(new FixedJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case kRollPitchYaw: {
          std::unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case kQuaternion: {
          std::unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        default:
          throw std::runtime_error("unknown floating base type");
      }
    }
  }

  if (num_floating_joints_added == 0) {
    // TODO(liang.fok) Handle this by disconnecting one of the internal nodes,
    // replacing the disconnected joint with a loop joint, and then connecting
    // the newly freed body (i.e., the body without a joint) to the world.
    throw std::runtime_error(
        "No root bodies found. Every body referenced by the supplied list of "
        "body indices has a joint connecting it to some other body.  This will "
        "result in RigidBodyTree::compile() looping indefinitely. This "
        "scenario currently not supported in Drake.");
  }

  return num_floating_joints_added;
}

}  // namespace parsers
}  // namespace drake
