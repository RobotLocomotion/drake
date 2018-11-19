/* Adapted for Drake from Simbody's MultibodyGraphModeler class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/multibody/graph_modeler/multibody_graph_modeler.h"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/multibody/graph_modeler/multibody_tree_model.h"

using std::cout;
using std::endl;

namespace drake {
namespace multibody {

namespace {
template <typename FlagsType>
void add_if_flag(FlagsType or_flags, FlagsType test_flag, const char* flag_name,
                 std::string* out) {
  if (or_flags & test_flag) {
    if (!out->empty()) *out += "|";
    *out += flag_name;
  }
}
}  // namespace

std::string to_string(BodyFlags flags) {
  std::string out;
  if (flags == 0) {
    out = "default";
  } else {
    add_if_flag(flags, kStaticBody, "static", &out);
    add_if_flag(flags, kMustBeBaseBody, "must_be_base_body", &out);
    add_if_flag(flags, kMustNotBeTerminalBody, "must_be_nonterminal", &out);
  }
  return out;
}

std::string to_string(JointFlags flags) {
  std::string out;
  if (flags == 0) {
    out = "default";
  } else {
    add_if_flag(flags, kMustBeConstraint, "must_be_constraint", &out);
  }
  return out;
}

std::string to_string(JointTypeFlags flags) {
  std::string out;
  if (flags == 0) {
    out = "default";
  } else {
    add_if_flag(flags, kOkToUseAsJointConstraint, "ok_to_use_as_loop_joint",
                &out);
  }
  return out;
}

//------------------------------------------------------------------------------
//                              CONSTRUCTOR
//------------------------------------------------------------------------------
MultibodyGraphModeler::MultibodyGraphModeler(bool use_drake_defaults)
    : weld_type_name_("weld"), free_type_name_("free") {
  Clear(use_drake_defaults);
}

//------------------------------------------------------------------------------
//                                 CLEAR
//------------------------------------------------------------------------------
void MultibodyGraphModeler::Clear(bool use_drake_defaults) {
  ClearContainers();
  RegisterJointType(weld_type_name_, 0, 0);
  RegisterJointType(free_type_name_, 7, 6);

  if (use_drake_defaults) {
    AddBody("world", ModelInstanceIndex(0), kStaticBody);
    // These are the names used in .sdf files.
    RegisterJointType("revolute", 1, 1);
    RegisterJointType("prismatic", 1, 1);
    // TODO(sherm1) Should allow for a loop ball joint as soon as one is
    // available (change flag to kOkToUseAsJointConstraint).
    RegisterJointType("ball", 4, 3, kDefaultJointTypeFlags);
    RegisterJointType("fixed", 0, 0);  // .sdf for "weld".

    // "weld" and "free" are always present.
  }
}

//------------------------------------------------------------------------------
//                            WORLD BODY NAME
//------------------------------------------------------------------------------
const std::string& MultibodyGraphModeler::world_body_name() const {
  if (bodies_.empty())
    throw std::logic_error(
        "get_world_body_name(): you can't call this until you have called "
        "AddBody() at least once -- the first body is World.");
  return bodies_[0].name();
}

//------------------------------------------------------------------------------
//                          REGISTER JOINT TYPE
//------------------------------------------------------------------------------
auto MultibodyGraphModeler::RegisterJointType(const std::string& name,
                                              int num_q, int num_v,
                                              JointTypeFlags flags,
                                              void* user_ref)
    -> JointTypeIndex {
  if (!(0 <= num_q && num_q <= 7 && 0 <= num_v && num_v <= 6 && num_v <= num_q))
    throw std::runtime_error(
        fmt::format("RegisterJointType(): Illegal joint state specification "
                    "nq={}, nv={} for joint type '{}'",
                    num_q, num_v, name));

  // Reject duplicate type name.
  std::map<std::string, JointTypeIndex>::const_iterator p =
      joint_type_name_to_num_.find(name);
  if (p != joint_type_name_to_num_.end())
    throw std::runtime_error(
        fmt::format("RegisterJointType(): Duplicate joint type '{}'.", name));

  const JointTypeIndex joint_type_num(num_joint_types());  // next available
  joint_type_name_to_num_[name] = joint_type_num;  // provide fast name lookup

  // Can't use emplace_back because constructor is private.
  joint_types_.push_back(JointType(name, num_q, num_v, flags, user_ref));

  return joint_type_num;
}

//------------------------------------------------------------------------------
//                                 ADD BODY
//------------------------------------------------------------------------------
void MultibodyGraphModeler::AddBody(std::string body_name,
                                    ModelInstanceIndex model_instance,
                                    BodyFlags flags, void* user_ref) {
  DRAKE_DEMAND(!body_name.empty() && model_instance.is_valid());
  const BodyIndex body_num(num_bodies());  // next available

  // If this is the first body it is going to be World, which has its own
  // model instance. If the supplied model instance is "default", we'll
  // substitute the world instance. Otherwise we'll complain.
  if (body_num == world_body_num()) {
    if (model_instance == default_model_instance())
      model_instance = world_model_instance();
    if (model_instance != world_model_instance()) {
      throw std::runtime_error(fmt::format(
          "AddBody({},{}): World body must be in the World model instance.",
          body_name, model_instance));
    }

    // Make flags suitable for World also.
    flags = kStaticBody;
  }

  // Creates an empty map if we haven't seen this name before.
  InstanceBodyIndexMap& all_body_nums = body_name_to_num_[body_name];
  auto p = all_body_nums.find(model_instance);

  // Reject duplicate body name.
  if (p != all_body_nums.end()) {
    throw std::logic_error(fmt::format(
        "AddBody({},{}): Name already present in this model instance.",
        body_name, model_instance));
  }

  all_body_nums[model_instance] = body_num;
  // Can't use emplace_back here since constructor is private.
  bodies_.push_back(Body(body_name, model_instance, flags, user_ref));
}

//------------------------------------------------------------------------------
//                             CHANGE BODY FLAGS
//------------------------------------------------------------------------------
void MultibodyGraphModeler::ChangeBodyFlags(BodyIndex body_num,
                                            BodyFlags flags) {
  Body& body = get_mutable_body(body_num);
  body.set_flags(flags);
}

//------------------------------------------------------------------------------
//                                 DELETE BODY
//------------------------------------------------------------------------------
bool MultibodyGraphModeler::DeleteBody(BodyIndex body_num) {
  const ModelInstanceIndex model_instance = get_body(body_num).model_instance();
  DRAKE_DEMAND(model_instance.is_valid());

  // Remove joints for which this is the parent body.
  const std::vector<JointIndex>& joints_as_parent =
      get_mutable_body(body_num).joints_as_parent();
  while (joints_as_parent.size() > 0) DeleteJoint(joints_as_parent[0]);

  // Remove joints for which this is the child body.
  const std::vector<JointIndex>& joints_as_child =
      get_mutable_body(body_num).joints_as_child();
  while (joints_as_child.size() > 0) DeleteJoint(joints_as_child[0]);

  // Remove the body itself. This implicitly renumbers the bodies after this
  // one, because we expect the BodyIndex to be an index into bodies_.
  bodies_.erase(bodies_.begin() + body_num);

  // Update body indexes in the remaining joints due to the deletion
  // of this body.
  for (JointIndex i(0); i < num_joints(); ++i) {
    if (joints_[i].parent_body_num() > body_num)
      get_mutable_joint(i).set_parent_body_num(
          BodyIndex(get_joint(i).parent_body_num() - 1));
    if (joints_[i].child_body_num() > body_num)
      get_mutable_joint(i).set_child_body_num(
          BodyIndex(get_joint(i).child_body_num() - 1));
  }

  // Rebuild the body name-to-body number map to reflect the new numbering.
  body_name_to_num_.clear();
  for (BodyIndex i(0); i < num_bodies(); ++i) {
    const Body& body = get_body(i);
    body_name_to_num_[body.name()][body.model_instance()] = i;
  }

  return true;
}

//------------------------------------------------------------------------------
//                                ADD JOINT
//------------------------------------------------------------------------------
void MultibodyGraphModeler::AddJoint(const std::string& joint_name,
                                     ModelInstanceIndex model_instance,
                                     const std::string& type,
                                     BodyIndex parent_body_num,
                                     BodyIndex child_body_num, JointFlags flags,
                                     void* user_ref) {
  // Reject duplicate joint name, unrecognized type or body names.
  DRAKE_DEMAND(!joint_name.empty() && model_instance.is_valid());
  DRAKE_DEMAND(parent_body_num.is_valid() && child_body_num.is_valid());

  InstanceJointIndexMap& all_joint_nums = joint_name_to_num_[joint_name];
  auto p = all_joint_nums.find(model_instance);

  // Reject duplicate joint name.
  if (p != all_joint_nums.end()) {
    throw std::logic_error(fmt::format(
        "AddJoint({},{}): Name already present in this model instance.",
        joint_name, model_instance));
  }

  const JointTypeIndex type_num = FindJointTypeNum(type);
  if (!type_num.is_valid()) {
    throw std::logic_error("AddJoint(): Joint " + joint_name +
                           " had unrecognized joint type '" + type + "'");
  }

  if (!has_body(parent_body_num)) {
    throw std::logic_error(
        fmt::format("AddJoint({},{}): Invalid parent body {}."));
  }

  if (!has_body(child_body_num)) {
    throw std::logic_error(
        fmt::format("AddJoint({},{}): Invalid child body {}."));
  }

  const JointIndex joint_num(num_joints());  // next available
  all_joint_nums[model_instance] = joint_num;

  // Can't use emplace_back because the constructor is private.
  joints_.push_back(Joint(joint_name, model_instance, type_num, parent_body_num,
                          child_body_num, flags, user_ref));

  get_mutable_body(parent_body_num).add_joint_as_parent(joint_num);
  get_mutable_body(child_body_num).add_joint_as_child(joint_num);
}

//------------------------------------------------------------------------------
//                                DELETE JOINT
//------------------------------------------------------------------------------
bool MultibodyGraphModeler::DeleteJoint(JointIndex joint_num) {
  if (!has_joint(joint_num)) return false;

  const ModelInstanceIndex model_instance =
      get_joint(joint_num).model_instance();
  DRAKE_DEMAND(model_instance.is_valid());

  std::vector<JointIndex>& joints_as_parent =
      get_mutable_body(joints_[joint_num].parent_body_num())
          .mutable_joints_as_parent();
  std::vector<JointIndex>::iterator it =
      std::find(joints_as_parent.begin(), joints_as_parent.end(), joint_num);

  // Data structures are corrupted if this joint is not found in
  // joints_as_parent of parent body.
  DRAKE_DEMAND(it != joints_as_parent.end());
  joints_as_parent.erase(it);

  std::vector<JointIndex>& joints_as_child =
      get_mutable_body(joints_[joint_num].child_body_num())
          .mutable_joints_as_child();
  it = std::find(joints_as_child.begin(), joints_as_child.end(), joint_num);

  // Data structures are corrupted if this joint is not found in
  // joints_as_child of child body.
  DRAKE_DEMAND(it != joints_as_child.end());
  joints_as_child.erase(it);

  // Remove the joint itself. This implicitly renumbers the joints after this
  // one, because we expect the JointIndex to be an index into joints_.
  joints_.erase(joints_.begin() + joint_num);

  // Update indices due to the deletion of this joint
  for (BodyIndex i(0); i < num_bodies(); ++i) {
    auto& children = get_mutable_body(i).mutable_joints_as_parent();
    for (auto& child : children) {
      if (child > joint_num) --child;
    }

    auto& parents = get_mutable_body(i).mutable_joints_as_child();
    for (auto& parent : parents) {
      if (parent > joint_num) --parent;
    }
  }

  // Rebuild the joint name-to-joint number map to reflect the new numbering.
  joint_name_to_num_.clear();
  for (JointIndex i(0); i < num_joints(); ++i) {
    const Joint& joint = get_joint(i);
    joint_name_to_num_[joint.name()][joint.model_instance()] = i;
  }

  return true;
}

//------------------------------------------------------------------------------
//                               DUMP INPUT
//------------------------------------------------------------------------------
void MultibodyGraphModeler::DumpInput(std::ostream& o) const {
  o << "\nMULTIBODY GRAPH MODELER INPUT\n";
  o << "-----------------------------\n";
  o << "\n" << num_bodies() << " BODIES:\n";
  for (BodyIndex i(0); i < num_bodies(); ++i) {
    const Body& body = get_body(i);
    const std::string out =
        fmt::format("{}: {} flags={}", i, body.name(), to_string(body.flags()));
    o << out;
    o << "\n    joints_as_parent=[";
    for (JointIndex j(0); j < body.num_children(); ++j)
      o << " " << body.joints_as_parent()[j];
    o << "]\t  joints_as_child=[";
    for (JointIndex j(0); j < body.num_parents(); ++j)
      o << " " << body.joints_as_child()[j];
    o << "]\n";
  }

  o << "\n" << num_joints() << " JOINTS:\n";
  for (JointIndex i(0); i < num_joints(); ++i) {
    const Joint& joint = get_joint(i);
    const std::string out =
        fmt::format("{}: {} {}->{} {} flags={}\n", i, joint.name(),
                    get_body(joint.parent_body_num()).name(),
                    get_body(joint.child_body_num()).name(),
                    get_joint_type(joint.joint_type_num()).name(),
                    to_string(joint.flags()));
    o << out;
  }

  o << "\n---------- END OF MULTIBODY GRAPH MODELER INPUT.\n\n";
}

//------------------------------------------------------------------------------
//                        GET BODY NUM FROM NAME
//------------------------------------------------------------------------------
auto MultibodyGraphModeler::GetAllBodyIndexFromName(
    const std::string& body_name) const -> const InstanceBodyIndexMap* {
  const auto body_nums = body_name_to_num_.find(body_name);
  if (body_nums == body_name_to_num_.end()) return nullptr;
  const auto& instance_map = body_nums->second;
  DRAKE_DEMAND(!instance_map.empty());
  return &instance_map;
}

auto MultibodyGraphModeler::GetBodyIndexFromNameAndInstance(
    const std::string& body_name, ModelInstanceIndex model_instance,
    const char* func) const -> BodyIndex {
  DRAKE_DEMAND(model_instance.is_valid());

  if (auto instance_map = GetAllBodyIndexFromName(body_name)) {
    const auto entry = instance_map->find(model_instance);
    if (entry != instance_map->end()) {
      DRAKE_DEMAND(entry->second.is_valid());
      return entry->second;
    }
  }

  throw std::logic_error(
      fmt::format("{}: body '{}' not found in model_instance {}.", func,
                  body_name, model_instance));
}

auto MultibodyGraphModeler::GetUniqueBodyIndexFromName(
    const std::string& body_name, const char* func) const -> BodyIndex {
  auto instance_map = GetAllBodyIndexFromName(body_name);

  if (instance_map == nullptr) {
    throw std::logic_error(fmt::format(
        "{}: body '{}' not found in any model_instance.", func, body_name));
  }

  if (instance_map->size() == 1) {
    const BodyIndex body_num = instance_map->cbegin()->second;
    DRAKE_DEMAND(body_num.is_valid());
    return body_num;
  }

  throw std::logic_error(
      fmt::format("{}: body name '{}' not unique. It appears in {} "
                  "model instances. Provide a model instance to "
                  "disambiguate.",
                  func, body_name, instance_map->size()));
}

//------------------------------------------------------------------------------
//                        GET JOINT INDEX FROM NAME
//------------------------------------------------------------------------------
auto MultibodyGraphModeler::GetAllJointIndexFromName(
    const std::string& joint_name) const -> const InstanceJointIndexMap* {
  const auto joint_nums = joint_name_to_num_.find(joint_name);
  if (joint_nums == joint_name_to_num_.end()) return nullptr;
  const auto& instance_map = joint_nums->second;
  DRAKE_DEMAND(!instance_map.empty());
  return &instance_map;
}

auto MultibodyGraphModeler::GetJointIndexFromNameAndInstance(
    const std::string& joint_name, ModelInstanceIndex model_instance,
    const char* func) const -> JointIndex {
  DRAKE_DEMAND(model_instance.is_valid());

  if (auto instance_map = GetAllJointIndexFromName(joint_name)) {
    const auto entry = instance_map->find(model_instance);
    if (entry != instance_map->end()) {
      DRAKE_DEMAND(entry->second.is_valid());
      return entry->second;
    }
  }

  throw std::logic_error(
      fmt::format("{}: joint '{}' not found in model_instance {}.", func,
                  joint_name, model_instance));
}

auto MultibodyGraphModeler::GetUniqueJointIndexFromName(
    const std::string& joint_name, const char* func) const -> JointIndex {
  auto instance_map = GetAllJointIndexFromName(joint_name);

  if (instance_map && instance_map->size() == 1) {
    const JointIndex joint_num = instance_map->cbegin()->second;
    DRAKE_DEMAND(joint_num.is_valid());
    return joint_num;
  }

  throw std::logic_error(
      fmt::format("{}: joint name '{}' not unique. It appears in {} "
                  "model instances. Provide a model instance to "
                  "disambiguate.",
                  func, joint_name, instance_map->size()));
}

//------------------------------------------------------------------------------
//                         BODIES ARE CONNECTED
//------------------------------------------------------------------------------
// Return true if there is a joint between two bodies given by body number,
// regardless of parent/child ordering.
bool MultibodyGraphModeler::BodiesAreConnected(BodyIndex body1_num,
                                               BodyIndex body2_num) const {
  const Body& body1 = get_body(body1_num);
  for (JointIndex joint_num : body1.joints_as_parent()) {
    if (get_joint(joint_num).child_body_num() == body2_num) return true;
  }
  for (JointIndex joint_num : body1.joints_as_child()) {
    if (get_joint(joint_num).parent_body_num() == body2_num) return true;
  }
  return false;
}

}  // namespace multibody
}  // namespace drake
