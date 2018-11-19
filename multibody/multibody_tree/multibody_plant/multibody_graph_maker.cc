/* Adapted for Drake from Simbody's MultibodyGraphMaker class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/multibody/multibody_tree/multibody_plant/multibody_graph_maker.h"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"

using std::cout;
using std::endl;

namespace drake {
namespace multibody {
namespace multibody_plant {

//------------------------------------------------------------------------------
//                              CONSTRUCTOR
//------------------------------------------------------------------------------
MultibodyGraphMaker::MultibodyGraphMaker(bool use_drake_defaults)
    : weld_type_name_("weld"), free_type_name_("free") {
  Clear(use_drake_defaults);
}

//------------------------------------------------------------------------------
//                                 CLEAR
//------------------------------------------------------------------------------
void MultibodyGraphMaker::Clear(bool use_drake_defaults) {
  ClearContainers();
  joint_types_.push_back(JointType(weld_type_name_, 0, true, nullptr));
  joint_types_.push_back(JointType(free_type_name_, 6, true, nullptr));
  joint_type_name_to_num_[weld_type_name_] = 0;
  joint_type_name_to_num_[free_type_name_] = 1;

  if (use_drake_defaults) {
    AddLink("world", 0, false);
    // These are the names used in .sdf files.
    RegisterJointType("revolute", 1);
    RegisterJointType("prismatic", 1);
    // TODO(sherm1) Should allow for a loop ball joint as soon as one is
    // available (change false to true here).
    RegisterJointType("ball", 3, false);
    RegisterJointType("fixed", 0);  // .sdf for "weld".

    // "weld" and "free" are always present.
  }
}

//------------------------------------------------------------------------------
//                           GET WORLD BODY NAME
//------------------------------------------------------------------------------
const std::string& MultibodyGraphMaker::get_world_body_name() const {
  if (bodies_.empty())
    throw std::logic_error(
        "get_world_body_name(): you can't call this until you have called "
        "AddLink() at least once -- the first body is World.");
  return bodies_[0].name();
}

//------------------------------------------------------------------------------
//                          REGISTER JOINT TYPE
//------------------------------------------------------------------------------
int MultibodyGraphMaker::RegisterJointType(const std::string& name,
                                           int num_mobilities,
                                           bool have_good_loop_joint_available,
                                           void* user_ref) {
  if (!(0 <= num_mobilities && num_mobilities <= 6))
    throw std::runtime_error(
        "RegisterJointType(): Illegal number of mobilities for joint type '" +
        name + "'");

  // Reject duplicate type name, and reserved type names.
  if (name == get_weld_joint_type_name() || name == get_free_joint_type_name())
    throw std::runtime_error(
        "RegisterJointType(): Joint type '" + name +
        "' is reserved (you can change the reserved names).");
  std::map<std::string, int>::const_iterator p =
      joint_type_name_to_num_.find(name);
  if (p != joint_type_name_to_num_.end())
    throw std::runtime_error("RegisterJointType(): Duplicate joint type '" +
                             name + "'");

  const int jointTypeNum =
      static_cast<int>(joint_types_.size());     // next available
  joint_type_name_to_num_[name] = jointTypeNum;  // provide fast name lookup

  joint_types_.push_back(JointType(name, num_mobilities,
                                   have_good_loop_joint_available, user_ref));

  return jointTypeNum;
}

//------------------------------------------------------------------------------
//                                 ADD LINK
//------------------------------------------------------------------------------
void MultibodyGraphMaker::AddLink(const std::string& name, double mass,
                                  bool must_be_base_body, void* user_ref) {
  if (mass < 0) {
    throw std::invalid_argument("AddLink(): Link '" + name +
        "' specified negative mass");
  }

  // Make sure we'll regenerate the graph.
  ClearGraph();

  // Reject duplicate body name.
  std::map<std::string, int>::const_iterator p = body_name_to_num_.find(name);
  if (p != body_name_to_num_.end())
    throw std::runtime_error("AddLink(): Duplicate link name '" + name + "'");

  const int bodyNum = static_cast<int>(bodies_.size());  // next available slot
  body_name_to_num_[name] = bodyNum;  // provide fast name lookup

  if (bodyNum == 0) {  // First body is World; use only the name and ref.
    Body world(name, std::numeric_limits<double>::infinity(), false, user_ref);
    world.set_level(0);  // already in tree
    bodies_.push_back(world);
  } else {  // This is a real body.
    bodies_.push_back(Body(name, mass, must_be_base_body, user_ref));
  }
}

void MultibodyGraphMaker::ChangeLinkMass(const std::string& name,
                                         double new_mass) {
  if (new_mass < 0) {
    throw std::invalid_argument("AddLink(): Link '" + name +
                                "' specified negative mass");
  }
  const int body_num = FindBodyNum(name);
  if (body_num == -1) {
    throw std::logic_error("ChangeLinkMass(): link '" + name + "' not found.");
  }

  Body& body = get_mutable_body(body_num);
  body.set_mass(new_mass);
  ClearGraph();
}

//------------------------------------------------------------------------------
//                                 DELETE LINK
//------------------------------------------------------------------------------
bool MultibodyGraphMaker::DeleteLink(const std::string& name) {
  // Reject non-existing link name.
  std::map<std::string, int>::iterator p = body_name_to_num_.find(name);
  if (p == body_name_to_num_.end()) return false;

  const int body_num = p->second;

  const std::vector<int>& joints_as_parent =
      get_mutable_body(body_num).joints_as_parent();
  while (joints_as_parent.size() > 0)
    DeleteJoint(joints_[joints_as_parent[0]].name());

  const std::vector<int>& joints_as_child =
      get_mutable_body(body_num).joints_as_child();
  while (joints_as_child.size() > 0)
    DeleteJoint(joints_[joints_as_child[0]].name());

  bodies_.erase(bodies_.begin() + body_num);
  body_name_to_num_.erase(p);

  // Update body indexes due to the deletion of this body
  for (int i = 0; i < num_joints(); ++i) {
    if (joints_[i].parent_body_num() > body_num)
      get_mutable_joint(i).set_parent_body_num(get_joint(i).parent_body_num() -
                                               1);
    if (joints_[i].child_body_num() > body_num)
      get_mutable_joint(i).set_child_body_num(get_joint(i).child_body_num() -
                                              1);
  }

  for (int i = body_num; i < num_bodies(); ++i)
    body_name_to_num_[bodies_[i].name()] = i;

  ClearGraph();
  return true;
}

//------------------------------------------------------------------------------
//                                ADD JOINT
//------------------------------------------------------------------------------
void MultibodyGraphMaker::AddJoint(const std::string& name,
                                   const std::string& type,
                                   const std::string& parent_body_name,
                                   const std::string& child_body_name,
                                   bool must_be_loop_joint, void* userRef) {
  ClearGraph();  // Must regenerate after deletion.

  // Reject duplicate joint name, unrecognized type or body names.
  std::map<std::string, int>::const_iterator p = joint_name_to_num_.find(name);
  if (p != joint_name_to_num_.end())
    throw std::runtime_error("AddJoint(): Duplicate joint name '" + name + "'");

  const int type_num = GetJointTypeNum(type);
  if (type_num < 0)
    throw std::runtime_error("AddJoint(): Joint " + name +
                             " had unrecognized joint type '" + type + "'");

  const int parent_body_num = FindBodyNum(parent_body_name);
  if (parent_body_num < 0)
    throw std::runtime_error("AddJoint(): Joint " + name +
                             " had unrecognized parent body '" +
                             parent_body_name + "'");

  const int child_body_num = FindBodyNum(child_body_name);
  if (child_body_num < 0)
    throw std::runtime_error("AddJoint(): Joint " + name +
                             " had unrecognized child body '" +
                             child_body_name + "'");

  const int joint_num = num_joints();    // next available slot
  joint_name_to_num_[name] = joint_num;  // provide fast name lookup

  joints_.push_back(Joint(name, type_num, parent_body_num, child_body_num,
                          must_be_loop_joint, userRef));

  get_mutable_body(parent_body_num).add_joint_as_parent(joint_num);
  get_mutable_body(child_body_num).add_joint_as_child(joint_num);
}

//------------------------------------------------------------------------------
//                                DELETE JOINT
//------------------------------------------------------------------------------
bool MultibodyGraphMaker::DeleteJoint(const std::string& name) {
  std::map<std::string, int>::iterator p = joint_name_to_num_.find(name);
  if (p == joint_name_to_num_.end()) return false;

  const int joint_num = p->second;
  joint_name_to_num_.erase(p);

  std::vector<int>& joints_as_parent =
      get_mutable_body(joints_[joint_num].parent_body_num())
          .mutable_joints_as_parent();
  std::vector<int>::iterator it =
      std::find(joints_as_parent.begin(), joints_as_parent.end(), joint_num);
  if (it == joints_as_parent.end())
    throw std::runtime_error(
        "DeleteJoint(): Joint " + name +
        " doesn't exist in joints_as_parent of parent body ");
  joints_as_parent.erase(it);

  std::vector<int>& joints_as_child =
      get_mutable_body(joints_[joint_num].child_body_num())
          .mutable_joints_as_child();
  it = std::find(joints_as_child.begin(), joints_as_child.end(), joint_num);
  if (it == joints_as_child.end())
    throw std::runtime_error(
        "DeleteJoint(): Joint " + name +
        " doesn't exist in joints_as_child of child body ");
  joints_as_child.erase(it);

  joints_.erase(joints_.begin() + joint_num);

  // Update indices due to the deletion of this joint
  for (int i = 0; i < num_bodies(); ++i) {
    auto& children = get_mutable_body(i).mutable_joints_as_parent();
    for (auto child = children.begin(); child != children.end(); ++child)
      if (*child > joint_num) {
        --(*child);
        joint_name_to_num_[joints_[*child].name()] = *child;
      }

    auto& parents = get_mutable_body(i).mutable_joints_as_child();
    for (auto parent = parents.begin(); parent != parents.end(); ++parent)
      if (*parent > joint_num) {
        --(*parent);
        // No need to adjust joint_name_to_num_ here since the first loop has
        // done it already.
      }
  }

  ClearGraph();  // Must regenerate after deletion.
  return true;
}

//------------------------------------------------------------------------------
//                              GENERATE GRAPH
//------------------------------------------------------------------------------
void MultibodyGraphMaker::GenerateGraph() {
  if (graph_generated_) {
    throw std::logic_error(
        "GenerateGraph(): graph already generated since last change. Call "
        "ClearGraph() first if you want to regenerate.");
  }

  // Body and Joint inputs have been supplied. Body 0 is World.

  // 1. Add free joints to World for any bodies that said they must be base
  // bodies and don't already have a connection to World.
  //
  // While we're at it, look for dangling massless bodies -- they are only
  // allowed if they were originally connected to at least two other bodies
  // by given tree-eligible joints, or if they are connected by a weld
  // joint and thus have no mobility.
  // TODO(sherm1): "must be loop joint" joints shouldn't count towards the
  // required two for massless bodies, but we're not checking.
  for (int body_num = 1; body_num < num_bodies(); ++body_num) {  // skip World
    const Body& body = get_body(body_num);
    if (body.mass() == 0) {
      if (body.num_joints() == 0) {
        throw std::runtime_error(
            "GenerateGraph(): body " + body.name() +
            " is massless but free (no joint). Must be internal or"
            " welded to a massful body.");
      }
      if (body.num_joints() == 1) {
        const int joint_num = body.joints_as_child().empty()
                                  ? body.joints_as_parent()[0]
                                  : body.joints_as_child()[0];
        const Joint& joint = get_joint(joint_num);
        const JointType& joint_type = get_joint_type(joint.joint_type_num());
        if (joint_type.num_mobilities() > 0) {
          throw std::runtime_error(
              "GenerateGraph(): body " + body.name() +
              " is massless but not internal and not welded to"
              " a massful body.");
        }
      }
    }

    // Attach the must-be-base-body bodies to World.
    if (body.must_be_base_body() && !BodiesAreConnected(body_num, 0)) {
      ConnectBodyToWorld(body_num);
    }
  }

  // 2. Repeat until all bodies are in the tree:
  //   - try to build the tree from World outwards
  //   - if incomplete, add one missing connection to World
  // This terminates because we add at least one body to the tree each time.
  while (true) {
    GrowTree();
    const int new_base_body = ChooseNewBaseBody();
    if (new_base_body < 0) break;  // all bodies are in the tree
    // Add joint to World.
    ConnectBodyToWorld(new_base_body);
  }

  // 3. Split the loops
  // This requires adding new "slave" bodies to replace the child bodies in
  // the loop-forming joints. Then each slave is welded back to its master
  // body (the original child).
  BreakLoops();

  graph_generated_ = true;
}

//------------------------------------------------------------------------------
//                              CLEAR GRAPH
//------------------------------------------------------------------------------
void MultibodyGraphMaker::ClearGraph() {
  if (!graph_generated_)
    return;  // Already cleared.

  // Remove any bodies that were added during graph generation. That is just
  // the slave bodies.
  for (int body_num = 1; body_num < num_bodies(); ++body_num) {  // skip World
    if (bodies_[body_num].is_slave()) {
      // Assumption: all slave bodies are clustered at end of the body array.
      bodies_.erase(bodies_.begin() + body_num,
                    bodies_.begin() + bodies_.size());
      break;
    }
    get_mutable_body(body_num).ForgetGraph(&*this);
  }

  // Remove any joints that were added during graph generation.
  for (int joint_num = 0; joint_num < num_joints(); ++joint_num) {
    if (get_mutable_joint(joint_num).ForgetGraph(&*this)) --joint_num;
  }

  mobilizers_.clear();
  constraints_.clear();
  graph_generated_ = false;
}

//------------------------------------------------------------------------------
//                               DUMP GRAPH
//------------------------------------------------------------------------------
void MultibodyGraphMaker::DumpGraph(std::ostream& o) const {
  constexpr int kBufSize = 1024;
  char buf[kBufSize];
  o << "\nMULTIBODY GRAPH (generated="
    << (graph_generated_ ? "true" : "false") << ")\n";
  o << "------------------------------\n";
  o << "\n" << num_bodies() << " BODIES:\n";
  for (int i = 0; i < num_bodies(); ++i) {
    const MultibodyGraphMaker::Body& body = get_body(i);
    snprintf(buf, kBufSize, "%2d %2d: %s mass=%g mob=%d master=%d %s\n", i,
             body.level(), body.name().c_str(), body.mass(),
             body.mobilizer_num(), body.master_body_num(),
             body.must_be_base_body() ? "MUST BE BASE BODY" : "");
    o << buf;
    o << "  joints_as_parent=[";
    for (int j = 0; j < body.num_children(); ++j)
      o << " " << body.joints_as_parent()[j];
    o << "]\t  joints_as_child=[";
    for (int j = 0; j < body.num_parents(); ++j)
      o << " " << body.joints_as_child()[j];
    o << "]\t  slaves=[";
    for (int j = 0; j < body.num_slaves(); ++j) o << " " << body.slaves()[j];
    o << "]\n";
  }

  o << "\n" << num_joints() << " JOINTS:\n";
  for (int i = 0; i < num_joints(); ++i) {
    const MultibodyGraphMaker::Joint& joint = get_joint(i);
    snprintf(buf, kBufSize, "%2d %2d: %20s %20s->%-20s %10s loopc=%2d %s %s\n",
             i, joint.mobilizer_num(), joint.name().c_str(),
             get_body(joint.parent_body_num()).name().c_str(),
             get_body(joint.child_body_num()).name().c_str(),
             get_joint_type(joint.joint_type_num()).name().c_str(),
             joint.loop_constraint_num(),
             joint.must_be_loop_joint() ? "MUST BE LOOP" : "",
             joint.is_added_base_joint() ? "ADDED BASE JOINT" : "");
    o << buf;
  }

  o << "\n" << num_mobilizers() << " MOBILIZERS:\n";
  for (int i = 0; i < num_mobilizers(); ++i) {
    const MultibodyGraphMaker::Mobilizer& mo = get_mobilizer(i);
    const MultibodyGraphMaker::Joint& joint = get_joint(mo.joint_num());
    const MultibodyGraphMaker::Body& inb = get_body(mo.inboard_body_num());
    const MultibodyGraphMaker::Body& outb = get_body(mo.outboard_body_num());
    snprintf(buf, kBufSize, "%2d %2d: %20s %20s->%-20s %10s %2d %3s\n", i,
             mo.level(), joint.name().c_str(), inb.name().c_str(),
             outb.name().c_str(), mo.get_joint_type_name().c_str(),
             mo.joint_num(), (mo.is_reversed_from_joint() ? "REV" : ""));
    o << buf;
  }

  o << "\n" << num_loop_constraints() << " LOOP CONSTRAINTS:\n";
  for (int i = 0; i < num_loop_constraints(); ++i) {
    const MultibodyGraphMaker::LoopConstraint& lc = get_loop_constraint(i);
    const MultibodyGraphMaker::Body parent = get_body(lc.parent_body_num());
    const MultibodyGraphMaker::Body child = get_body(lc.child_body_num());
    snprintf(buf, kBufSize, "%d: %s parent=%s child=%s jointNum=%d\n", i,
             lc.get_constraint_type_name().c_str(), parent.name().c_str(),
             child.name().c_str(), lc.joint_num());
    o << buf;
  }

  std::vector<int> base_bodies = FindBaseBodies();
  o << "\n" << base_bodies.size() << " BASE BODIES:\n";
  for (int body_num : base_bodies) o << " " << get_body(body_num).name();
  o << "\n---------- END OF MULTIBODY GRAPH.\n\n";
}

//------------------------------------------------------------------------------
//                            FIND BASE BODIES
//------------------------------------------------------------------------------
std::vector<int> MultibodyGraphMaker::FindBaseBodies() const {
  std::vector<int> base_bodies;
  for (const auto& mobilizer : mobilizers_) {
    if (mobilizer.level() != 1) continue;
    base_bodies.push_back(mobilizer.outboard_body_num());
  }
  return base_bodies;
}

int MultibodyGraphMaker::FindBaseBody(int body_num) const {
  std::vector<int> path_bodies = FindPathToWorld(body_num);
  return path_bodies.empty() ? -1 : path_bodies.back();
}

//------------------------------------------------------------------------------
//                            FIND PATH TO WORLD
//------------------------------------------------------------------------------
std::vector<int> MultibodyGraphMaker::FindPathToWorld(int body_num) const {
  DRAKE_DEMAND(body_num >= 0);
  std::vector<int> path_bodies;
  // Note that nothing gets pushed if body_num is World.
  while (get_body(body_num).level() > 0) {
    path_bodies.push_back(body_num);
    const Mobilizer& mobilizer =
        get_mobilizer(get_body(body_num).mobilizer_num());
    body_num = mobilizer.inboard_body_num();
  }
  return path_bodies;
}

//------------------------------------------------------------------------------
//                                SPLIT BODY
//------------------------------------------------------------------------------
// Create a new slave body for the given master, and add it to the list of
// bodies. Does not create the related loop constraint. The body
// number assigned to the slave is returned.
int MultibodyGraphMaker::SplitBody(int master_body_num) {
  const int slave_body_num = num_bodies();  // next available
  Body& master = get_mutable_body(master_body_num);
  // First slave is number 1, slave 0 is the master.
  std::stringstream ss;
  ss << "#" << master.name() << "_slave_" << master.num_slaves() + 1;
  Body slave(ss.str(), std::numeric_limits<double>::quiet_NaN(), false,
             nullptr);
  slave.set_master_body_num(master_body_num);
  master.add_slave_body_num(slave_body_num);
  bodies_.push_back(slave);
  // No name lookup for slave bodies.
  return slave_body_num;
}

//------------------------------------------------------------------------------
//                          CHOOSE NEW BASE BODY
//------------------------------------------------------------------------------
// We've tried to build the tree but might not have succeeded in using
// all the bodies. That means we'll have to connect one of them to World.
// Select the best unattached body to use as a base body, or -1 if all bodies
// are already included in the spanning tree. We hope to find a body that is
// serving as a parent but has never been listed as a child; that is crying out
// to be connected directly to World. We'll take the first one of those we
// find to roughly preserve the order of connected graphs in the input.
//
// If there is no parent-only body, we're going to have to pick a child body
// as a base body, meaning that any joint for which it was the child will have
// to use a reversed mobilizer (because a base body is always a parent). Of
// those we pick the one with the most children.
//
// Note that any must-be-base-body bodies get attached to World before anything
// else; that is done at the start of GenerateGraph() rather than here.
int MultibodyGraphMaker::ChooseNewBaseBody() const {
  int best_body = -1;
  int num_children = -1;
  // Skip the World body.
  // TODO(sherm1) This runs through all the bodies each time; if that
  // becomes expensive, start after the last-found base body.
  for (int body_num = 1; body_num < num_bodies(); ++body_num) {
    const Body& body = bodies_[body_num];
    if (body.is_in_tree()) continue;  // Unavailable.

    // If this is a parent-only body, choose it.
    if (body.num_parents() == 0) return body_num;

    // This is a childish body, keep the one with the most children.
    if (body.num_children() > num_children) {
      best_body = body_num;
      num_children = body.num_children();
    }
  }
  return best_body;
}

//------------------------------------------------------------------------------
//                          CONNECT BODY TO WORLD
//------------------------------------------------------------------------------
// Connect the given body to World by a free joint with parent World and
// child the given body. This makes the body a base body (level 1 body)
// for some subtree of the multibody graph.
void MultibodyGraphMaker::ConnectBodyToWorld(int body_num) {
  const Body& body = get_body(body_num);
  DRAKE_ASSERT(!body.is_in_tree());
  const std::string jointName = "#" + get_world_body_name() + "_" + body.name();
  AddJoint(jointName, get_free_joint_type_name(), get_world_body_name(),
           body.name(), false, nullptr);
  GetMutableJointByName(jointName).set_is_added_base_joint(true);
}

//------------------------------------------------------------------------------
//                          ADD MOBILIZER FOR JOINT
//------------------------------------------------------------------------------
// We've already determined this joint is eligible to become a mobilizer; now
// make it so. Given a joint for which either the parent or child is in the
// tree, but not both, create a mobilizer implementing this joint and attaching
// the unattached body (which will be the mobilizer's outboard body) to the
// tree. The mobilizer number is returned and also recorded in the joint
// and outboard body.
int MultibodyGraphMaker::AddMobilizerForJoint(int jointNum) {
  Joint& joint = get_mutable_joint(jointNum);
  DRAKE_DEMAND(!joint.must_be_loop_joint());  // can't be a tree joint
  DRAKE_DEMAND(!joint.has_mobilizer());       // already done
  const int pNum = joint.parent_body_num(), cNum = joint.child_body_num();
  Body& parent = get_mutable_body(pNum);
  Body& child = get_mutable_body(cNum);
  // Exactly one of these must already be in the tree.
  DRAKE_DEMAND(parent.is_in_tree() ^ child.is_in_tree());

  const int mobilizer_num = num_mobilizers();  // next available
  if (parent.is_in_tree()) {
    // Child is outboard body (forward joint)
    child.set_level(parent.level() + 1);
    mobilizers_.push_back(
        Mobilizer(jointNum, child.level(), pNum, cNum, false, this));
    child.set_mobilizer_num(mobilizer_num);
  } else if (child.is_in_tree()) {
    // Parent is outboard body (reverse joint)
    parent.set_level(child.level() + 1);
    mobilizers_.push_back(
        Mobilizer(jointNum, parent.level(), cNum, pNum, true, this));
    parent.set_mobilizer_num(mobilizer_num);
  }
  joint.set_mobilizer_num(mobilizer_num);
  return mobilizer_num;
}

//------------------------------------------------------------------------------
//                   FIND HEAVIEST UNASSIGNED FORWARD JOINT
//------------------------------------------------------------------------------
// Starting with a given body that is in the tree already, look at its
// unassigned children (meaning bodies connected by joints that aren't
// mobilizers yet) and return the joint connecting it to the child with the
// largest mass. (The idea is that this would be a good direction to extend the
// chain.) Return -1 if this body has no children.
int MultibodyGraphMaker::FindHeaviestUnassignedForwardJoint(
    int inboard_body_num) const {
  const Body& inboard_body = get_body(inboard_body_num);
  int joint_num = -1;
  double max_mass = 0;
  // Search joints for which this is the parent body.
  for (int i = 0; i < inboard_body.num_children(); ++i) {
    const int joint_forward = inboard_body.joints_as_parent()[i];
    const Joint& joint = joints_[joint_forward];
    if (joint.has_mobilizer()) continue;       // already in the tree
    if (joint.must_be_loop_joint()) continue;  // can't be a tree joint
    const Body& child = get_body(joint.child_body_num());
    if (child.is_in_tree()) continue;  // this is a loop joint
    if (child.mass() > max_mass) {
      joint_num = joint_forward;
      max_mass = child.mass();
    }
  }
  return joint_num;
}

//------------------------------------------------------------------------------
//                   FIND HEAVIEST UNASSIGNED REVERSE JOINT
//------------------------------------------------------------------------------
// Same as previous method, but now we are looking for unassigned joints where
// the given body serves as the child so we would have to reverse the joint to
// add it to the tree. (This is less desirable so is a fallback.)
int MultibodyGraphMaker::FindHeaviestUnassignedReverseJoint(
    int inboard_body_num) const {
  const Body& inboard_body = get_body(inboard_body_num);
  int joint_num = -1;
  double max_mass = 0;
  // Search joints for which this is the child body.
  for (int i = 0; i < inboard_body.num_parents(); ++i) {
    const int joint_reverse = inboard_body.joints_as_child()[i];
    const Joint& joint = joints_[joint_reverse];
    if (joint.has_mobilizer()) continue;       // already in the tree
    if (joint.must_be_loop_joint()) continue;  // can't be a tree joint
    const Body& parent = get_body(joint.parent_body_num());
    if (parent.is_in_tree()) continue;  // this is a loop joint
    if (parent.mass() > max_mass) {
      joint_num = joint_reverse;
      max_mass = parent.mass();
    }
  }
  return joint_num;
}

//------------------------------------------------------------------------------
//                                 GROW TREE
//------------------------------------------------------------------------------
// Process unused joints for which one body is in the tree (at level h) and the
// other is not. Add the other body to the tree at level h+1, marking the joint
// as forward (other body is child) or reverse (other body is parent). Repeat
// until no changes are made. Does not assign loop joints or any bodies that
// don't have a path to World. Extend the multibody tree starting with the
// lowest-level eligible joints and moving outwards. We're doing this
// breadth-first so that we get roughly even-length chains and we'll stop at
// the first level at which we fail to find any joint. If we fail to consume
// all the bodies, the caller will have to add a level 1 joint to attach a
// previously-disconnected base body to World and then call this again.
//
// We violate the breadth-first heuristic to avoid ending a branch with a
// massless body, unless it is welded to its parent. If we add a mobile massless
// body, we'll keep going out along its branch until we hit a massful body. It
// is a disaster if we fail to find a massful body because a tree that
// terminates in a mobile massless body will generate a singular mass matrix.
// We'll throw an exception in that case, but note that this may just be a
// failure of the heuristic; there may be some tree that could have avoided the
// terminal massless body but we failed to discover it.
//
// TODO(sherm1): keep a list of unprocessed joints so we don't have to go
// through all of them again at each level.
void MultibodyGraphMaker::GrowTree() {
  // Record the joints for which we added mobilizers during this subtree
  // sweep. That way if we jumped levels ahead due to massless bodies we
  // can take credit and keep going rather than quit.
  std::set<int> joints_added;
  for (int level = 1;; ++level) {  // level of outboard (mobilized) body
    bool any_mobilizer_added = false;
    for (int joint_num = 0; joint_num < num_joints(); ++joint_num) {
      // See if this joint is at the right level (meaning its inboard
      // body is at level-1) and is available to become a mobilizer.
      const Joint& joint = get_joint(joint_num);
      if (joint.has_mobilizer()) {
        // Already done -- one of ours?
        if (joints_added.count(joint_num)) {
          // We added it during this GrowTree() call -- is it at
          // the current level?
          const Mobilizer& mobilizer = mobilizers_[joint.mobilizer_num()];
          if (mobilizer.level() == level)
            any_mobilizer_added = true;  // Added one at level.
        }
        continue;
      }
      if (joint.must_be_loop_joint()) continue;  // can't be a tree joint
      const Body& parent = get_body(joint.parent_body_num());
      const Body& child = get_body(joint.child_body_num());
      // Exactly one of parent or child must already be in the tree.
      if (!(parent.is_in_tree() ^ child.is_in_tree())) continue;
      // Stop growing this branch after the current level so we can balance
      // branch lengths. However, if our outboard body is massless we don't
      // want to risk it ending up as terminal so keep going in that case.
      if (parent.is_in_tree()) {
        if (child.mass() > 0 && parent.level() != level - 1)
          continue;  // not time yet
      } else {       // child is in tree
        if (parent.mass() > 0 && child.level() != level - 1)
          continue;  // not time yet
      }
      AddMobilizerForJoint(joint_num);
      joints_added.insert(joint_num);
      any_mobilizer_added = true;

      // We just made joint joint_num a mobilizer. If its outboard body
      // is massless and the mobilizer was not a weld, we need to keep
      // extending this branch of the tree until we can end it with a
      // massful body.
      const Body& outboard = get_body(mobilizers_.back().outboard_body_num());
      const JointType& joint_type = get_joint_type(joint.joint_type_num());
      if (joint_type.num_mobilities() == 0 || outboard.mass() > 0) continue;

      // Pick a further-outboard body with mass and extend branch to it.
      // Prefer forward-direction joints if possible.
      // If only massless outboard bodies are available, add one and
      // keep going.
      while (true) {
        const int body_num = mobilizers_.back().outboard_body_num();
        const int joint_forward = FindHeaviestUnassignedForwardJoint(body_num);
        if (joint_forward >= 0 &&
            get_body(get_joint(joint_forward).child_body_num()).mass() > 0) {
          AddMobilizerForJoint(joint_forward);
          joints_added.insert(joint_forward);
          break;
        }
        const int joint_reverse = FindHeaviestUnassignedReverseJoint(body_num);
        if (joint_reverse >= 0 &&
            get_body(get_joint(joint_reverse).parent_body_num()).mass() > 0) {
          AddMobilizerForJoint(joint_reverse);
          joints_added.insert(joint_reverse);
          break;
        }

        // Couldn't find a massful body to add. Add another massless
        // body (if there is one) and keep trying.
        if (joint_forward >= 0) {
          AddMobilizerForJoint(joint_forward);
          joints_added.insert(joint_forward);
          continue;
        }
        if (joint_reverse >= 0) {
          AddMobilizerForJoint(joint_reverse);
          joints_added.insert(joint_reverse);
          continue;
        }

        // Uh oh. Nothing outboard of the massless body we just added.
        // We're ending this chain with a massless body; not good.
        throw std::runtime_error(
            "GrowTree(): algorithm produced an"
            " invalid tree containing a terminal massless body (" +
            get_body(body_num).name() +
            "). This may be due to an invalid"
            " model or failure of heuristics. In the latter case you"
            " may be able to get a valid tree by forcing a different"
            " loop break or changing parent->child ordering.");
      }
    }
    if (!any_mobilizer_added) break;
  }
}

//------------------------------------------------------------------------------
//                                 BREAK LOOPS
//------------------------------------------------------------------------------
// Find any remaining unused joints, which will have both parent and
// child bodies already in the tree. This includes joints that the user
// told us must be loop joints, and ones picked by the algorithm.
// For each of those, implement the joint with a loop constraint if one
// is provided, otherwise implement it with a mobilizer and split off a
// slave body from the "split body" (normally the child), use that slave as the
// outboard body for the mobilizer, and add a weld constraint to reconnect the
// slave to its master (the original split body). If the child is World, we
// must reverse the mobilizer since World must always be the inboard body. In
// that case the split body is the parent.
void MultibodyGraphMaker::BreakLoops() {
  for (int joint_num = 0; joint_num < num_joints(); ++joint_num) {
    Joint& joint = joints_[joint_num];
    if (joint.has_mobilizer()) continue;  // already done
    const int parent_body_num = joint.parent_body_num();
    const int child_body_num = joint.child_body_num();
    DRAKE_DEMAND(get_body(parent_body_num).is_in_tree() &&
                 get_body(child_body_num).is_in_tree());

    const int loop_constraint_num = num_loop_constraints();  // next available

    // Use a loop joint if there's a good one available.
    const JointType& joint_type = get_joint_type(joint.joint_type_num());
    if (joint_type.have_good_loop_joint_available()) {
      constraints_.push_back(LoopConstraint(
          joint_type.name(), joint_num, parent_body_num, child_body_num, this));
      joint.set_loop_constraint_num(loop_constraint_num);
      continue;
    }

    // No usable loop constraint for this type of joint. Add a new slave
    // body so we can use a mobilizer. (Body references are invalidated here
    // since we're adding a new body -- don't reuse them.)

    // First assume we will be able to split the child, which is the usual case.
    int split_body_num = child_body_num;
    int inboard_body_num = parent_body_num;
    bool is_reversed = false;
    // If the child is World, choose the parent instead, reverse the mobilizer.
    if (get_body(child_body_num).is_world_body()) {
      DRAKE_DEMAND(!get_body(parent_body_num).is_world_body());
      split_body_num = parent_body_num;
      inboard_body_num = child_body_num;  // That is, World.
      is_reversed = true;
    }
    const int slave_body_num = SplitBody(split_body_num);
    Body& inboard_body = get_mutable_body(inboard_body_num);
    Body& slave = get_mutable_body(slave_body_num);

    // Mobilize the slave body.
    const int mobilizer_num = num_mobilizers();  // next available
    const int level = inboard_body.level() + 1;
    joint.set_mobilizer_num(mobilizer_num);
    slave.set_mobilizer_num(mobilizer_num);
    slave.add_joint_as_child(joint_num);
    mobilizers_.push_back(Mobilizer(joint_num, level, inboard_body_num,
                                    slave_body_num, is_reversed, this));
    slave.set_level(level);

    // Add the weld loop constraint.
    constraints_.push_back(
        LoopConstraint(get_weld_joint_type_name(), -1 /* No associated joint */,
                       split_body_num, slave_body_num, this));
  }
}

//------------------------------------------------------------------------------
//                         BODIES ARE CONNECTED
//------------------------------------------------------------------------------
// Return true if there is a joint between two bodies given by body number,
// regardless of parent/child ordering.
bool MultibodyGraphMaker::BodiesAreConnected(int body1_num,
                                             int body2_num) const {
  const Body& body1 = get_body(body1_num);
  for (int i = 0; i < body1.num_children(); ++i) {
    if (get_joint(body1.joints_as_parent()[i]).child_body_num() == body2_num)
      return true;
  }
  for (int i = 0; i < body1.num_parents(); ++i) {
    if (get_joint(body1.joints_as_child()[i]).parent_body_num() == body2_num)
      return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//                          BODY :: FORGET GRAPH
//------------------------------------------------------------------------------
// Restore the Body to its state prior to GenerateGraph().
void MultibodyGraphMaker::Body::ForgetGraph(MultibodyGraphMaker*) {
  level_ = -1;
  mobilizer_num_ = -1;
  master_body_num_ = -1;
  slaves_.clear();
}

//------------------------------------------------------------------------------
//                          JOINT :: FORGET GRAPH
//------------------------------------------------------------------------------
// Restore the Joint to its state prior to GenerateGraph().
bool MultibodyGraphMaker::Joint::ForgetGraph(MultibodyGraphMaker* graph) {
  DRAKE_DEMAND(graph != nullptr);
  if (is_added_base_joint()) {
    graph->DeleteJoint(name_);
    return true;
  }
  mobilizer_num_ = -1;
  loop_constraint_num_ = -1;
  return false;
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
