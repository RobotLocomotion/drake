/* Adapted for Drake from Simbody's MultibodyGraphMaker class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/multibody/graph_maker/multibody_graph_maker.h"

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
#include "drake/multibody/graph_maker/multibody_graph.h"

using std::cout;
using std::endl;

namespace drake {
namespace multibody {

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
  RegisterJointType(weld_type_name_, 0, 0);
  RegisterJointType(free_type_name_, 7, 6);

  if (use_drake_defaults) {
    AddLink("world", ModelInstanceIndex(0), MultibodyGraph::kStaticLink);
    // These are the names used in .sdf files.
    RegisterJointType("revolute", 1, 1);
    RegisterJointType("prismatic", 1, 1);
    // TODO(sherm1) Should allow for a loop ball joint as soon as one is
    // available (change flag to kOkToUseAsJointConstraint).
    RegisterJointType("ball", 4, 3, MultibodyGraph::kDefaultJointTypeFlags);
    RegisterJointType("fixed", 0, 0);  // .sdf for "weld".

    // "weld" and "free" are always present.
  }
}

//------------------------------------------------------------------------------
//                            WORLD LINK NAME
//------------------------------------------------------------------------------
const std::string& MultibodyGraphMaker::world_link_name() const {
  if (links_.empty())
    throw std::logic_error(
        "get_world_link_name(): you can't call this until you have called "
        "AddLink() at least once -- the first link is World.");
  return links_[0].name();
}

//------------------------------------------------------------------------------
//                          REGISTER JOINT TYPE
//------------------------------------------------------------------------------
auto MultibodyGraphMaker::RegisterJointType(
    const std::string& name, int num_q, int num_v,
    MultibodyGraph::JointTypeFlags flags, void* user_ref) -> JointTypeNum {
  if (!(0 <= num_q && num_q <= 7 && 0 <= num_v && num_v <= 6 && num_v <= num_q))
    throw std::runtime_error(
        fmt::format("RegisterJointType(): Illegal joint state specification "
                    "nq={}, nv={} for joint type '{}'", num_q, num_v, name));

  // Reject duplicate type name.
  std::map<std::string, JointTypeNum>::const_iterator p =
      joint_type_name_to_num_.find(name);
  if (p != joint_type_name_to_num_.end())
    throw std::runtime_error(
        fmt::format("RegisterJointType(): Duplicate joint type '{}'.", name));

  const JointTypeNum joint_type_num(num_joint_types());  // next available
  joint_type_name_to_num_[name] = joint_type_num;  // provide fast name lookup

  // Can't use emplace_back because constructor is private.
  joint_types_.push_back(
      JointType(name, num_q, num_v, flags, user_ref, joint_type_num));

  return joint_type_num;
}

//------------------------------------------------------------------------------
//                                 ADD LINK
//------------------------------------------------------------------------------
void MultibodyGraphMaker::AddLink(const std::string& link_name,
                                  ModelInstanceIndex model_instance,
                                  MultibodyGraph::LinkFlags flags,
                                  void* user_ref) {
  if (!model_instance.is_valid())
    model_instance = default_model_instance();




  // Reject duplicate body name.
  std::map<std::string, LinkNum>::const_iterator p =
      link_name_to_num_.find(link_name);
  if (p != link_name_to_num_.end())
    throw std::runtime_error("AddLink(): Duplicate link name '" + link_name + "'");

  const LinkNum link_num(num_links());  // next available
  link_name_to_num_[link_name] = link_num;   // provide fast name lookup

  // Can't use emplace_back below because the constructor is private.
  if (link_num ==
      world_link_num()) {  // First link is World; use only the name and ref.
    links_.push_back(Link(link_name, ModelInstanceIndex(0),
                          MultibodyGraph::kStaticLink, user_ref));
  } else {  // This is a real link.
    links_.push_back(Link(link_name, model_instance, flags, user_ref));
  }
}

//------------------------------------------------------------------------------
//                             CHANGE LINK FLAGS
//------------------------------------------------------------------------------
void MultibodyGraphMaker::ChangeLinkFlags(const std::string& name,
                                          MultibodyGraph::LinkFlags flags) {
  const LinkNum link_num = FindLinkNum(name);
  if (!link_num.is_valid()) {
    throw std::logic_error("ChangeLinkMass(): link '" + name + "' not found.");
  }

  Link& link = get_mutable_link(link_num);
  link.set_flags(flags);
}

//------------------------------------------------------------------------------
//                                 DELETE LINK
//------------------------------------------------------------------------------
bool MultibodyGraphMaker::DeleteLink(const std::string& name) {
  // Reject non-existing link name.
  std::map<std::string, LinkNum>::iterator p = link_name_to_num_.find(name);
  if (p == link_name_to_num_.end()) return false;

  const LinkNum link_num = p->second;

  const std::vector<JointNum>& joints_as_parent =
      get_mutable_link(link_num).joints_as_parent();
  while (joints_as_parent.size() > 0)
    DeleteJoint(joints_[joints_as_parent[0]].name());

  const std::vector<JointNum>& joints_as_child =
      get_mutable_link(link_num).joints_as_child();
  while (joints_as_child.size() > 0)
    DeleteJoint(joints_[joints_as_child[0]].name());

  links_.erase(links_.begin() + link_num);
  link_name_to_num_.erase(p);

  // Update link indexes due to the deletion of this body.
  for (JointNum i(0); i < num_joints(); ++i) {
    if (joints_[i].parent_link_num() > link_num)
      get_mutable_joint(i).set_parent_link_num(
          LinkNum(get_joint(i).parent_link_num() - 1));
    if (joints_[i].child_link_num() > link_num)
      get_mutable_joint(i).set_child_link_num(
          LinkNum(get_joint(i).child_link_num() - 1));
  }

  for (LinkNum i = link_num; i < num_links(); ++i)
    link_name_to_num_[links_[i].name()] = i;

  return true;
}

//------------------------------------------------------------------------------
//                                ADD JOINT
//------------------------------------------------------------------------------
void MultibodyGraphMaker::AddJoint(const std::string& name,
                                   ModelInstanceIndex model_instance,
                                   const std::string& type,
                                   const std::string& parent_link_name,
                                   const std::string& child_link_name,
                                   MultibodyGraph::JointFlags flags,
                                   void* user_ref) {
  // Reject duplicate joint name, unrecognized type or body names.
  std::map<std::string, JointNum>::const_iterator p =
      joint_name_to_num_.find(name);
  if (p != joint_name_to_num_.end()) {
    throw std::runtime_error("AddJoint(): Duplicate joint name '" + name + "'");
  }

  const JointTypeNum type_num = FindJointTypeNum(type);
  if (!type_num.is_valid()) {
    throw std::runtime_error("AddJoint(): Joint " + name +
                             " had unrecognized joint type '" + type + "'");
  }

  const LinkNum parent_link_num = FindLinkNum(parent_link_name);
  if (!parent_link_num.is_valid()) {
    throw std::runtime_error("AddJoint(): Joint " + name +
                             " had unrecognized parent body '" +
                             parent_link_name + "'");
  }

  const LinkNum child_link_num = FindLinkNum(child_link_name);
  if (!child_link_num.is_valid()) {
    throw std::runtime_error("AddJoint(): Joint " + name +
                             " had unrecognized child body '" +
                             child_link_name + "'");
  }

  const JointNum joint_num(num_joints());  // next available slot
  joint_name_to_num_[name] = joint_num;    // provide fast name lookup

  // Can't use emplace_back because the constructor is private.
  joints_.push_back(Joint(name, model_instance, type_num, parent_link_num,
                          child_link_num, flags, user_ref));

  get_mutable_link(parent_link_num).add_joint_as_parent(joint_num);
  get_mutable_link(child_link_num).add_joint_as_child(joint_num);
}

//------------------------------------------------------------------------------
//                                DELETE JOINT
//------------------------------------------------------------------------------
bool MultibodyGraphMaker::DeleteJoint(const std::string& name) {
  std::map<std::string, JointNum>::iterator p = joint_name_to_num_.find(name);
  if (p == joint_name_to_num_.end()) return false;

  const JointNum joint_num = p->second;
  joint_name_to_num_.erase(p);

  std::vector<JointNum>& joints_as_parent =
      get_mutable_link(joints_[joint_num].parent_link_num())
          .mutable_joints_as_parent();
  std::vector<JointNum>::iterator it =
      std::find(joints_as_parent.begin(), joints_as_parent.end(), joint_num);
  if (it == joints_as_parent.end()) {
    throw std::runtime_error(
        "DeleteJoint(): Joint " + name +
        " doesn't exist in joints_as_parent of parent body ");
  }
  joints_as_parent.erase(it);

  std::vector<JointNum>& joints_as_child =
      get_mutable_link(joints_[joint_num].child_link_num())
          .mutable_joints_as_child();
  it = std::find(joints_as_child.begin(), joints_as_child.end(), joint_num);
  if (it == joints_as_child.end()) {
    throw std::runtime_error(
        "DeleteJoint(): Joint " + name +
        " doesn't exist in joints_as_child of child body ");
  }
  joints_as_child.erase(it);

  joints_.erase(joints_.begin() + joint_num);

  // Update indices due to the deletion of this joint
  for (LinkNum i(0); i < num_links(); ++i) {
    auto& children = get_mutable_link(i).mutable_joints_as_parent();
    for (auto& child : children)
      if (child > joint_num) {
        --child;
        joint_name_to_num_[joints_[child].name()] = child;
      }

    auto& parents = get_mutable_link(i).mutable_joints_as_child();
    for (auto& parent : parents)
      if (parent > joint_num) {
        --parent;
        // No need to adjust joint_name_to_num_ here since the first loop has
        // done it already.
      }
  }

  return true;
}

//------------------------------------------------------------------------------
//                               DUMP INPUT
//------------------------------------------------------------------------------
void MultibodyGraphMaker::DumpInput(std::ostream& o) const {
  o << "\nMULTIBODY GRAPH MAKER INPUT\n";
  o << "---------------------------\n";
  o << "\n" << num_links() << " LINKS:\n";
  for (LinkNum i(0); i < num_links(); ++i) {
    const Link& link = get_link(i);
    const std::string out =
        fmt::format("{}: {} flags={}", i, link.name(), to_string(link.flags()));
    o << out;
    o << "\n    joints_as_parent=[";
    for (JointNum j(0); j < link.num_children(); ++j)
      o << " " << link.joints_as_parent()[j];
    o << "]\t  joints_as_child=[";
    for (JointNum j(0); j < link.num_parents(); ++j)
      o << " " << link.joints_as_child()[j];
    o << "]\n";
  }

  o << "\n" << num_joints() << " JOINTS:\n";
  for (JointNum i(0); i < num_joints(); ++i) {
    const Joint& joint = get_joint(i);
    const std::string out =
        fmt::format("{}: {} {}->{} {} flags={}\n", i, joint.name(),
                    get_link(joint.parent_link_num()).name(),
                    get_link(joint.child_link_num()).name(),
                    get_joint_type(joint.joint_type_num()).name(),
                    to_string(joint.flags()));
    o << out;
  }

  o << "\n---------- END OF MULTIBODY GRAPH MAKER INPUT.\n\n";
}

//------------------------------------------------------------------------------
//                              GENERATE GRAPH
//------------------------------------------------------------------------------
std::unique_ptr<MultibodyGraph> MultibodyGraphMaker::MakeGraph() {
  std::unique_ptr<MultibodyGraph> graph(new MultibodyGraph());

  // Link and Joint inputs have been supplied. Link 0 is World.

  // First construct the table of joint types in the generated graph.
  for (JointTypeNum joint_type_num(0); joint_type_num < num_joint_types();
       ++joint_type_num) {
    const JointType& joint_type = get_joint_type(joint_type_num);
    const JointTypeNum graph_joint_type_num =
        graph->AddJointType(joint_type.name(), joint_type.user_ref());
    DRAKE_DEMAND(graph_joint_type_num == joint_type_num);
  }

  // Next, add a subset of the link and joint information to the graph.
  for (LinkNum link_num(0); link_num < num_links(); ++link_num) {
    const Link& link = get_link(link_num);
    const LinkNum graph_link_num =
        graph->AddLinkInfo(link.name(), link.user_ref());
    DRAKE_DEMAND(graph_link_num == link_num);
  }

  for (JointNum joint_num(0); joint_num < num_joints(); ++joint_num) {
    const Joint& joint = get_joint(joint_num);
    const JointNum graph_joint_num = graph->AddJointInfo(
        joint.name(), joint.joint_type_num(), joint.user_ref());
    DRAKE_DEMAND(graph_joint_num == joint_num);
  }

  // Next, create a Body in graph corresponding to each Link.
  // By construction, body numbers are the same as their corresponding
  // link numbers.
  for (LinkNum link_num(0); link_num < num_links(); ++link_num) {
    const MultibodyGraph::BodyNum body_num = graph->AddBodyFromLink(link_num);
    DRAKE_DEMAND(body_num.to_int() == link_num.to_int());
  }

  // 1. Add a free mobilizer to World for any body whose link said it must be
  // a base body and don't already have a connection to World.
  //
  // While we're at it, look for dangling massless links -- they are only
  // allowed if they were originally connected to at least two other links
  // by given tree-eligible joints, or if they are connected by a weld
  // joint and thus have no mobility.
  // TODO(sherm1): "must be constraint" joints shouldn't count towards the
  // required two for massless bodies, but we're not checking.
  for (LinkNum link_num(1); link_num < num_links(); ++link_num) {  // skip World
    const Link& link = get_link(link_num);
    if (link.must_be_nonterminal()) {
      if (link.num_joints() == 0) {
        throw std::runtime_error(
            "MakeGraph(): link " + link.name() +
            " must be nonterminal but it is free free (no joint). Must be"
            " internal or welded to a terminal-ok link.");
      }
      if (link.num_joints() == 1) {
        const JointNum joint_num = link.joints_as_child().empty()
                                       ? link.joints_as_parent()[0]
                                       : link.joints_as_child()[0];
        const Joint& joint = get_joint(joint_num);
        const JointType& joint_type = get_joint_type(joint.joint_type_num());
        if (joint_type.num_v() > 0) {
          throw std::runtime_error(
              "MakeGraph(): link " + link.name() +
              " must be nonterminal but is not internal and not welded to"
              " a terminal-ok link.");
        }
      }
    }

    // Attach the body of must-be-base-body links to World.
    if (link.must_be_base_body() &&
        !LinksAreConnected(link_num, world_link_num())) {
      const MultibodyGraph::BodyNum body_num{
          link_num.to_int()};  // By construction.
      graph->ConnectBodyToWorld(body_num, false /* not static */);
    }
  }

  // 2. Repeat until all bodies are in the tree:
  //   - try to build the tree from World outwards
  //   - if incomplete, add one missing connection to World
  // This terminates because we add at  least one body to the tree each time.
  int start_level = 1;  // Start by looking for base bodies.
  while (true) {
    GrowTree(start_level, &*graph);
    const LinkNum new_base_link = ChooseNewBaseLink(*graph);
    if (!new_base_link.is_valid()) break;  // all link are in the tree
    // Add mobilizer to World for this link's body.
    graph->ConnectBodyToWorld(graph->link_to_body_num(new_base_link),
                              false /* not static */);
    start_level = 2;  // Start from new base link.
  }

  // 3. Split the loops
  // This requires adding new "slave" bodies to replace the child bodies in
  // the loop-forming joints. Then each slave is welded back to its master
  // body (the original child).
  BreakLoops(&*graph);

  return graph;
}

//------------------------------------------------------------------------------
//                        GET LINK NUM FROM NAME
//------------------------------------------------------------------------------
auto MultibodyGraphMaker::GetAllLinkNumsFromName(const std::string& link_name)
    const -> const std::map<ModelInstanceIndex, LinkNum>* {
  const auto link_nums = link_name_to_num_.find(link_name);
  if (link_nums == link_name_to_num_.end()) return nullptr;
  const auto& instance_map = link_nums->second;
  DRAKE_DEMAND(!instance_map.empty());
  return &instance_map;
}

auto MultibodyGraphMaker::GetLinkNumFromNameAndInstance(
    const std::string& link_name, ModelInstanceIndex model_instance,
    const char* func) const -> LinkNum {
  DRAKE_DEMAND(model_instance.is_valid());

  if (auto instance_map = GetAllLinkNumsFromName(link_name)) {
    const auto entry = instance_map->find(model_instance);
    if (entry != instance_map->end()) {
      DRAKE_DEMAND(entry->second.is_valid());
      return entry->second;
    }
  }

  throw std::logic_error(
      fmt::format("{}: link '{}' not found in model_instance {}.", func,
                  link_name, model_instance));
}

auto MultibodyGraphMaker::GetUniqueLinkNumFromName(const std::string& link_name,
                                                   const char* func) const
    -> LinkNum {
  auto instance_map = GetAllLinkNumsFromName(link_name);

  if (instance_map && instance_map->size() == 1) {
    const LinkNum link_num = instance_map->cbegin()->second;
    DRAKE_DEMAND(link_num.is_valid());
    return link_num;
  }

  throw std::logic_error(
      fmt::format("{}: link name '{}' not unique. It appears in {} "
                  "model instances. Provide a model instance to "
                  "disambiguate.",
                  func, link_name, instance_map->size()));
}

//------------------------------------------------------------------------------
//                        GET JOINT NUM FROM NAME
//------------------------------------------------------------------------------
auto MultibodyGraphMaker::GetAllJointNumsFromName(const std::string& joint_name,
                                                  const char* func) const
    -> const std::map<ModelInstanceIndex, JointNum>& {
  const auto joint_nums = joint_name_to_num_.find(joint_name);
  if (joint_nums == joint_name_to_num_.end()) {
    throw std::logic_error(
        fmt::format("{}: joint '{}' not found.", func, joint_name));
  }
  const auto& instance_map = joint_nums->second;
  DRAKE_DEMAND(!instance_map.empty());
  return instance_map;
}

auto MultibodyGraphMaker::GetJointNumFromNameAndInstance(
    const std::string& joint_name, ModelInstanceIndex model_instance,
    const char* func) const -> JointNum {
  DRAKE_DEMAND(model_instance.is_valid());
  const auto& instance_map = GetAllJointNumsFromName(joint_name, func);
  const auto entry = instance_map.find(model_instance);
  if (entry == instance_map.end()) {
    throw std::logic_error(
        fmt::format("{}: joint '{}' not found in model_instance {}.", func,
                    joint_name, model_instance));
  }
  DRAKE_DEMAND(entry->second.is_valid());
  return entry->second;
}

auto MultibodyGraphMaker::GetUniqueJointNumFromName(
    const std::string& joint_name, const char* func) const -> JointNum {
  const auto& instance_map = GetAllJointNumsFromName(joint_name, func);
  if (instance_map.size() > 1) {
    throw std::logic_error(
        fmt::format("{}: joint name '{}' not unique. It appears in {} "
                    "model instances. Provide a model instance to "
                    "disambiguate.",
                    func, joint_name, instance_map.size()));
  }
  const JointNum joint_num = instance_map.cbegin()->second;
  DRAKE_DEMAND(joint_num.is_valid());
  return joint_num;
}

//------------------------------------------------------------------------------
//                          CHOOSE NEW BASE LINK
//------------------------------------------------------------------------------
// We've tried to build the tree but might not have succeeded in using
// all the links. That means we'll have to connect one of them to World.
// Select the best unattached link to model with a base body, or returns
// an invalid LinkNum if all links
// are already modeled in the spanning tree. We hope to find a link that is
// serving as a parent but has never been listed as a child; that is crying out
// to be connected directly to World. We'll take the first one of those we
// find to roughly preserve the order of connected graphs in the input.
//
// If there is no parent-only link, we're going to have to pick a child link
// as a base link, meaning that any joint for which it was the child will have
// to use a reversed mobilizer (because a base link is always a parent). Of
// those we pick the one with the most children.
//
// Note that any must-be-base-body links get attached to World before anything
// else; that is done at the start of MakeGraph() rather than here.
auto MultibodyGraphMaker::ChooseNewBaseLink(const MultibodyGraph& graph) const
    -> LinkNum {
  LinkNum best_link;
  int num_children = -1;
  // Skip the World body.
  // TODO(sherm1) This runs through all the links each time; if that
  // becomes expensive, start after the last-found base link.
  for (LinkNum link_num(1); link_num < num_links(); ++link_num) {
    const Link& link = links_[link_num];
    const MultibodyGraph::Body& body =
        graph.body(graph.link_to_body_num(link_num));
    if (body.is_in_tree()) continue;  // Unavailable.

    // If this is a parent-only link, choose it.
    if (link.num_parents() == 0) return link_num;

    // This is a childish link, keep the one with the most children.
    if (link.num_children() > num_children) {
      best_link = link_num;
      num_children = link.num_children();
    }
  }
  return best_link;
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
MultibodyGraph::MobilizerNum MultibodyGraphMaker::AddMobilizerForJoint(
    JointNum joint_num, MultibodyGraph* graph) {
  Joint& joint = get_mutable_joint(joint_num);
  DRAKE_DEMAND(!joint.must_be_constraint());  // can't be a tree joint
  DRAKE_DEMAND(!graph->joint_info(joint_num).has_mobilizer());  // already done

  const LinkNum parent_link_num = joint.parent_link_num();
  const LinkNum child_link_num = joint.child_link_num();

  const MultibodyGraph::BodyNum parent_body_num =
      graph->link_to_body_num(parent_link_num);
  const MultibodyGraph::BodyNum child_body_num =
      graph->link_to_body_num(child_link_num);
  MultibodyGraph::Body& parent_body = graph->get_mutable_body(parent_body_num);
  MultibodyGraph::Body& child_body = graph->get_mutable_body(parent_body_num);

  // Exactly one of these must already be in the tree.
  DRAKE_DEMAND(parent_body.is_in_tree() ^ child_body.is_in_tree());

  MultibodyGraph::MobilizerNum mobilizer_num;
  if (parent_body.is_in_tree()) {
    // Child is outboard body (forward joint)
    mobilizer_num = graph->AddMobilizerFromJoint(
        joint_num, parent_body_num, child_body_num, false /* not reversed */);
  } else if (child_body.is_in_tree()) {
    // Parent is outboard body (reverse joint)
    mobilizer_num = graph->AddMobilizerFromJoint(
        joint_num, child_body_num, parent_body_num, true /* reversed */);
  }
  return mobilizer_num;
}

//------------------------------------------------------------------------------
//                   FIND HEAVIEST UNASSIGNED FORWARD JOINT
//------------------------------------------------------------------------------
// Starting with a given link whose body is in the tree already, look at its
// unassigned children (meaning links connected by joints that aren't
// mobilizers yet) and return a joint connecting it to a child that can serve
// as a terminal body (has mass). (The idea is that this would be a good
// direction to extend the chain.) Returns an invalid joint number if this link
// has no children.
auto MultibodyGraphMaker::FindHeaviestUnassignedForwardJoint(
    LinkNum parent_link_num, const MultibodyGraph& graph) const -> JointNum {
  const Link& parent_link = get_link(parent_link_num);
  const MultibodyGraph::Body& inboard_body =
      graph.link_to_body(parent_link_num);
  DRAKE_DEMAND(inboard_body.is_in_tree());

  JointNum joint_num;
  // Search joints for which this is the parent link.
  for (JointNum joint_forward : parent_link.joints_as_parent()) {
    const Joint& joint = joints_[joint_forward];
    if (joint.must_be_constraint()) continue;  // can't be a tree joint

    if (graph.joint_info(joint_forward).has_mobilizer())
      continue;  // already in the tree

    const Link& child_link = get_link(joint.child_link_num());
    const MultibodyGraph::Body& child_body =
        graph.link_to_body(joint.child_link_num());

    if (child_body.is_in_tree()) continue;  // this is a loop joint

    if (!child_link.must_be_nonterminal()) {
      joint_num = joint_forward;
      break;
    }
  }
  return joint_num;
}

//------------------------------------------------------------------------------
//                   FIND HEAVIEST UNASSIGNED REVERSE JOINT
//------------------------------------------------------------------------------
// Same as previous method, but now we are looking for unassigned joints where
// the given link serves as the child so we would have to reverse the joint to
// add it to the tree. (This is less desirable so is a fallback.)
auto MultibodyGraphMaker::FindHeaviestUnassignedReverseJoint(
    LinkNum child_link_num, const MultibodyGraph& graph) const -> JointNum {
  const Link& child_link = get_link(child_link_num);
  const MultibodyGraph::Body& outboard_body =
      graph.link_to_body(child_link_num);
  DRAKE_DEMAND(outboard_body.is_in_tree());

  JointNum joint_num;
  // Search joints for which this is the child body.
  for (JointNum joint_reverse : child_link.joints_as_child()) {
    const Joint& joint = joints_[joint_reverse];
    if (joint.must_be_constraint()) continue;  // can't be a tree joint

    if (graph.joint_info(joint_reverse).has_mobilizer())
      continue;  // already in the tree

    const Link& parent_link = get_link(joint.parent_link_num());
    const MultibodyGraph::Body& parent_body =
        graph.link_to_body(joint.parent_link_num());

    if (parent_body.is_in_tree()) continue;  // this is a loop joint

    if (!parent_link.must_be_nonterminal()) {
      joint_num = joint_reverse;
      break;
    }
  }
  return joint_num;
}

//------------------------------------------------------------------------------
//                                 GROW TREE
//------------------------------------------------------------------------------
// Process unused joints for which one link's body is in the tree (at level h)
// and the other is not. Add the other link's body as the outboard body at
// level h+1, marking the mobilizer as forward (other body is child) or reverse
// (other body is parent). Repeat
// until no changes are made. Does not assign loop joints or any links that
// don't have a path to World. Extend the multibody tree starting with the
// lowest-level eligible joints and moving outwards. We're doing this
// breadth-first so that we get roughly even-length chains and we'll stop at
// the first level at which we fail to find any joint. If we fail to consume
// all the links, the caller will have to add a level 1 mobilizer to attach a
// previously-disconnected base link to World and then call this again.
//
// We violate the breadth-first heuristic to avoid ending a branch with a
// massless link, unless it is welded to its parent. If we add a mobile massless
// link, we'll keep going out along its branch until we hit a massful link. It
// is a disaster if we fail to find a massful link because a tree that
// terminates in a mobile massless body will generate a singular mass matrix.
// We'll throw an exception in that case, but note that this may just be a
// failure of the heuristic; there may be some tree that could have avoided the
// terminal massless body but we failed to discover it.
//
// TODO(sherm1): keep a list of unprocessed joints so we don't have to go
// through all of them again at each level.
void MultibodyGraphMaker::GrowTree(int start_level, MultibodyGraph* graph) {
  // Record the joints for which we added mobilizers during this subtree
  // sweep. That way if we jumped levels ahead due to massless bodies we
  // can take credit and keep going rather than quit.
  std::set<JointNum> joints_added;
  for (int level = start_level;; ++level) {  // level of outboard body
    bool any_mobilizer_added = false;
    for (JointNum joint_num(0); joint_num < num_joints(); ++joint_num) {
      // See if this joint is at the right level (meaning its inboard
      // body is at level-1) and is available to become a mobilizer.
      const Joint& joint = get_joint(joint_num);
      const MultibodyGraph::JointInfo& joint_info =
          graph->joint_info(joint_num);
      if (joint_info.has_mobilizer()) {
        // Already done -- one of ours?
        if (joints_added.count(joint_num)) {
          // We added it during this GrowTree() call -- is it at
          // the current level?
          const MultibodyGraph::Mobilizer& mobilizer =
              graph->get_mobilizer(joint_info.mobilizer_num());
          if (mobilizer.level() == level)
            any_mobilizer_added = true;  // Added one at level.
        }
        continue;
      }

      if (joint.must_be_constraint()) continue;  // can't be a tree joint

      const Link& parent_link = get_link(joint.parent_link_num());
      const Link& child_link = get_link(joint.child_link_num());

      const MultibodyGraph::BodyNum parent_body_num =
          graph->link_to_body_num(joint.parent_link_num());
      const MultibodyGraph::BodyNum child_body_num =
          graph->link_to_body_num(joint.child_link_num());

      const MultibodyGraph::Body& parent_body = graph->body(parent_body_num);
      const MultibodyGraph::Body& child_body = graph->body(child_body_num);

      // Exactly one of parent or child body must already be in the tree.
      if (!(parent_body.is_in_tree() ^ child_body.is_in_tree())) continue;

      // Stop growing this branch after the current level so we can balance
      // branch lengths. However, if our outboard body is massless we don't
      // want to risk it ending up as terminal so keep going in that case.
      LinkNum inboard_link_num, outboard_link_num;
      MultibodyGraph::BodyNum inboard_body_num, outboard_body_num;
      if (parent_body.is_in_tree()) {
        if (!child_link.must_be_nonterminal() &&
            parent_body.level() != level - 1)
          continue;  // not time yet
        inboard_link_num = joint.parent_link_num();
        outboard_link_num = joint.child_link_num();
        inboard_body_num = parent_body_num;
        outboard_body_num = child_body_num;
      } else {  // child is in tree
        if (!parent_link.must_be_nonterminal() &&
            child_body.level() != level - 1)
          continue;  // not time yet
        inboard_link_num = joint.child_link_num();
        outboard_link_num = joint.parent_link_num();
        inboard_body_num = child_body_num;
        outboard_body_num = parent_body_num;
      }

      MultibodyGraph::MobilizerNum mobilizer_num = graph->AddMobilizerFromJoint(
          joint_num, inboard_body_num, outboard_body_num,
          child_body.is_in_tree());

      joints_added.insert(joint_num);
      any_mobilizer_added = true;

      // We just made joint joint_num a mobilizer. If its outboard link
      // is massless and the mobilizer was not a weld, we need to keep
      // extending this branch of the tree until we can end it with a
      // massful link.
      const Link& outboard_link = get_link(outboard_link_num);
      const JointType& joint_type = get_joint_type(joint.joint_type_num());
      if (joint_type.num_v() == 0 || !outboard_link.must_be_nonterminal())
        continue;

      // Pick a further-outboard body with mass and extend branch to it.
      // Prefer forward-direction joints if possible.
      // If only massless outboard bodies are available, add one and
      // keep going.
      while (true) {
        // The outboard link of the mobilizer we just added will be the next
        // one we connect. Ideally it will serve as inboard, but we may have to
        // reverse.
        const LinkNum link_num = graph->body_to_link_num(
            graph->get_mobilizer(mobilizer_num).outboard_body_num());

        const JointNum joint_forward_num =
            FindHeaviestUnassignedForwardJoint(link_num, *graph);
        if (joint_forward_num.is_valid()) {
          const Joint& joint_forward = get_joint(joint_forward_num);
          const LinkNum fwd_outboard_link_num = joint_forward.child_link_num();
          if (!get_link(fwd_outboard_link_num).must_be_nonterminal()) {
            mobilizer_num = graph->AddMobilizerFromJoint(
                joint_forward_num, graph->link_to_body_num(link_num),
                graph->link_to_body_num(fwd_outboard_link_num),
                false /* not reversed */);
            joints_added.insert(joint_forward_num);
            break;
          }
        }

        const JointNum joint_reverse_num =
            FindHeaviestUnassignedReverseJoint(link_num, *graph);
        if (joint_reverse_num.is_valid()) {
          const Joint& joint_reverse = get_joint(joint_reverse_num);
          const LinkNum rev_outboard_link_num = joint_reverse.parent_link_num();
          if (!get_link(rev_outboard_link_num).must_be_nonterminal()) {
            mobilizer_num = graph->AddMobilizerFromJoint(
                joint_reverse_num, graph->link_to_body_num(link_num),
                graph->link_to_body_num(rev_outboard_link_num),
                true /* reversed */);
            joints_added.insert(joint_reverse_num);
            break;
          }
        }

        // Couldn't find a massful body to add. Add another massless
        // body (if there is one) and keep trying.
        if (joint_forward_num.is_valid()) {
          mobilizer_num = graph->AddMobilizerFromJoint(
              joint_forward_num, graph->link_to_body_num(link_num),
              graph->link_to_body_num(joint.child_link_num()),
              false /* not reversed */);
          joints_added.insert(joint_forward_num);
          continue;
        }
        if (joint_reverse_num.is_valid()) {
          mobilizer_num = graph->AddMobilizerFromJoint(
              joint_reverse_num, graph->link_to_body_num(link_num),
              graph->link_to_body_num(joint.parent_link_num()),
              true /* reversed */);
          joints_added.insert(joint_reverse_num);
          continue;
        }

        // Uh oh. Nothing outboard of the massless body we just added.
        // We're ending this chain with a massless body; not good.
        throw std::runtime_error(
            "GrowTree(): algorithm produced an"
            " invalid tree containing a terminal massless link (" +
            get_link(link_num).name() +
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
void MultibodyGraphMaker::BreakLoops(MultibodyGraph* graph) {
  for (JointNum joint_num(0); joint_num < num_joints(); ++joint_num) {
    const Joint& joint = joints_[joint_num];
    if (graph->joint_info(joint_num).has_mobilizer()) continue;  // already done

    const LinkNum parent_link_num = joint.parent_link_num();
    const LinkNum child_link_num = joint.child_link_num();
    const MultibodyGraph::BodyNum parent_body_num =
        graph->link_to_body_num(parent_link_num);
    const MultibodyGraph::BodyNum child_body_num =
        graph->link_to_body_num(child_link_num);
    DRAKE_DEMAND(graph->body(parent_body_num).is_in_tree() &&
                 graph->body(child_body_num).is_in_tree());

    // Use a loop joint if there's a good one available.
    const JointType& joint_type = get_joint_type(joint.joint_type_num());
    if (joint_type.have_good_joint_constraint_available()) {
      graph->AddConstraintFromJoint(joint_num, parent_body_num, child_body_num);
      continue;
    }

    // No usable loop constraint for this type of joint. Add a new slave
    // body so we can use a mobilizer. (Body references are invalidated here
    // since we're adding a new body -- don't reuse them.)

    // First assume we will be able to split the child, which is the usual case.
    LinkNum split_link_num = child_link_num;
    LinkNum inboard_link_num = parent_link_num;
    bool is_reversed = false;
    // If the child is World, choose the parent instead, reverse the mobilizer.
    if (child_link_num == world_link_num()) {
      DRAKE_DEMAND(parent_link_num != world_link_num());
      split_link_num = parent_link_num;
      inboard_link_num = child_link_num;  // That is, World.
      is_reversed = true;
    }
    const MultibodyGraph::BodyNum split_body_num =
        graph->link_to_body_num(split_link_num);
    const MultibodyGraph::BodyNum inboard_body_num =
        graph->link_to_body_num(inboard_link_num);
    const MultibodyGraph::BodyNum slave_body_num =
        graph->SplitBody(split_body_num);

    // Mobilize the slave body.
    graph->AddMobilizerFromJoint(joint_num, inboard_body_num, slave_body_num,
                                 is_reversed);

    // Add the weld loop constraint.
    std::string constraint_name = "#" + graph->body(split_body_num).name() +
                                  "|" + graph->body(slave_body_num).name();
    graph->AddSlaveWeldConstraint(std::move(constraint_name),
                                  joint_num, split_body_num,
                                  slave_body_num);
  }
}

//------------------------------------------------------------------------------
//                         LINKS ARE CONNECTED
//------------------------------------------------------------------------------
// Return true if there is a joint between two links given by link number,
// regardless of parent/child ordering.
bool MultibodyGraphMaker::LinksAreConnected(LinkNum link1_num,
                                            LinkNum link2_num) const {
  const Link& link1 = get_link(link1_num);
  for (JointNum joint_num : link1.joints_as_parent()) {
    if (get_joint(joint_num).child_link_num() == link2_num) return true;
  }
  for (JointNum joint_num : link1.joints_as_child()) {
    if (get_joint(joint_num).parent_link_num() == link2_num) return true;
  }
  return false;
}

}  // namespace multibody
}  // namespace drake
