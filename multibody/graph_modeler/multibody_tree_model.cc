/* Adapted for Drake from Simbody's MultibodyGraphModeler class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/multibody/graph_modeler/multibody_tree_model.h"

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

using std::cout;
using std::endl;

namespace drake {
namespace multibody {

//------------------------------------------------------------------------------
//                              MAKE TREE MODEL
//------------------------------------------------------------------------------
void MultibodyTreeModel::MakeTreeModel(const MultibodyGraphModeler& graph) {
  // Body and Joint inputs have been supplied. Body 0 is World.

  // First construct the table of joint types in the generated tree.
  for (JointTypeIndex joint_type_num(0);
       joint_type_num < graph.num_joint_types(); ++joint_type_num) {
    const JointType& joint_type = graph.get_joint_type(joint_type_num);
    const JointTypeIndex graph_joint_type_num =
        AddJointType(joint_type.name(), joint_type.user_ref());
    DRAKE_DEMAND(graph_joint_type_num == joint_type_num);
  }

  // Next, add a subset of the body and joint information to the tree.
  for (BodyIndex body_num(0); body_num < graph.num_bodies(); ++body_num) {
    const Body& body = graph.get_body(body_num);
    const BodyIndex graph_body_num = AddBodyInfo(body.name(), body.user_ref());
    DRAKE_DEMAND(graph_body_num == body_num);
  }

  for (JointIndex joint_num(0); joint_num < graph.num_joints(); ++joint_num) {
    const Joint& joint = graph.get_joint(joint_num);
    const JointIndex graph_joint_num =
        AddJointInfo(joint.name(), joint.joint_type_num(), joint.user_ref());
    DRAKE_DEMAND(graph_joint_num == joint_num);
  }

  // Next, create a MobilizedBody in tree corresponding to each Body.
  // By construction, mobilized body numbers are the same as their corresponding
  // input body numbers.
  for (BodyIndex body_num(0); body_num < graph.num_bodies(); ++body_num) {
    const MobilizedBodyNum mobod_num = AddMobodFromBody(body_num);
    DRAKE_DEMAND(mobod_num.to_int() == body_num.to_int());
  }

  // 1. Add a free mobilizer to World for any mobilized body whose input body
  // said it must be a base body and didn't already have a connection to World.
  //
  // While we're at it, look for dangling massless bodies -- they are only
  // allowed if they were originally connected to at least two other bodies
  // by given tree-eligible joints, or if they are connected by a weld
  // joint and thus have no mobility.
  // TODO(sherm1): "must be constraint" joints shouldn't count towards the
  // required two for massless bodies, but we're not checking.
  for (BodyIndex body_num(1); body_num < graph.num_bodies();
       ++body_num) {  // skip World
    const Body& body = graph.get_body(body_num);
    if (body.must_be_nonterminal()) {
      if (body.num_joints() == 0) {
        throw std::runtime_error(
            "MakeTreeModel(): body " + body.name() +
            " must be nonterminal but it is free (no joint). Must be"
            " internal or welded to a terminal-ok body.");
      }
      if (body.num_joints() == 1) {
        const JointIndex joint_num = body.joints_as_child().empty()
                                         ? body.joints_as_parent()[0]
                                         : body.joints_as_child()[0];
        const Joint& joint = graph.get_joint(joint_num);
        const JointType& joint_type =
            graph.get_joint_type(joint.joint_type_num());
        if (joint_type.num_v() > 0) {
          throw std::runtime_error(
              "MakeTreeModel(): body " + body.name() +
              " must be nonterminal but is not internal and not welded to"
              " a terminal-ok body.");
        }
      }
    }

    // Attach the body of must-be-base-body bodies to World.
    if (body.must_be_base_body() &&
        !graph.BodiesAreConnected(body_num, graph.world_body_num())) {
      const MultibodyTreeModel::MobilizedBodyNum mobod_num{
          body_num.to_int()};  // By construction.
      ConnectBodyToWorld(mobod_num, false /* not static */);
    }
  }

  // 2. Repeat until all bodies are in the tree:
  //   - try to build the tree from World outwards
  //   - if incomplete, add one missing connection to World
  // This terminates because we add at  least one body to the tree each time.
  int start_level = 1;  // Start by looking for base bodies.
  while (true) {
    GrowTree(graph, start_level);
    const BodyIndex new_base_body = ChooseNewBaseBody(graph);
    if (!new_base_body.is_valid()) break;  // all body are in the tree
    // Add mobilizer to World for this mobilized body's body.
    ConnectBodyToWorld(body_to_mobod_num(new_base_body),
                       false /* not static */);
    start_level = 2;  // Start from new base body.
  }

  // 3. Split the loops
  // This requires adding new "slave" bodies to replace the child bodies in
  // the loop-forming joints. Then each slave is welded back to its master
  // body (the original child).
  BreakLoops(graph);
}

//------------------------------------------------------------------------------
//                          CHOOSE NEW BASE BODY
//------------------------------------------------------------------------------
// We've tried to build the tree but might not have succeeded in using
// all the bodies. That means we'll have to connect one of them to World.
// Select the best unattached body to model with a base body, or returns
// an invalid BodyIndex if all bodies
// are already modeled in the spanning tree. We hope to find a body that is
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
// else; that is done at the start of MakeTreeModel() rather than here.
auto MultibodyTreeModel::ChooseNewBaseBody(
    const MultibodyGraphModeler& graph) const -> BodyIndex {
  BodyIndex best_body;
  int num_children = -1;
  // Skip the World body.
  // TODO(sherm1) This runs through all the bodies each time; if that
  // becomes expensive, start after the last-found base body.
  for (BodyIndex body_num(1); body_num < graph.num_bodies(); ++body_num) {
    const Body& body = graph.get_body(body_num);
    const MobilizedBody& mobod = this->mobod(body_to_mobod_num(body_num));
    if (mobod.is_in_tree()) continue;  // Unavailable.

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
//                   FIND HEAVIEST UNASSIGNED FORWARD JOINT
//------------------------------------------------------------------------------
// Starting with a given body whose body is in the tree already, look at its
// unassigned children (meaning bodies connected by joints that aren't
// mobilizers yet) and return a joint connecting it to a child that can serve
// as a terminal body (has mass). (The idea is that this would be a good
// direction to extend the chain.) Returns an invalid joint number if this body
// has no children.
auto MultibodyTreeModel::FindHeaviestUnassignedForwardJoint(
    BodyIndex parent_body_num, const MultibodyGraphModeler& graph) const
    -> JointIndex {
  const Body& parent_body = graph.get_body(parent_body_num);
  const MultibodyTreeModel::MobilizedBody& inboard_body =
      body_to_mobod(parent_body_num);
  DRAKE_DEMAND(inboard_body.is_in_tree());

  JointIndex joint_num;
  // Search joints for which this is the parent body.
  for (JointIndex joint_forward : parent_body.joints_as_parent()) {
    const Joint& joint = graph.get_joint(joint_forward);
    if (joint.must_be_constraint()) continue;  // Can't be a tree joint.

    if (joint_info(joint_forward).has_mobilizer())
      continue;  // already in the tree

    const Body& child_body = graph.get_body(joint.child_body_num());
    const MobilizedBody& child_mobod = body_to_mobod(joint.child_body_num());

    if (child_mobod.is_in_tree()) continue;  // This is a loop joint.

    if (!child_body.must_be_nonterminal()) {
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
// the given body serves as the child so we would have to reverse the joint to
// add it to the tree. (This is less desirable so is a fallback.)
auto MultibodyTreeModel::FindHeaviestUnassignedReverseJoint(
    BodyIndex child_body_num, const MultibodyGraphModeler& graph) const
    -> JointIndex {
  const Body& child_body = graph.get_body(child_body_num);
  const MultibodyTreeModel::MobilizedBody& outboard_body =
      body_to_mobod(child_body_num);
  DRAKE_DEMAND(outboard_body.is_in_tree());

  JointIndex joint_num;
  // Search joints for which this is the child body.
  for (JointIndex joint_reverse : child_body.joints_as_child()) {
    const Joint& joint = graph.get_joint(joint_reverse);
    if (joint.must_be_constraint()) continue;  // Can't be a tree joint.

    if (joint_info(joint_reverse).has_mobilizer())
      continue;  // already in the tree

    const Body& parent_body = graph.get_body(joint.parent_body_num());
    const MobilizedBody& parent_mobod =
        body_to_mobod(joint.parent_body_num());

    if (parent_mobod.is_in_tree()) continue;  // This is a loop joint.

    if (!parent_body.must_be_nonterminal()) {
      joint_num = joint_reverse;
      break;
    }
  }
  return joint_num;
}

//------------------------------------------------------------------------------
//                                 GROW TREE
//------------------------------------------------------------------------------
// Process unused joints for which one body's body is in the tree (at level h)
// and the other is not. Add the other body's body as the outboard body at
// level h+1, marking the mobilizer as forward (other body is child) or reverse
// (other body is parent). Repeat
// until no changes are made. Does not assign loop joints or any bodies that
// don't have a path to World. Extend the multibody tree starting with the
// lowest-level eligible joints and moving outwards. We're doing this
// breadth-first so that we get roughly even-length chains and we'll stop at
// the first level at which we fail to find any joint. If we fail to consume
// all the bodies, the caller will have to add a level 1 mobilizer to attach a
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
void MultibodyTreeModel::GrowTree(const MultibodyGraphModeler& graph,
                                  int start_level) {
  // Record the joints for which we added mobilizers during this subtree
  // sweep. That way if we jumped levels ahead due to massless bodies we
  // can take credit and keep going rather than quit.
  std::set<JointIndex> joints_added;
  for (int level = start_level;; ++level) {  // level of outboard body
    bool any_mobilizer_added = false;
    for (JointIndex joint_num(0); joint_num < num_joints(); ++joint_num) {
      // See if this joint is at the right level (meaning its inboard
      // body is at level-1) and is available to become a mobilizer.
      const Joint& joint = graph.get_joint(joint_num);
      const JointInfo& joint_info =
          this->joint_info(joint_num);
      if (joint_info.has_mobilizer()) {
        // Already done -- one of ours?
        if (joints_added.count(joint_num)) {
          // We added it during this GrowTree() call -- is it at
          // the current level?
          const Mobilizer& mobilizer =
              get_mobilizer(joint_info.mobilizer_num());
          if (mobilizer.level() == level)
            any_mobilizer_added = true;  // Added one at level.
        }
        continue;
      }

      if (joint.must_be_constraint()) continue;  // can't be a tree joint

      const Body& parent_body = graph.get_body(joint.parent_body_num());
      const Body& child_body = graph.get_body(joint.child_body_num());

      const MobilizedBodyNum parent_mobod_num =
          body_to_mobod_num(joint.parent_body_num());
      const MobilizedBodyNum child_mobod_num =
          body_to_mobod_num(joint.child_body_num());

      const MobilizedBody& parent_mobod =
          mobod(parent_mobod_num);
      const MobilizedBody& child_mobod =
          mobod(child_mobod_num);

      // Exactly one of parent or child body must already be in the tree.
      if (!(parent_mobod.is_in_tree() ^ child_mobod.is_in_tree())) continue;

      // Stop growing this branch after the current level so we can balance
      // branch lengths. However, if our outboard body is massless we don't
      // want to risk it ending up as terminal so keep going in that case.
      BodyIndex inboard_body_num, outboard_body_num;
      MobilizedBodyNum inboard_mobod_num,
          outboard_mobod_num;
      if (parent_mobod.is_in_tree()) {
        if (!child_body.must_be_nonterminal() &&
            parent_mobod.level() != level - 1)
          continue;  // not time yet
        inboard_body_num = joint.parent_body_num();
        outboard_body_num = joint.child_body_num();
        inboard_mobod_num = parent_mobod_num;
        outboard_mobod_num = child_mobod_num;
      } else {  // child is in tree
        if (!parent_body.must_be_nonterminal() &&
            child_mobod.level() != level - 1)
          continue;  // not time yet
        inboard_body_num = joint.child_body_num();
        outboard_body_num = joint.parent_body_num();
        inboard_mobod_num = child_mobod_num;
        outboard_mobod_num = parent_mobod_num;
      }

      MobilizerNum mobilizer_num =
          AddMobilizerFromJoint(joint_num, inboard_mobod_num,
                                outboard_mobod_num, child_mobod.is_in_tree());

      joints_added.insert(joint_num);
      any_mobilizer_added = true;

      // We just made joint joint_num a mobilizer. If its outboard body
      // is massless and the mobilizer was not a weld, we need to keep
      // extending this branch of the tree until we can end it with a
      // massful body.
      const Body& outboard_body = graph.get_body(outboard_body_num);
      const JointType& joint_type =
          graph.get_joint_type(joint.joint_type_num());
      if (joint_type.num_v() == 0 || !outboard_body.must_be_nonterminal())
        continue;

      // Pick a further-outboard body with mass and extend branch to it.
      // Prefer forward-direction joints if possible.
      // If only massless outboard bodies are available, add one and
      // keep going.
      while (true) {
        // The outboard mobilized body of the mobilizer we just added will be
        // the next one we connect. Ideally it will serve as inboard, but we
        // may have to reverse.
        const BodyIndex body_num = mobod_to_body_num(
            get_mobilizer(mobilizer_num).outboard_mobod_num());

        const JointIndex joint_forward_num =
            FindHeaviestUnassignedForwardJoint(body_num, graph);
        if (joint_forward_num.is_valid()) {
          const Joint& joint_forward = graph.get_joint(joint_forward_num);
          const BodyIndex fwd_outboard_body_num =
              joint_forward.child_body_num();
          if (!graph.get_body(fwd_outboard_body_num).must_be_nonterminal()) {
            mobilizer_num = AddMobilizerFromJoint(
                joint_forward_num, body_to_mobod_num(body_num),
                body_to_mobod_num(fwd_outboard_body_num),
                false /* not reversed */);
            joints_added.insert(joint_forward_num);
            break;
          }
        }

        const JointIndex joint_reverse_num =
            FindHeaviestUnassignedReverseJoint(body_num, graph);
        if (joint_reverse_num.is_valid()) {
          const Joint& joint_reverse = graph.get_joint(joint_reverse_num);
          const BodyIndex rev_outboard_body_num =
              joint_reverse.parent_body_num();
          if (!graph.get_body(rev_outboard_body_num).must_be_nonterminal()) {
            mobilizer_num = AddMobilizerFromJoint(
                joint_reverse_num, body_to_mobod_num(body_num),
                body_to_mobod_num(rev_outboard_body_num), true /* reversed */);
            joints_added.insert(joint_reverse_num);
            break;
          }
        }

        // Couldn't find a massful body to add. Add another massless
        // body (if there is one) and keep trying.
        if (joint_forward_num.is_valid()) {
          mobilizer_num = AddMobilizerFromJoint(
              joint_forward_num, body_to_mobod_num(body_num),
              body_to_mobod_num(joint.child_body_num()),
              false /* not reversed */);
          joints_added.insert(joint_forward_num);
          continue;
        }
        if (joint_reverse_num.is_valid()) {
          mobilizer_num = AddMobilizerFromJoint(
              joint_reverse_num, body_to_mobod_num(body_num),
              body_to_mobod_num(joint.parent_body_num()), true /* reversed */);
          joints_added.insert(joint_reverse_num);
          continue;
        }

        // Uh oh. Nothing outboard of the massless body we just added.
        // We're ending this chain with a massless body; not good.
        throw std::runtime_error(
            "GrowTree(): algorithm produced an"
            " invalid tree containing a terminal massless body (" +
            graph.get_body(body_num).name() +
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
void MultibodyTreeModel::BreakLoops(const MultibodyGraphModeler& graph) {
  for (JointIndex joint_num(0); joint_num < num_joints(); ++joint_num) {
    const Joint& joint = graph.get_joint(joint_num);
    if (joint_info(joint_num).has_mobilizer()) continue;  // already done

    const BodyIndex parent_body_num = joint.parent_body_num();
    const BodyIndex child_body_num = joint.child_body_num();
    const MultibodyTreeModel::MobilizedBodyNum parent_mobod_num =
        body_to_mobod_num(parent_body_num);
    const MultibodyTreeModel::MobilizedBodyNum child_mobod_num =
        body_to_mobod_num(child_body_num);
    DRAKE_DEMAND(mobod(parent_mobod_num).is_in_tree() &&
                 mobod(child_mobod_num).is_in_tree());

    // Use a loop joint if there's a good one available.
    const JointType& joint_type = graph.get_joint_type(joint.joint_type_num());
    if (joint_type.have_good_joint_constraint_available()) {
      AddConstraintFromJoint(joint_num, parent_mobod_num, child_mobod_num);
      continue;
    }

    // No usable loop constraint for this type of joint. Add a new slave
    // body so we can use a mobilizer. (MobilizedBody references are invalidated
    // here since we're adding a new body -- don't reuse them.)

    // First assume we will be able to split the child, which is the usual case.
    BodyIndex split_body_num = child_body_num;
    BodyIndex inboard_body_num = parent_body_num;
    bool is_reversed = false;
    // If the child is World, choose the parent instead, reverse the mobilizer.
    if (child_body_num == graph.world_body_num()) {
      DRAKE_DEMAND(parent_body_num != graph.world_body_num());
      split_body_num = parent_body_num;
      inboard_body_num = child_body_num;  // That is, World.
      is_reversed = true;
    }
    const MultibodyTreeModel::MobilizedBodyNum split_mobod_num =
        body_to_mobod_num(split_body_num);
    const MultibodyTreeModel::MobilizedBodyNum inboard_mobod_num =
        body_to_mobod_num(inboard_body_num);
    const MultibodyTreeModel::MobilizedBodyNum slave_mobod_num =
        SplitBody(split_mobod_num);

    // Mobilize the slave body.
    AddMobilizerFromJoint(joint_num, inboard_mobod_num, slave_mobod_num,
                          is_reversed);

    // Add the weld loop constraint.
    std::string constraint_name = "#" + mobod(split_mobod_num).name() + "|" +
                                  mobod(slave_mobod_num).name();
    AddSlaveWeldConstraint(std::move(constraint_name), joint_num,
                           split_mobod_num, slave_mobod_num);
  }
}

//------------------------------------------------------------------------------
//                         ADD TO MULTIBODY TREE
//------------------------------------------------------------------------------

auto MultibodyTreeModel::AddJointType(std::string joint_type_name,
                                      void* joint_type_user_ref)
    -> JointTypeIndex {
  const JointTypeIndex joint_type_num(num_joint_types());
  joint_type_info_.push_back(
      JointTypeInfo(std::move(joint_type_name), joint_type_user_ref));
  return joint_type_num;
}

auto MultibodyTreeModel::AddBodyInfo(std::string body_name, void* body_user_ref)
    -> BodyIndex {
  const BodyIndex body_num(num_bodies());
  body_info_.push_back(BodyInfo(std::move(body_name), body_user_ref));
  return body_num;
}

auto MultibodyTreeModel::AddJointInfo(std::string joint_name,
                                      JointTypeIndex joint_type_num,
                                      void* joint_user_ref) -> JointIndex {
  const JointIndex joint_num(num_joints());
  joint_info_.push_back(
      JointInfo(std::move(joint_name), joint_type_num, joint_user_ref));
  return joint_num;
}

auto MultibodyTreeModel::AddMobodFromBody(BodyIndex body_num)
    -> MobilizedBodyNum {
  const MobilizedBodyNum mobod_num(num_mobods());
  mobods_.push_back(MobilizedBody(body_num, this));
  BodyInfo& body_info = body_info_[body_num];
  body_info.set_master_mobod_num(mobod_num);
  body_name_to_mobod_num_[body_info.body_name()] = mobod_num;

  // World goes in the tree right away.
  if (mobod_num == world_mobod_num()) mobods_.back().set_level(0);

  return mobod_num;
}

//------------------------------------------------------------------------------
//                       ADD MOBILIZER FROM JOINT
//------------------------------------------------------------------------------
auto MultibodyTreeModel::AddMobilizerFromJoint(
    JointIndex joint_num, MobilizedBodyNum inboard_mobod_num,
    MobilizedBodyNum outboard_mobod_num, bool is_reversed) -> MobilizerNum {
  MobilizedBody& inboard = get_mutable_body(inboard_mobod_num);
  MobilizedBody& outboard = get_mutable_body(outboard_mobod_num);
  DRAKE_DEMAND(inboard.is_in_tree());
  const int level = inboard.level() + 1;
  const MobilizerNum mobilizer_num =
      MobilizerNum(num_mobilizers());  // next available
  mobilizers_.push_back(Mobilizer(joint_num, level, inboard_mobod_num,
                                  outboard_mobod_num, is_reversed, this));
  joint_info_[joint_num].set_mobilizer_num(mobilizer_num);

  outboard.set_level(level);
  outboard.set_mobilizer_num(mobilizer_num);
  inboard.add_outboard_mobilizer_num(mobilizer_num);
  return mobilizer_num;
}

//------------------------------------------------------------------------------
//                          CONNECT BODY TO WORLD
//------------------------------------------------------------------------------
// Connect the given body to World by a free mobilizer with inboard body
// World and outboard body the given body. This makes the body a base body
// (level 1 body) for some subtree of the multibody graph.
auto MultibodyTreeModel::ConnectBodyToWorld(MobilizedBodyNum mobod_num,
                                            bool is_static) -> MobilizerNum {
  MobilizedBody& mobod = get_mutable_body(mobod_num);
  DRAKE_ASSERT(!mobod.is_in_tree());
  MobilizedBody& world = get_mutable_body(world_mobod_num());

  const std::string mobilizer_name =
      "#" + world_body_name() + "_" + mobod.name();

  const MobilizerNum mobilizer_num(num_mobilizers());
  mobilizers_.push_back(
      Mobilizer(std::move(mobilizer_name),
                is_static ? weld_joint_type_num() : free_joint_type_num(),
                1 /* level */, world_mobod_num(), mobod_num, this));

  mobod.set_level(1);
  mobod.set_mobilizer_num(mobilizer_num);
  world.add_outboard_mobilizer_num(mobilizer_num);
  return mobilizer_num;
}

//------------------------------------------------------------------------------
//                       ADD CONSTRAINT FROM JOINT
//------------------------------------------------------------------------------
auto MultibodyTreeModel::AddConstraintFromJoint(
    JointIndex joint_num, MobilizedBodyNum parent_mobod_num,
    MobilizedBodyNum child_mobod_num) -> ConstraintNum {
  MobilizedBody& parent = get_mutable_body(parent_mobod_num);
  MobilizedBody& child = get_mutable_body(child_mobod_num);
  DRAKE_DEMAND(parent.is_in_tree() && child.is_in_tree());
  const ConstraintNum constraint_num = ConstraintNum(num_constraints());
  constraints_.push_back(
      Constraint(joint_num, parent_mobod_num, child_mobod_num, this));
  joint_info_[joint_num].set_constraint_num(constraint_num);
  parent.add_joint_constraint_num(constraint_num);
  child.add_joint_constraint_num(constraint_num);
  return constraint_num;
}

auto MultibodyTreeModel::AddSlaveWeldConstraint(
    std::string constraint_name, JointIndex joint_num,
    MobilizedBodyNum master_mobod_num, MobilizedBodyNum slave_mobod_num)
    -> ConstraintNum {
  MobilizedBody& master = get_mutable_body(master_mobod_num);
  MobilizedBody& slave = get_mutable_body(slave_mobod_num);
  DRAKE_DEMAND(master.is_in_tree() && slave.is_in_tree());
  const ConstraintNum constraint_num = ConstraintNum(num_constraints());
  constraints_.push_back(Constraint(constraint_name, weld_joint_type_num(),
                                    joint_num, master_mobod_num,
                                    slave_mobod_num, this));
  master.add_joint_constraint_num(constraint_num);
  slave.add_joint_constraint_num(constraint_num);
  return constraint_num;
}

//------------------------------------------------------------------------------
//                                SPLIT BODY
//------------------------------------------------------------------------------
// Create a new slave body for the given master, and add it to the list of
// bodies. Does not create the related loop constraint. The body
// number assigned to the slave is returned.
auto MultibodyTreeModel::SplitBody(MobilizedBodyNum master_mobod_num)
    -> MobilizedBodyNum {
  // First slave is number 1, slave 0 is the master.
  const MobilizedBody& master = mobod(master_mobod_num);
  std::string slave_name =
      "#" + master.name() + "_slave_" + std::to_string(master.num_slaves() + 1);
  const MobilizedBodyNum slave_mobod_num(num_mobods());  // next available
  mobods_.push_back(
      MobilizedBody(std::move(slave_name), master_mobod_num, this));
  // There is no corresponding BodyInfo.

  // Careful -- master reference above is invalid now since we pushed a
  // new MobilizedBody.
  get_mutable_body(master_mobod_num).add_slave_mobod_num(slave_mobod_num);
  // No name lookup for slave bodies.
  return slave_mobod_num;
}

//------------------------------------------------------------------------------
//                               DUMP GRAPH
//------------------------------------------------------------------------------
void MultibodyTreeModel::DumpTreeModel(std::ostream& out) const {
  // constexpr int kBufSize = 1024;
  out << "\nMULTIBODY TREE MODEL\n";
  out << "--------------------\n";
  out << "\n" << num_mobods() << " MOBILIZED BODIES:\n";
  out << "body@lev: mob name\n";
  for (MobilizedBodyNum mobod_num(0); mobod_num < num_mobods(); ++mobod_num) {
    const MobilizedBody& mobod = this->mobod(mobod_num);
    const std::string inb_mob = mobod.is_world_body()
                                    ? std::string("--")
                                    : fmt::format("{}", mobod.mobilizer_num());
    out << fmt::format("{}{:<3}@{:3}: {:3} {}",
                       mobod.is_master() ? "M" : (mobod.is_slave() ? "S" : " "),
                       mobod_num, mobod.level(), inb_mob, mobod.name());
    if (mobod.is_slave())
      out << fmt::format(" master={}", mobod.master_mobod_num());
    if (mobod.num_outboard_mobilizers()) {
      out << "  outboard mob=[";
      for (int j = 0; j < mobod.num_outboard_mobilizers(); ++j)
        out << " " << mobod.outboard_mobilizers()[j];
      out << "]";
    }
    if (mobod.num_slaves()) {
      out << "  slaves=[";
      for (int j = 0; j < mobod.num_slaves(); ++j)
        out << " " << mobod.slaves()[j];
      out << "]";
    }
    out << "\n";
  }

  out << "\n" << num_mobilizers() << " MOBILIZERS:\n";
  for (MobilizerNum i(0); i < num_mobilizers(); ++i) {
    const Mobilizer& mo = get_mobilizer(i);
    const MobilizedBody& inb = mobod(mo.inboard_mobod_num());
    const MobilizedBody& outb = mobod(mo.outboard_mobod_num());
    const std::string joint_num = mo.joint_num().is_valid()
                                      ? fmt::format("{}", mo.joint_num())
                                      : std::string("--");
    out << fmt::format("{:2} {:2}: {:20} {:20}->{:<20} {:10} {:2} {:3}\n", i,
                       mo.level(), mo.name(), inb.name(), outb.name(),
                       mo.get_joint_type_name(), joint_num,
                       (mo.is_reversed_from_joint() ? "REV" : ""));
  }

  out << "\n" << num_constraints() << " LOOP CONSTRAINTS:\n";
  for (ConstraintNum i(0); i < num_constraints(); ++i) {
    const Constraint& lc = get_constraint(i);
    const MobilizedBody& parent = mobod(lc.parent_mobod_num());
    const MobilizedBody& child = mobod(lc.child_mobod_num());
    out << fmt::format("{}: {}({}) parent={} child={} joint_num={}\n", i,
                       lc.name(), lc.constraint_type_name(), parent.name(),
                       child.name(), lc.joint_num());
  }

  std::vector<MobilizedBodyNum> base_bodies = FindBaseBodies();
  out << "\n" << base_bodies.size() << " BASE BODIES:\n";
  for (MobilizedBodyNum mobod_num : base_bodies)
    out << " " << mobod(mobod_num).name();
  out << "\n---------- END OF MULTIBODY TREE MODEL.\n\n";
}

//------------------------------------------------------------------------------
//                            FIND BASE BODIES
//------------------------------------------------------------------------------
auto MultibodyTreeModel::FindBaseBodies() const
    -> std::vector<MobilizedBodyNum> {
  std::vector<MobilizedBodyNum> base_bodies;
  for (const auto& mobilizer : mobilizers_) {
    if (mobilizer.level() != 1) continue;
    base_bodies.push_back(mobilizer.outboard_mobod_num());
  }
  return base_bodies;
}

auto MultibodyTreeModel::FindBaseBody(MobilizedBodyNum mobod_num) const
    -> MobilizedBodyNum {
  std::vector<MobilizedBodyNum> path_bodies = FindPathToWorld(mobod_num);
  return path_bodies.empty() ? MobilizedBodyNum() : path_bodies.back();
}

//------------------------------------------------------------------------------
//                            FIND PATH TO WORLD
//------------------------------------------------------------------------------
auto MultibodyTreeModel::FindPathToWorld(MobilizedBodyNum mobod_num) const
    -> std::vector<MobilizedBodyNum> {
  DRAKE_DEMAND(mobod_num.is_valid());
  std::vector<MobilizedBodyNum> path_bodies;
  // Note that nothing gets pushed if mobod_num is World.
  while (mobod(mobod_num).level() > 0) {
    path_bodies.push_back(mobod_num);
    const Mobilizer& mobilizer =
        get_mobilizer(mobod(mobod_num).mobilizer_num());
    mobod_num = mobilizer.inboard_mobod_num();
  }
  return path_bodies;
}

//------------------------------------------------------------------------------
//                      BODIES ARE CONNECTED BY MOBILIZER
//------------------------------------------------------------------------------
// Return true if there is a mobilizer between two bodies given by body number,
// regardless of inboard/outboard ordering.
bool MultibodyTreeModel::MobodsAreConnectedByMobilizer(
    MobilizedBodyNum mobod1_num, MobilizedBodyNum mobod2_num) const {
  const MobilizedBody& mobod1 = mobod(mobod1_num);
  if (get_mobilizer(mobod1.mobilizer_num()).inboard_mobod_num() == mobod2_num)
    return true;  // Body2 is mobod1's parent.

  const MobilizedBody& mobod2 = mobod(mobod2_num);
  if (get_mobilizer(mobod2.mobilizer_num()).inboard_mobod_num() == mobod1_num)
    return true;  // Body1 is mobod2's parent.

  return false;  // Otherwise they aren't connected.
}

//------------------------------------------------------------------------------
//                BODIES ARE CONNECTED BY JOINT CONSTRAINT
//------------------------------------------------------------------------------
// Return true if there is a joint constraint between two bodies given by
// body number, regardless of parent/child ordering.
bool MultibodyTreeModel::MobodsAreConnectedByJointConstraint(
    MobilizedBodyNum mobod1_num, MobilizedBodyNum mobod2_num) const {
  const MobilizedBody& mobod1 = mobod(mobod1_num);
  for (ConstraintNum constraint_num : mobod1.joint_constraints()) {
    const Constraint& constraint = this->get_constraint(constraint_num);
    const MobilizedBodyNum parent = constraint.parent_mobod_num();
    const MobilizedBodyNum child = constraint.child_mobod_num();
    DRAKE_DEMAND(parent == mobod1_num || child == mobod1_num);
    if (parent == mobod2_num || child == mobod2_num) {
      return true;
    }
  }

  return false;  // They aren't connected.
}

}  // namespace multibody
}  // namespace drake
