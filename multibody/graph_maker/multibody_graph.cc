/* Adapted for Drake from Simbody's MultibodyGraphMaker class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/multibody/graph_maker/multibody_graph.h"

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

std::string to_string(MultibodyGraph::LinkFlags flags) {
  std::string out;
  if (flags == 0) {
    out = "default";
  } else {
    add_if_flag(flags, MultibodyGraph::kStaticLink, "static", &out);
    add_if_flag(flags, MultibodyGraph::kMustBeBaseBody, "must_be_base_body",
                &out);
    add_if_flag(flags, MultibodyGraph::kMustNotBeTerminalBody,
                "must_be_nonterminal", &out);
  }
  return out;
}

std::string to_string(MultibodyGraph::JointFlags flags) {
  std::string out;
  if (flags == 0) {
    out = "default";
  } else {
    add_if_flag(flags, MultibodyGraph::kMustBeConstraint, "must_be_constraint",
                &out);
  }
  return out;
}

std::string to_string(MultibodyGraph::JointTypeFlags flags) {
  std::string out;
  if (flags == 0) {
    out = "default";
  } else {
    add_if_flag(flags, MultibodyGraph::kOkToUseAsJointConstraint,
                "ok_to_use_as_loop_joint", &out);
  }
  return out;
}

//------------------------------------------------------------------------------
//                         ADD TO MULTIBODY GRAPH
//------------------------------------------------------------------------------

auto MultibodyGraph::AddJointType(std::string joint_type_name,
                                  void* joint_type_user_ref) -> JointTypeNum {
  const JointTypeNum joint_type_num(num_joint_types());
  joint_type_info_.push_back(
      JointTypeInfo(std::move(joint_type_name), joint_type_user_ref));
  return joint_type_num;
}

auto MultibodyGraph::AddLinkInfo(std::string link_name, void* link_user_ref)
    -> LinkNum {
  const LinkNum link_num(num_links());
  link_info_.push_back(LinkInfo(std::move(link_name), link_user_ref));
  return link_num;
}

auto MultibodyGraph::AddJointInfo(std::string joint_name,
                                  JointTypeNum joint_type_num,
                                  void* joint_user_ref) -> JointNum {
  const JointNum joint_num(num_joints());
  joint_info_.push_back(
      JointInfo(std::move(joint_name), joint_type_num, joint_user_ref));
  return joint_num;
}

auto MultibodyGraph::AddBodyFromLink(LinkNum link_num) -> BodyNum {
  const BodyNum body_num(num_bodies());
  bodies_.push_back(Body(link_num, this));
  LinkInfo& link_info = link_info_[link_num];
  link_info.set_master_body_num(body_num);
  link_name_to_body_num_[link_info.link_name()] = body_num;

  // World goes in the tree right away.
  if (body_num == world_body_num())
    bodies_.back().set_level(0);

  return body_num;
}

//------------------------------------------------------------------------------
//                       ADD MOBILIZER FROM JOINT
//------------------------------------------------------------------------------
auto MultibodyGraph::AddMobilizerFromJoint(JointNum joint_num,
                                           BodyNum inboard_body_num,
                                           BodyNum outboard_body_num,
                                           bool is_reversed) -> MobilizerNum {
  Body& inboard = get_mutable_body(inboard_body_num);
  Body& outboard = get_mutable_body(outboard_body_num);
  DRAKE_DEMAND(inboard.is_in_tree());
  const int level = inboard.level() + 1;
  const MobilizerNum mobilizer_num =
      MobilizerNum(num_mobilizers());  // next available
  mobilizers_.push_back(Mobilizer(joint_num, level, inboard_body_num,
                                  outboard_body_num, is_reversed, this));
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
auto MultibodyGraph::ConnectBodyToWorld(BodyNum body_num, bool is_static)
    -> MobilizerNum {
  Body& body = get_mutable_body(body_num);
  DRAKE_ASSERT(!body.is_in_tree());
  Body& world = get_mutable_body(world_body_num());

  const std::string mobilizer_name =
      "#" + world_body_name() + "_" + body.name();

  const MobilizerNum mobilizer_num(num_mobilizers());
  mobilizers_.push_back(
      Mobilizer(std::move(mobilizer_name),
                is_static ? weld_joint_type_num() : free_joint_type_num(),
                1 /* level */, world_body_num(), body_num, this));

  body.set_level(1);
  body.set_mobilizer_num(mobilizer_num);
  world.add_outboard_mobilizer_num(mobilizer_num);
  return mobilizer_num;
}

//------------------------------------------------------------------------------
//                       ADD CONSTRAINT FROM JOINT
//------------------------------------------------------------------------------
auto MultibodyGraph::AddConstraintFromJoint(JointNum joint_num,
                                            BodyNum parent_body_num,
                                            BodyNum child_body_num)
    -> ConstraintNum {
  Body& parent = get_mutable_body(parent_body_num);
  Body& child = get_mutable_body(child_body_num);
  DRAKE_DEMAND(parent.is_in_tree() && child.is_in_tree());
  const ConstraintNum constraint_num = ConstraintNum(num_constraints());
  constraints_.push_back(
      Constraint(joint_num, parent_body_num, child_body_num, this));
  joint_info_[joint_num].set_constraint_num(constraint_num);
  parent.add_joint_constraint_num(constraint_num);
  child.add_joint_constraint_num(constraint_num);
  return constraint_num;
}

auto MultibodyGraph::AddSlaveWeldConstraint(std::string constraint_name,
                                            JointNum joint_num,
                                            BodyNum master_body_num,
                                            BodyNum slave_body_num)
    -> ConstraintNum {
  Body& master = get_mutable_body(master_body_num);
  Body& slave = get_mutable_body(slave_body_num);
  DRAKE_DEMAND(master.is_in_tree() && slave.is_in_tree());
  const ConstraintNum constraint_num = ConstraintNum(num_constraints());
  constraints_.push_back(Constraint(constraint_name, weld_joint_type_num(),
                                        joint_num,
                                        master_body_num, slave_body_num, this));
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
auto MultibodyGraph::SplitBody(BodyNum master_body_num) -> BodyNum {
  // First slave is number 1, slave 0 is the master.
  const Body& master = body(master_body_num);
  std::string slave_name =
      "#" + master.name() + "_slave_" + std::to_string(master.num_slaves() + 1);
  const BodyNum slave_body_num(num_bodies());  // next available
  bodies_.push_back(Body(std::move(slave_name), master_body_num, this));
  // There is no corresponding LinkInfo.

  // Careful -- master reference above is invalid now since we pushed a
  // new Body.
  get_mutable_body(master_body_num).add_slave_body_num(slave_body_num);
  // No name lookup for slave bodies.
  return slave_body_num;
}

//------------------------------------------------------------------------------
//                               DUMP GRAPH
//------------------------------------------------------------------------------
void MultibodyGraph::DumpGraph(std::ostream& o) const {
  // constexpr int kBufSize = 1024;
  o << "\nMULTIBODY GRAPH\n";
  o << "---------------\n";
  o << "\n" << num_bodies() << " BODIES:\n";
  o << "body@lev: mob name\n";
  for (BodyNum body_num(0); body_num < num_bodies(); ++body_num) {
    const Body& body = this->body(body_num);
    const std::string inb_mob = body.is_world_body()
                                    ? std::string("--")
                                    : fmt::format("{}", body.mobilizer_num());
    o << fmt::format("{}{:<3}@{:3}: {:3} {}",
                     body.is_master() ? "M" : (body.is_slave() ? "S" : " "),
                     body_num, body.level(), inb_mob, body.name());
    if (body.is_slave()) o << fmt::format(" master={}", body.master_body_num());
    if (body.num_outboard_mobilizers()) {
      o << "  outboard mob=[";
      for (int j = 0; j < body.num_outboard_mobilizers(); ++j)
        o << " " << body.outboard_mobilizers()[j];
      o << "]";
    }
    if (body.num_slaves()) {
      o << "  slaves=[";
      for (int j = 0; j < body.num_slaves(); ++j) o << " " << body.slaves()[j];
      o << "]";
    }
    o << "\n";
  }

  o << "\n" << num_mobilizers() << " MOBILIZERS:\n";
  for (MobilizerNum i(0); i < num_mobilizers(); ++i) {
    const Mobilizer& mo = get_mobilizer(i);
    const Body& inb = body(mo.inboard_body_num());
    const Body& outb = body(mo.outboard_body_num());
    const std::string joint_num = mo.joint_num().is_valid()
                                ? fmt::format("{}", mo.joint_num())
                                : std::string("--");
    o << fmt::format("{:2} {:2}: {:20} {:20}->{:<20} {:10} {:2} {:3}\n", i,
                     mo.level(), mo.name(), inb.name(), outb.name(),
                     mo.get_joint_type_name(), joint_num,
                     (mo.is_reversed_from_joint() ? "REV" : ""));
  }

  o << "\n" << num_constraints() << " LOOP CONSTRAINTS:\n";
  for (ConstraintNum i(0); i < num_constraints(); ++i) {
    const Constraint& lc = get_constraint(i);
    const Body& parent = body(lc.parent_body_num());
    const Body& child = body(lc.child_body_num());
    o << fmt::format("{}: {}({}) parent={} child={} joint_num={}\n", i,
                     lc.name(), lc.constraint_type_name(), parent.name(),
                     child.name(), lc.joint_num());
  }

  std::vector<BodyNum> base_bodies = FindBaseBodies();
  o << "\n" << base_bodies.size() << " BASE BODIES:\n";
  for (BodyNum body_num : base_bodies) o << " " << body(body_num).name();
  o << "\n---------- END OF MULTIBODY GRAPH.\n\n";
}

//------------------------------------------------------------------------------
//                            FIND BASE BODIES
//------------------------------------------------------------------------------
auto MultibodyGraph::FindBaseBodies() const -> std::vector<BodyNum> {
  std::vector<BodyNum> base_bodies;
  for (const auto& mobilizer : mobilizers_) {
    if (mobilizer.level() != 1) continue;
    base_bodies.push_back(mobilizer.outboard_body_num());
  }
  return base_bodies;
}

auto MultibodyGraph::FindBaseBody(BodyNum body_num) const -> BodyNum {
  std::vector<BodyNum> path_bodies = FindPathToWorld(body_num);
  return path_bodies.empty() ? BodyNum() : path_bodies.back();
}

//------------------------------------------------------------------------------
//                            FIND PATH TO WORLD
//------------------------------------------------------------------------------
auto MultibodyGraph::FindPathToWorld(BodyNum body_num) const
    -> std::vector<BodyNum> {
  DRAKE_DEMAND(body_num.is_valid());
  std::vector<BodyNum> path_bodies;
  // Note that nothing gets pushed if body_num is World.
  while (body(body_num).level() > 0) {
    path_bodies.push_back(body_num);
    const Mobilizer& mobilizer = get_mobilizer(body(body_num).mobilizer_num());
    body_num = mobilizer.inboard_body_num();
  }
  return path_bodies;
}

//------------------------------------------------------------------------------
//                      BODIES ARE CONNECTED BY MOBILIZER
//------------------------------------------------------------------------------
// Return true if there is a mobilizer between two bodies given by body number,
// regardless of inboard/outboard ordering.
bool MultibodyGraph::BodiesAreConnectedByMobilizer(BodyNum body1_num,
                                                   BodyNum body2_num) const {
  const Body& body1 = body(body1_num);
  if (get_mobilizer(body1.mobilizer_num()).inboard_body_num() == body2_num)
    return true;  // Body2 is body1's parent.

  const Body& body2 = body(body2_num);
  if (get_mobilizer(body2.mobilizer_num()).inboard_body_num() == body1_num)
    return true;  // Body1 is body2's parent.

  return false;  // Otherwise they aren't connected.
}

//------------------------------------------------------------------------------
//                BODIES ARE CONNECTED BY JOINT CONSTRAINT
//------------------------------------------------------------------------------
// Return true if there is a joint constraint between two bodies given by
// body number, regardless of parent/child ordering.
bool MultibodyGraph::BodiesAreConnectedByJointConstraint(
    BodyNum body1_num, BodyNum body2_num) const {
  const Body& body1 = body(body1_num);
  for (ConstraintNum constraint_num : body1.joint_constraints()) {
    const Constraint& constraint =
        this->get_constraint(constraint_num);
    const BodyNum parent = constraint.parent_body_num();
    const BodyNum child = constraint.child_body_num();
    DRAKE_DEMAND(parent == body1_num || child == body1_num);
    if (parent == body2_num || child == body2_num) {
      return true;
    }
  }

  return false;  // They aren't connected.
}

}  // namespace multibody
}  // namespace drake
