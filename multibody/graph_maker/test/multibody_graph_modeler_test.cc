/* Adapted for Drake from Simbody's MultibodyGraphModeler class.
Portions copyright (c) 2017 Stanford University and the Authors.
Authors: Chris Dembia

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/multibody/graph_maker/multibody_graph_modeler.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/graph_maker/multibody_tree_model.h"

namespace drake {
namespace multibody {
namespace {
// These are used for emphasis that the body is or is not massless.
constexpr double kMassless = 0.0;
constexpr double kNonzeroMass = 1.8;

using LinkNum = MultibodyTreeModel::LinkNum;
using JointNum = MultibodyTreeModel::JointNum;
using JointTypeNum = MultibodyTreeModel::JointTypeNum;

using BodyNum = MultibodyTreeModel::BodyNum;
using MobilizerNum = MultibodyTreeModel::MobilizerNum;
using ConstraintNum = MultibodyTreeModel::ConstraintNum;

// Test a straightforward serial chain of massful links connected by
// pin joints: world->link1->link2->link3->link4->link5
// Should produce the obvious multibody tree.
GTEST_TEST(MultibodyGraphModeler, SerialChain) {
  MultibodyGraphModeler maker(false);  // Don't use Drake defaults.
  maker.AddLink("world");
  maker.RegisterJointType("pin", 1, 1);

  maker.AddLink("link1");  // Default is massful links.
  maker.AddLink("link2");
  maker.AddLink("link3");
  maker.AddLink("link4");
  maker.AddLink("link5");

  maker.AddJoint("pin1", "pin", "world", "link1");
  maker.AddJoint("pin2", "pin", "link1", "link2");
  maker.AddJoint("pin3", "pin", "link2", "link3");
  maker.AddJoint("pin4", "pin", "link3", "link4");
  maker.AddJoint("pin5", "pin", "link4", "link5");

  auto graph = maker.MakeTreeModel();

  // First, some basic tests.
  EXPECT_EQ(maker.num_joint_types(), 3);  // include weld and free.
  EXPECT_EQ(maker.FindJointTypeNum("weld"), 0);
  EXPECT_EQ(maker.FindJointTypeNum("free"), 1);
  EXPECT_EQ(maker.FindJointTypeNum("pin"), 2);  // 3rd joint.
  EXPECT_EQ(maker.world_link_name(), "world");
  EXPECT_EQ(maker.num_links(), 6);
  EXPECT_EQ(maker.FindLinkNum("link3"), 3);  // world is 0th body.
  EXPECT_EQ(maker.num_joints(), 5);
  EXPECT_EQ(maker.FindJointNum("pin4"), 3);

  EXPECT_EQ(graph->num_constraints(), 0);

  EXPECT_EQ(graph->num_mobilizers(), 5);
  for (MobilizerNum i(0); i < graph->num_mobilizers(); ++i) {
    const auto& mobilizer = graph->get_mobilizer(i);
    EXPECT_FALSE(mobilizer.is_added_base_mobilizer());
    EXPECT_FALSE(mobilizer.is_slave_mobilizer());
    EXPECT_FALSE(mobilizer.is_reversed_from_joint());
    EXPECT_EQ(mobilizer.get_joint_type_name(), "pin");
    EXPECT_EQ(mobilizer.inboard_body_num(), i.to_int());
    EXPECT_EQ(mobilizer.outboard_body_num(), i + 1);
    EXPECT_EQ(mobilizer.num_fragments(), 1);

    EXPECT_EQ(mobilizer.level(), i + 1);
  }
}

// This test ensures that the graph maker produces the same topology for a
// 5-body chain of pin joints (world-link1-link2-link3-link4-link5) whether or
// not link2 is massless.
GTEST_TEST(MultibodyGraphModeler, IntermediateMasslessBody) {
  MultibodyGraphModeler maker;

  maker.AddLink("link1");
  maker.AddLink("link2", MultibodyTreeModel::kMustNotBeTerminalBody);
  maker.AddLink("link3");
  maker.AddLink("link4");
  maker.AddLink("link5");

  maker.AddJoint("pin1", "revolute", "world", "link1");
  maker.AddJoint("pin2", "revolute", "link1", "link2");
  maker.AddJoint("pin3", "revolute", "link2", "link3");
  maker.AddJoint("pin4", "revolute", "link3", "link4");
  maker.AddJoint("pin5", "revolute", "link4", "link5");

  maker.DumpInput(std::cout);
  auto graph = maker.MakeTreeModel();
  graph->DumpTreeModel(std::cout);

  EXPECT_EQ(graph->num_mobilizers(), 5);
  for (MobilizerNum i(0); i < graph->num_mobilizers(); ++i) {
    const auto& mobilizer = graph->get_mobilizer(i);
    EXPECT_FALSE(mobilizer.is_added_base_mobilizer());
    EXPECT_FALSE(mobilizer.is_slave_mobilizer());
    EXPECT_FALSE(mobilizer.is_reversed_from_joint());
    EXPECT_EQ(mobilizer.get_joint_type_name(), "revolute");
    EXPECT_EQ(mobilizer.num_fragments(), 1);

    // Here's the test that the resulting graph is as expected:
    EXPECT_EQ(mobilizer.level(), i + 1);
  }
}

// Terminal massless bodies are not allowed (unless welded to a massful body).
GTEST_TEST(MultibodyGraphModeler, TerminalMasslessBody) {
  MultibodyGraphModeler maker;

  maker.AddLink("link1", MultibodyTreeModel::kMustNotBeTerminalBody);
  maker.AddJoint("pin1", "revolute", "world", "link1");
  EXPECT_THROW(maker.MakeTreeModel(), std::runtime_error);

  // If the terminal massless body is welded, then there's no issue.
  maker.DeleteJoint("pin1");
  maker.AddJoint("pin1", "weld", "world", "link1");
  EXPECT_NO_THROW(maker.MakeTreeModel());
}

// This is a basic test for how MultibodyGraphModeler handles a system with a
// loop (four-bar linkage). We have the input four-bar linkage:
//    world-->link1-->link2-->link3-->world
// We get
//                                              REV
//    world-->link1-->link2-->#link3_slave  world-->link3
// and a weld constraint link3##link3_slave. Note that the link3-->world joint
// had to be reversed. See the next test for what should happen if link3 is
// massless.
GTEST_TEST(MultibodyGraphModeler, HasLoop) {
  MultibodyGraphModeler maker(false);  // Don't use Drake defaults.
  maker.RegisterJointType("pin", 1, 1);
  maker.AddLink("world");
  for (int i = 1; i <= 3; ++i)
    maker.AddLink("link" + std::to_string(i));

  maker.AddJoint("pin1", "pin", "world", "link1");
  maker.AddJoint("pin2", "pin", "link1", "link2");
  maker.AddJoint("pin3", "pin", "link2", "link3");
  maker.AddJoint("pin4", "pin", "link3", "world");

  maker.DumpInput(std::cout);
  auto graph = maker.MakeTreeModel();
  graph->DumpTreeModel(std::cout);

  EXPECT_EQ(maker.num_links(), 4);
  EXPECT_EQ(maker.num_joints(), 4);
  EXPECT_EQ(maker.num_joint_types(), 3);  // include weld and free.
  EXPECT_EQ(maker.FindJointTypeNum("weld"), 0);
  EXPECT_EQ(maker.FindJointTypeNum("free"), 1);
  EXPECT_EQ(maker.FindJointTypeNum("pin"), 2);  // 3rd joint type.
  EXPECT_EQ(maker.world_link_name(), "world");

  // Should have one more body than link because we had to split link3.
  // Should have one loop constraint; the loop is broken by splitting a body and
  // adding a weld constraint.
  EXPECT_EQ(graph->num_bodies(), 5);
  EXPECT_EQ(graph->body(BodyNum(4)).name(), "#link3_slave_1");

  EXPECT_EQ(graph->num_mobilizers(), 4);
  EXPECT_EQ(graph->num_constraints(), 1);
  // Body numbers are the same as their corresponding link numbers.
  EXPECT_EQ(graph->FindBaseBodies(),
            std::vector<BodyNum>({BodyNum(1), BodyNum(3)}));

  // Resulting mobilizers (breadth-first):
  // 0: pin1 world->link1
  // 1: pin4 world->link3 (reversed)
  // 2: pin2 link1->link2
  // 3: pin3 link2->#link3_slave_1

  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(0)).level(), 1);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(1)).level(), 1);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(2)).level(), 2);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(3)).level(), 3);
  for (MobilizerNum i(0); i < graph->num_mobilizers(); ++i) {
    const auto& mobilizer = graph->get_mobilizer(i);
    EXPECT_FALSE(mobilizer.is_added_base_mobilizer());
    // The 4th mobilizer is a slave mobilizer (child body is a slave).
    EXPECT_EQ(mobilizer.is_slave_mobilizer(), (i == 3));
    EXPECT_EQ(mobilizer.is_reversed_from_joint(), (i == 1));
    EXPECT_EQ(mobilizer.get_joint_type_name(), "pin");
    // Body 3 is split, and it's the outboard body for mobilizers 1 and 3.
    EXPECT_EQ(mobilizer.num_fragments(), (i == 1 || i == 3 ? 2 : 1));
  }

  // Check the loop weld constraint.
  const auto& constraint = graph->get_constraint(ConstraintNum(0));
  EXPECT_EQ(constraint.constraint_type_name(), "weld");
  EXPECT_EQ(constraint.parent_body_num(), 3);  // link3
  EXPECT_EQ(constraint.child_body_num(), 4);   // #link3_slave
  EXPECT_EQ(constraint.joint_num(), 2);  // Needed for the link2->link3 joint.
}

// This is the same 4-bar linkage as in the previous test, but with massless
// bodies that affect how we must construct the spanning tree. First, link3 is
// massless and thus ineligible to serve as a terminal body as it did above.
// Then we expect    world-->link1-->link2-->link3-->world
// to become
//                                REV
//    world-->link1-->link2  world-->link3-->#link2_slave
// and a weld constraint link2##link2_slave.
//
// Then we'll try link2 as the massless body, and expect
//                                        REV
//    world-->link1-->link2-->link3  world-->#link3_slave
// and a weld constraint link3##link3_slave. That structure avoids making
// massless link2 a terminal body.
GTEST_TEST(MultibodyGraphModeler, HasLoopWithMasslessBody) {
  MultibodyGraphModeler maker(false);  // Don't use Drake defaults.
  maker.RegisterJointType("pin", 1, 1);
  maker.AddLink("world");
  maker.AddLink("link1");
  maker.AddLink("link2");
  maker.AddLink("link3", MultibodyTreeModel::kMustNotBeTerminalBody);

  maker.AddJoint("pin1", "pin", "world", "link1");
  maker.AddJoint("pin2", "pin", "link1", "link2");
  maker.AddJoint("pin3", "pin", "link2", "link3");
  maker.AddJoint("pin4", "pin", "link3", "world");

  maker.DumpInput(std::cout);
  auto graph = maker.MakeTreeModel();
  graph->DumpTreeModel(std::cout);

  EXPECT_EQ(maker.num_links(), 4);
  EXPECT_EQ(maker.num_joints(), 4);

  // Should have an extra body and one loop constraint; the loop is broken by
  // splitting a body and adding a weld constraint.
  EXPECT_EQ(graph->num_bodies(), 5);  // link2 is split into two bodies.
  EXPECT_EQ(graph->body(BodyNum(4)).name(), "#link2_slave_1");

  EXPECT_EQ(graph->num_constraints(), 1);
  EXPECT_EQ(graph->num_mobilizers(), 4);
  EXPECT_EQ(graph->FindBaseBodies(),
            std::vector<BodyNum>({BodyNum(1), BodyNum(3)}));

  // Resulting mobilizers (breadth-first):
  // 0: pin1 world->link1
  // 1: pin4 world->link3 (reversed)
  // 2: pin3 link3->link2 (reversed)
  // 3: pin2 link1->#link2_slave_1

  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(0)).level(), 1);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(1)).level(), 1);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(2)).level(), 2);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(3)).level(), 2);
  for (MobilizerNum i(0); i < graph->num_mobilizers(); ++i) {
    const auto& mobilizer = graph->get_mobilizer(i);
    EXPECT_FALSE(mobilizer.is_added_base_mobilizer());
    // The 4th mobilizer is a slave mobilizer (child body is a slave).
    EXPECT_EQ(mobilizer.is_slave_mobilizer(), (i == 3));
    EXPECT_EQ(mobilizer.is_reversed_from_joint(), (i == 1 || i == 2));
    EXPECT_EQ(mobilizer.get_joint_type_name(), "pin");
    // Body 2 is split, and it's the outboard body for mobilizers 2 and 3.
    EXPECT_EQ(mobilizer.num_fragments(), (i == 2 || i == 3 ? 2 : 1));
  }

  // Check the loop weld constraint.
  const auto& constraint = graph->get_constraint(ConstraintNum(0));
  EXPECT_EQ(constraint.constraint_type_name(), "weld");
  EXPECT_EQ(constraint.parent_body_num(), 2);  // link2
  EXPECT_EQ(constraint.child_body_num(), 4);   // #link2_slave
  EXPECT_EQ(constraint.joint_num(), 1);  // Needed for the link1->link2 joint.

  // Now make link3 massful and link2 massless.
  maker.ChangeLinkFlags("link3", MultibodyTreeModel::kDefaultLinkFlags);
  maker.ChangeLinkFlags("link2", MultibodyTreeModel::kMustNotBeTerminalBody);

  maker.DumpInput(std::cout);
  auto graph2 = maker.MakeTreeModel();
  graph2->DumpTreeModel(std::cout);

  // Resulting mobilizers:
  // 0: pin1 world->link1
  // 1: pin2 link1->link2
  // 2: pin3 link2->link3
  // 3: pin4 world->#link3_slave_1 (reversed)
  EXPECT_EQ(graph2->get_mobilizer(MobilizerNum(0)).level(), 1);
  EXPECT_EQ(graph2->get_mobilizer(MobilizerNum(1)).level(), 2);
  EXPECT_EQ(graph2->get_mobilizer(MobilizerNum(2)).level(), 3);
  EXPECT_EQ(graph2->get_mobilizer(MobilizerNum(3)).level(), 1);

  for (MultibodyTreeModel::MobilizerNum i(0); i < graph2->num_mobilizers(); ++i) {
    const auto& mobilizer = graph2->get_mobilizer(i);
    EXPECT_FALSE(mobilizer.is_added_base_mobilizer());
    // The 4th mobilizer is a slave mobilizer (child body is a slave).
    EXPECT_EQ(mobilizer.is_slave_mobilizer(), (i == 3));
    EXPECT_EQ(mobilizer.is_reversed_from_joint(), (i == 3));
    EXPECT_EQ(mobilizer.get_joint_type_name(), "pin");
    // Body 3 is split, and it's the outboard body for mobilizers 2 and 3.
    EXPECT_EQ(mobilizer.num_fragments(), (i == 2 || i == 3 ? 2 : 1));
  }

  // Check the loop weld constraint.
  const auto& constraint2 = graph2->get_constraint(ConstraintNum(0));
  EXPECT_EQ(constraint2.constraint_type_name(), "weld");
  EXPECT_EQ(constraint2.parent_body_num(), 3);  // link3
  EXPECT_EQ(constraint2.child_body_num(), 4);   // #link3_slave
  EXPECT_EQ(constraint2.joint_num(), maker.FindJointNum("pin4"));
}

// The base body choice heuristic is:
//   - pick the first body that has no parent
//   - if all bodies have a parent, then pick one with the most children
GTEST_TEST(MultibodyGraphModeler, ChooseBaseBody) {
  MultibodyGraphModeler maker(false);  // Don't use Drake defaults.
  maker.RegisterJointType("pin", 1, 1);
  maker.AddLink("ground");  // Instead of "world".

  // First system (arrows point from parent to child):
  //                                      -->link5
  //       link1     link2<--link3-->link4-->link6
  //                         ^^^^         -->link7
  // link1 and link3 are parent-only. We expect these to be processed
  // in body order, so link1 gets the first free joint. Then, link3 should be
  // chosen. Note that link4 has the most children but shouldn't be
  // picked as a base body since it is also a child.

  // Define bodies such that bodyi has body number i.
  for (int i = 1; i <= 7; ++i) {
    maker.AddLink("link" + std::to_string(i));
  }
  maker.AddJoint("joint0", "pin", "link3", "link2");
  maker.AddJoint("joint1", "pin", "link3", "link4");
  maker.AddJoint("joint2", "pin", "link4", "link5");
  maker.AddJoint("joint3", "weld", "link4", "link6");
  maker.AddJoint("joint4", "pin", "link4", "link7");

  maker.DumpInput(std::cout);
  auto graph = maker.MakeTreeModel();
  graph->DumpTreeModel(std::cout);

  const MobilizerNum zero(0), one(1);
  EXPECT_EQ(graph->get_mobilizer(zero).get_joint_type_name(), "free");
  EXPECT_EQ(graph->get_mobilizer(zero).inboard_body_num(),
            graph->FindLinkBodyNum("ground"));  // Should be 0.
  EXPECT_EQ(graph->get_mobilizer(zero).outboard_body_num(), 1);

  EXPECT_EQ(graph->get_mobilizer(one).get_joint_type_name(), "free");
  EXPECT_EQ(graph->get_mobilizer(one).inboard_body_num(), 0);
  EXPECT_EQ(graph->get_mobilizer(one).outboard_body_num(), 3);

  // Now add a joint from link7 to link3:
  //                                       -->link5
  //        link1     link2<--link3-->link4-->link6
  //                            ^     ^^^^ -->link7
  //                            |               |
  //                            +---------------+
  // Now link1 is the only parent-only body so should be chosen as a base body
  // first. But then another base body must be chosen and it has to be link4
  // since there is no remaining parent-only body, and link4 has the most
  // children. So the result should be:
  //                                           REV
  //   ground --> link1       ground --> link4 --> link3 --> link2
  //                                    🡧  🡣  🡦
  //                                link5 link6 link7 --> #link3_slave
  //
  // where "REV" indicates that the mobilizer had to be reversed from the
  // input joint. There should be a loop weld constraint between link3 and
  // its slave.

  maker.AddJoint("joint5", "pin", "link7", "link3");
  maker.DumpInput(std::cout);
  auto graph2 = maker.MakeTreeModel();
  graph2->DumpTreeModel(std::cout);

  EXPECT_EQ(graph2->num_mobilizers(), 8);
  EXPECT_EQ(graph2->num_bodies(), 9);  // Added one slave.
  EXPECT_EQ(graph2->num_constraints(), 1);

  EXPECT_EQ(graph2->get_mobilizer(zero).get_joint_type_name(), "free");
  EXPECT_EQ(graph2->get_mobilizer(zero).inboard_body_num(), 0);
  EXPECT_EQ(graph2->get_mobilizer(zero).outboard_body_num(), 1);

  EXPECT_EQ(graph2->get_mobilizer(one).get_joint_type_name(), "free");
  EXPECT_EQ(graph2->get_mobilizer(one).inboard_body_num(), 0);
  EXPECT_EQ(graph2->get_mobilizer(one).outboard_body_num(), 4);
}

// Test that a multiloop system can be resolved by chopping up one input body
// into more than one slave.
GTEST_TEST(MultibodyGraphModeler, MultipleSlaves) {
  MultibodyGraphModeler maker;  // Use Drake defaults.

  // A three-loop system with link3 involved in every loop.
  //       +-0-> link2
  //       |      3🡣
  // link1 +-1-> link3 <--6---+       (Joint numbers shown -- 3, 4, and 6
  //       |      4🡡          |        will need loop-breaking.)
  //       +-2-> link4 -5-> link5
  //
  // link1 should get chosen as the base body, and link3 should get chopped
  // into four pieces (mobilizer numbers shown):
  //
  //                  +-1-> link2 -4-> link3_slave1          Weld constraints
  //                  |                                          ## slave1 0
  // world -0-> link1 +-2-> link3                          link3 ## slave2 1
  //                  |                                          ## slave3 2
  //                  +-3-> link4 -5-> link3_slave2
  //                              -6-> link5 -7-> link3_slave3
  //
  // There should be a loop weld constraint connecting link3 to each of its
  // slaves.

  for (int i = 1; i <= 5; ++i) {
    maker.AddLink("link" + std::to_string(i));
  }
  maker.AddJoint("joint0", "revolute", "link1", "link2");
  maker.AddJoint("joint1", "revolute", "link1", "link3");
  maker.AddJoint("joint2", "revolute", "link1", "link4");
  maker.AddJoint("joint3", "revolute", "link2", "link3");
  maker.AddJoint("joint4", "revolute", "link4", "link3");
  maker.AddJoint("joint5", "revolute", "link4", "link5");
  maker.AddJoint("joint6", "revolute", "link5", "link3");

  // Before generating the graph we have just what we input.
  EXPECT_EQ(maker.num_links(), 6);   // World plus the above five.
  EXPECT_EQ(maker.num_joints(), 7);  // No free joint to World yet.

  maker.DumpInput(std::cout);
  auto graph = maker.MakeTreeModel();
  graph->DumpTreeModel(std::cout);

  const LinkNum link3_num = maker.FindLinkNum("link3");
  const MultibodyGraphModeler::Link& link3 = maker.get_link(link3_num);
  EXPECT_EQ(link3.num_children(), 0);  // As input.
  EXPECT_EQ(link3.num_parents(), 4);

  const MultibodyTreeModel::Body& body3 = graph->link_to_body(link3_num);

  EXPECT_TRUE(body3.is_master());
  EXPECT_FALSE(body3.is_slave());
  EXPECT_EQ(body3.num_fragments(), 4);
  EXPECT_EQ(body3.num_slaves(), 3);

  EXPECT_EQ(graph->num_bodies(), 9);  // Added three slave bodies.
  EXPECT_EQ(graph->body(BodyNum(6)).name(), "#link3_slave_1");
  EXPECT_EQ(graph->body(BodyNum(7)).name(), "#link3_slave_2");
  EXPECT_EQ(graph->body(BodyNum(8)).name(), "#link3_slave_3");

  EXPECT_EQ(graph->num_mobilizers(), 8);
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(7)).name(), "joint6");
  EXPECT_EQ(graph->num_constraints(), 3);

  for (ConstraintNum i(0); i < graph->num_constraints(); ++i) {
    const int expected_joint_num[] = {3, 4, 6};
    const auto& loop_constraint = graph->get_constraint(i);
    EXPECT_EQ(loop_constraint.constraint_type_name(),
              maker.get_weld_joint_type_name());
    EXPECT_EQ(loop_constraint.joint_num(), expected_joint_num[i]);
    EXPECT_EQ(loop_constraint.parent_body_num(),
              graph->FindLinkBodyNum("link3"));
    EXPECT_EQ(loop_constraint.child_body_num(), 6 + i);  // One of the slaves.
  }

  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(0)).get_joint_type_name(),
            "free");
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(0)).inboard_body_num(),
            graph->FindLinkBodyNum("world"));
  EXPECT_EQ(graph->get_mobilizer(MobilizerNum(0)).outboard_body_num(),
            graph->FindLinkBodyNum("link1"));
}

// When there are multiple disjoint subtrees, they should each get a base
// body attached to World by a free joint. Base bodies are important since
// they can be moved freely -- check that we can report the structure
// properly.
//
//     link1->link2->link3    link4->link5          link7->link8   link9
//                                 ->link6->world
// Here link1, link6, link7, and link9 should be the chosen base bodies (link6
// because it is already attached to world).
GTEST_TEST(MultibodyGraphModeler, MultipleSubtrees) {
  MultibodyGraphModeler maker;
  for (int i = 1; i <= 9; ++i) {
    maker.AddLink("link" + std::to_string(i));
  }
  maker.AddJoint("joint0", "prismatic", "link1", "link2");
  maker.AddJoint("joint1", "fixed", "link2", "link3");
  maker.AddJoint("joint2", "revolute", "link4", "link5");
  maker.AddJoint("joint3", "prismatic", "link4", "link6");
  maker.AddJoint("joint4", "ball", "link6", "world");
  maker.AddJoint("joint5", "prismatic", "link7", "link8");

  maker.DumpInput(std::cout);
  auto graph = maker.MakeTreeModel();
  graph->DumpTreeModel(std::cout);

  // link6 comes first since it was already attached to World.
  EXPECT_EQ(
      graph->FindBaseBodies(),
      std::vector<BodyNum>({BodyNum(6), BodyNum(1), BodyNum(7), BodyNum(9)}));

  // World has no base body.
  EXPECT_FALSE(graph->FindBaseBody(BodyNum(0)).is_valid());

  EXPECT_EQ(graph->FindBaseBody(BodyNum(3)), 1);  // See above.
  EXPECT_EQ(graph->FindBaseBody(BodyNum(4)), 6);
  EXPECT_EQ(graph->FindBaseBody(BodyNum(5)), 6);
  EXPECT_EQ(graph->FindBaseBody(BodyNum(7)), 7);  // Already a base body.
  EXPECT_EQ(graph->FindBaseBody(BodyNum(8)), 7);
  EXPECT_EQ(graph->FindBaseBody(BodyNum(9)), 9);

  EXPECT_EQ(graph->FindPathToWorld(BodyNum(3)),
            std::vector<BodyNum>({BodyNum(3), BodyNum(2), BodyNum(1)}));
  EXPECT_EQ(graph->FindPathToWorld(BodyNum(5)),
            std::vector<BodyNum>({BodyNum(5), BodyNum(4), BodyNum(6)}));
  EXPECT_EQ(graph->FindPathToWorld(BodyNum(9)),
            std::vector<BodyNum>({BodyNum(9)}));
  EXPECT_EQ(graph->FindPathToWorld(BodyNum(0)), std::vector<BodyNum>());
}

// Test that DeleteLink() and DeleteJoint() properly untangle affected
// connections.
//
// Starting with link1->link2->link3,
//   - delete link1 should give link2->link3 (new base body, joint deleted)
//   - or, delete link2 should give link1 link3  (two independent bodies)
// Body and joint numbers should change also.
//
// Also tests Clear() and ClearGraph().
GTEST_TEST(MultibodyGraphModeler, DeleteBodiesAndClear) {
  MultibodyGraphModeler maker;
  for (int i = 1; i <= 3; ++i) maker.AddLink("link" + std::to_string(i));
  maker.AddJoint("joint0", "prismatic", "link1", "link2");
  maker.AddJoint("joint1", "revolute", "link2", "link3");

  EXPECT_EQ(maker.num_links(), 4);
  EXPECT_EQ(maker.num_joints(), 2);
  EXPECT_EQ(maker.FindLinkNum("world"), 0);
  EXPECT_EQ(maker.FindLinkNum("link1"), 1);
  EXPECT_EQ(maker.FindLinkNum("link2"), 2);
  EXPECT_EQ(maker.FindLinkNum("link3"), 3);
  EXPECT_EQ(maker.FindJointNum("joint0"), 0);
  EXPECT_EQ(maker.FindJointNum("joint1"), 1);

  auto graph = maker.MakeTreeModel();
  EXPECT_EQ(graph->FindBaseBodies(),
            std::vector<BodyNum>({BodyNum(1)}));  // link1

  EXPECT_EQ(graph->num_mobilizers(), 3);

  maker.DeleteLink("link1");
  EXPECT_EQ(maker.num_links(), 3);
  EXPECT_EQ(maker.num_joints(), 1);
  // Deleting link1 should have taken out joint0 also.
  EXPECT_FALSE(maker.FindLinkNum("link1").is_valid());
  EXPECT_FALSE(maker.FindJointNum("joint0").is_valid());

  EXPECT_EQ(maker.FindLinkNum("world"), 0);
  EXPECT_EQ(maker.FindLinkNum("link2"), 1);
  EXPECT_EQ(maker.FindLinkNum("link3"), 2);
  EXPECT_EQ(maker.FindJointNum("joint1"), 0);

  auto graph2 = maker.MakeTreeModel();
  EXPECT_EQ(graph2->FindBaseBodies(),
            std::vector<BodyNum>({BodyNum(1)}));  // link2

  maker.Clear();
  EXPECT_EQ(maker.num_links(), 1);  // Just World.
  EXPECT_EQ(maker.num_joints(), 0);

  // Recreate the link1->link2->link3 graph->
  for (int i = 1; i <= 3; ++i) maker.AddLink("link" + std::to_string(i));
  maker.AddJoint("joint0", "prismatic", "link1", "link2");
  maker.AddJoint("joint1", "revolute", "link2", "link3");

  maker.DeleteLink("link2");  // Should remove 2 joints also.
  EXPECT_EQ(maker.num_links(), 3);
  EXPECT_EQ(maker.num_joints(), 0);
  EXPECT_FALSE(maker.FindLinkNum("link2").is_valid());
  EXPECT_FALSE(maker.FindJointNum("joint0").is_valid());
  EXPECT_FALSE(maker.FindJointNum("joint1").is_valid());

  EXPECT_EQ(maker.FindLinkNum("world"), 0);
  EXPECT_EQ(maker.FindLinkNum("link1"), 1);
  EXPECT_EQ(maker.FindLinkNum("link3"), 2);

  auto graph3 = maker.MakeTreeModel();
  EXPECT_EQ(graph3->FindBaseBodies(),
            std::vector<BodyNum>({BodyNum(1), BodyNum(2)}));  // link1
}

// Use MBGraphMaker twice: first, find the base body; then, weld that body
// to world.
//            link1<--link2<--link3-->link4-->link5
// Here link3 should be the base body.
GTEST_TEST(MultibodyGraphModeler, ReplaceJoint) {
  MultibodyGraphModeler maker;
  for (int i = 1; i <= 5; ++i) maker.AddLink("link" + std::to_string(i));
  maker.AddJoint("joint0", "revolute", "link2", "link1");
  maker.AddJoint("joint1", "revolute", "link3", "link2");
  maker.AddJoint("joint2", "revolute", "link3", "link4");
  maker.AddJoint("joint3", "revolute", "link4", "link5");
  auto graph = maker.MakeTreeModel();

  EXPECT_EQ(graph->FindBaseBody(BodyNum(1)), 3);  // link1's base body is link3.
  EXPECT_EQ(graph->FindBaseBodies(), std::vector<BodyNum>({BodyNum(3)}));

  // Now replace link3's free joint with a weld.
  maker.AddJoint("joint5", "weld", "world", maker.get_link(LinkNum(3)).name());
  auto graph2 = maker.MakeTreeModel();
  graph2->DumpTreeModel(std::cout);

  // Expected mobilizers:
  // 0 world->link3 (weld)
  // 1 link3->link2 (revolute)
  // 2 link3->link4
  // 3 link2->link1
  // 4 link4->link5
  EXPECT_EQ(graph2->num_mobilizers(), 5);
  std::vector<int> expect_inboard({0, 3, 3, 2, 4});
  std::vector<int> expect_outboard({3, 2, 4, 1, 5});
  for (MobilizerNum i(0); i < graph2->num_mobilizers(); ++i) {
    const MultibodyTreeModel::Mobilizer& mobilizer = graph2->get_mobilizer(i);
    EXPECT_EQ(mobilizer.inboard_body_num(), expect_inboard[i]);
    EXPECT_EQ(mobilizer.outboard_body_num(), expect_outboard[i]);
    EXPECT_EQ(mobilizer.get_joint_type_name(), (i == 0 ? "weld" : "revolute"));
    EXPECT_FALSE(mobilizer.is_added_base_mobilizer());
    EXPECT_FALSE(mobilizer.is_reversed_from_joint());
    EXPECT_FALSE(mobilizer.is_slave_mobilizer());
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
