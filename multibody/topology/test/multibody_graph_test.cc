#include "drake/multibody/topology/multibody_graph.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace internal {

const char kRevoluteType[] = "revolute";
const char kPrismaticType[] = "prismatic";

// Arbitrary world name for testing.
const char kWorldBodyName[] = "DefaultWorldBodyName";

// Test a straightforward serial chain of bodies connected by
// revolute joints: world->body1->body2->body3->body4->body5.
// We perform a number of sanity checks on the provided API.
GTEST_TEST(MultibodyGraph, SerialChain) {
  MultibodyGraph graph;
  EXPECT_EQ(graph.num_joint_types(), 1);  // "weld" joint thus far.
  graph.RegisterJointType(kRevoluteType);
  EXPECT_EQ(graph.num_joint_types(), 2);  // "weld" and "revolute".

  // Verify what joint types were registered.
  EXPECT_TRUE(graph.IsJointTypeRegistered(kRevoluteType));
  EXPECT_FALSE(graph.IsJointTypeRegistered(kPrismaticType));

  // The first body added defines the world's name and model instance.
  // We'll verify their values below.
  graph.AddBody(kWorldBodyName, world_model_instance());

  // We'll add the chain to this model.
  const ModelInstanceIndex model_instance(5);

  // We cannot register to the world model instance, unless it's the first call
  // to AddBody().
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddBody("InvalidBody", world_model_instance()), std::runtime_error,
      fmt::format("AddBody\\(\\): Model instance index = {}.*",
                  world_model_instance()));

  BodyIndex parent = graph.AddBody("body1", model_instance);
  graph.AddJoint("pin1", model_instance, kRevoluteType, world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    BodyIndex child = graph.AddBody("body" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, kRevoluteType,
                   parent, child);
    parent = child;
  }

  // We cannot duplicate the name of a body or joint.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddBody("body3", model_instance),
                              std::runtime_error,
                              "AddBody\\(\\): Duplicate body name.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("pin3", model_instance, kRevoluteType, BodyIndex(1),
                     BodyIndex(2)),
      std::runtime_error, "AddJoint\\(\\): Duplicate joint name.*");

  // We cannot add a redundant joint.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("other", model_instance, kRevoluteType, BodyIndex(1),
                     BodyIndex(2)),
      std::runtime_error,
      "This MultibodyGraph already has a joint 'pin2' connecting 'body1'"
      " to 'body2'. Therefore adding joint 'other' connecting 'body1' to"
      " 'body2' is not allowed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("reverse", model_instance, kRevoluteType, BodyIndex(2),
                     BodyIndex(1)),
      std::runtime_error,
      "This MultibodyGraph already has a joint 'pin2' connecting 'body1'"
      " to 'body2'. Therefore adding joint 'reverse' connecting 'body2' to"
      " 'body1' is not allowed.");

  // We cannot add an unregistered joint type.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddJoint("screw1", model_instance, "screw",
                                             BodyIndex(1), BodyIndex(2)),
                              std::runtime_error,
                              "AddJoint\\(\\): Unrecognized type.*");

  // Invalid parent/child body throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, BodyIndex(1),
                     BodyIndex(9)),
      std::runtime_error,
      "AddJoint\\(\\): child body index for joint '.*' is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, BodyIndex(9),
                     BodyIndex(1)),
      std::runtime_error,
      "AddJoint\\(\\): parent body index for joint '.*' is invalid.");

  // Verify the world's name and model instance are registered correctly.
  EXPECT_EQ(graph.world_body_name(), kWorldBodyName);
  EXPECT_EQ(graph.world_body().model_instance(), world_model_instance());
  // Body "0" is always the world.
  EXPECT_EQ(graph.get_body(BodyIndex(0)).name(), kWorldBodyName);
  EXPECT_EQ(graph.get_body(BodyIndex(0)).model_instance(),
            world_model_instance());

  // Sanity check sizes.
  EXPECT_EQ(graph.num_bodies(), 6);  // this includes the world body.
  EXPECT_EQ(graph.num_joints(), 5);

  // Verify we can get bodies/joints.
  EXPECT_EQ(graph.get_body(BodyIndex(3)).name(), "body3");
  EXPECT_EQ(graph.get_joint(JointIndex(3)).name(), "pin4");
  DRAKE_EXPECT_THROWS_MESSAGE(graph.get_body(BodyIndex(9)),
                              ".*index < num_bodies\\(\\).*");
  DRAKE_EXPECT_THROWS_MESSAGE(graph.get_joint(JointIndex(9)),
                              ".*index < num_joints\\(\\).*");

  // Verify we can query if a body/joint is in the graph.
  const ModelInstanceIndex kInvalidModelInstance(666);
  EXPECT_TRUE(graph.HasBodyNamed("body3", model_instance));
  EXPECT_FALSE(graph.HasBodyNamed("body3", kInvalidModelInstance));
  EXPECT_FALSE(graph.HasBodyNamed("invalid_body_name", model_instance));
  EXPECT_TRUE(graph.HasJointNamed("pin3", model_instance));
  EXPECT_FALSE(graph.HasJointNamed("pin3", kInvalidModelInstance));
  EXPECT_FALSE(graph.HasJointNamed("invalid_joint_name", model_instance));
}

// We build a model containing a number of kinematic loops and subgraphs of
// welded bodies.
//
// subgraph A (forms closed loop):
//  - WeldJoint(1, 13)
//  - WeldJoint(1, 4)
//  - WeldJoint(4, 13)
//
// subgraph B (forms closed loop):
//  - WeldJoint(6, 10)
//  - WeldJoint(6, 8)
//  - WeldJoint(8, 10)
//
// subgraph C (the "world" subgraph):
//  - WeldJoint(5, 7)
//  - WeldJoint(5, 12)
//
// Non-weld joints kinematic loop:
//  - PrimaticJoint(2, 11)
//  - PrimaticJoint(2, 7)
//  - RevoluteJoint(7, 11)
//
// Additionally we have the following non-weld joints:
//  - RevoluteJoint(3, 13): connects body 3 to subgraph A.
//  - PrimaticJoint(1, 10): connects subgraph A and B.
//
// Therefore we expect the following subgraphs, in no particular oder, but with
// the "world" subgraph first:
//   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
GTEST_TEST(MultibodyGraph, Weldedsubgraphs) {
  MultibodyGraph graph;
  graph.RegisterJointType(kRevoluteType);
  graph.RegisterJointType(kPrismaticType);
  EXPECT_EQ(graph.num_joint_types(), 3);  // weld, revolute and prismatic.

  // The first body added defines the world's name and model instance.
  graph.AddBody(kWorldBodyName, world_model_instance());

  // We'll add bodies and joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the model.
  for (int i = 1; i <= 13; ++i) {
    graph.AddBody("body" + std::to_string(i), model_instance);
  }

  // Add joints.
  int j = 0;

  // subgraph A: formed by bodies 1, 4, 13.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(1), BodyIndex(13));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(4), BodyIndex(1));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(13), BodyIndex(4));

  // Body 3 connects to subgraph A via a revolute joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 BodyIndex(3), BodyIndex(13));

  // subgraph B: formed by bodies 8, 6, 10.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(10), BodyIndex(6));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(10), BodyIndex(8));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(6), BodyIndex(8));

  // Body 1 (in subgraph A) and body 10 (in subgraph B) connect through a
  // prismatic joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 BodyIndex(1), BodyIndex(10));

  // Closed kinematic loop of non-weld joints.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 BodyIndex(2), BodyIndex(11));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 BodyIndex(7), BodyIndex(2));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 BodyIndex(7), BodyIndex(11));

  // subgraph C: formed by bodies 5, 7, 12.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(5), BodyIndex(7));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(5), world_index());
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(12), BodyIndex(5));

  EXPECT_EQ(graph.num_bodies(), 14);  // this includes the world body.

  const std::vector<std::set<BodyIndex>> welded_subgraphs =
      graph.FindSubgraphsOfWeldedBodies();

  // Verify number of expected subgraphs.
  EXPECT_EQ(welded_subgraphs.size(), 7);

  // The first subgraph must contain the world.
  const std::set<BodyIndex> world_subgraph = welded_subgraphs[0];
  EXPECT_EQ(world_subgraph.count(world_index()), 1);

  // Build the expected set of subgraphs.
  std::set<std::set<BodyIndex>> expected_subgraphs;
  //   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
  const std::set<BodyIndex>& expected_world_subgraph =
      *expected_subgraphs
           .insert({BodyIndex(0), BodyIndex(5), BodyIndex(7), BodyIndex(12)})
           .first;
  const std::set<BodyIndex>& expected_subgraphA =
      *expected_subgraphs.insert({BodyIndex(1), BodyIndex(4), BodyIndex(13)})
           .first;
  const std::set<BodyIndex>& expected_subgraphB =
      *expected_subgraphs.insert({BodyIndex(6), BodyIndex(8), BodyIndex(10)})
           .first;
  expected_subgraphs.insert({BodyIndex(3)});
  expected_subgraphs.insert({BodyIndex(9)});
  expected_subgraphs.insert({BodyIndex(2)});
  expected_subgraphs.insert({BodyIndex(11)});

  // We do expect the first subgraph to correspond to the set of bodies welded
  // to the world.
  EXPECT_EQ(world_subgraph, expected_world_subgraph);

  // In order to compare the computed list of welded bodies against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed subgraphs to a set.
  const std::set<std::set<BodyIndex>> welded_subgraphs_set(
      welded_subgraphs.begin(), welded_subgraphs.end());
  EXPECT_EQ(welded_subgraphs_set, expected_subgraphs);

  // Verify we can query the list of bodies welded to a particular body.
  EXPECT_EQ(graph.FindBodiesWeldedTo(BodyIndex(9)).size(), 1);
  EXPECT_EQ(graph.FindBodiesWeldedTo(BodyIndex(11)).size(), 1);
  EXPECT_EQ(graph.FindBodiesWeldedTo(BodyIndex(4)), expected_subgraphA);
  EXPECT_EQ(graph.FindBodiesWeldedTo(BodyIndex(13)), expected_subgraphA);
  EXPECT_EQ(graph.FindBodiesWeldedTo(BodyIndex(10)), expected_subgraphB);
  EXPECT_EQ(graph.FindBodiesWeldedTo(BodyIndex(6)), expected_subgraphB);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
