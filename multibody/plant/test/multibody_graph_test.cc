#include "drake/multibody/plant/multibody_graph.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace internal {

const std::string kRevoluteType = "revolute";
const std::string kPrismaticType = "prismatic";

// Arbitrary world name for testing.
const std::string kWorldLinkName = "DefaultWorldLinkName";

// Arbitrary non-zero world model instance for testing.
const ModelInstanceIndex kWorldModelInstance(999);

// Test a straightforward serial chain of massful links connected by
// pin joints: world->link1->link2->link3->link4->link5
// Should produce the obvious multibody tree.
GTEST_TEST(MultibodyGraph, SerialChain) {
  MultibodyGraph graph;
  EXPECT_EQ(graph.num_joint_types(), 1);  // "weld" joint thus far.
  graph.RegisterJointType(kRevoluteType);
  EXPECT_EQ(graph.num_joint_types(), 2);  // "weld" and "revolute".

  // The first link added defines the world's name and model instance.
  // We'll verify their values below.
  graph.AddLink(kWorldLinkName, kWorldModelInstance);

  // We'll add the chain to this model.
  const ModelInstanceIndex model_instance(5);

  // We cannot register to the world model intance.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("InvalidLink", kWorldModelInstance), std::runtime_error,
      fmt::format("AddLink\\(\\): Model instance index = {}.*",
                  kWorldModelInstance));

  LinkIndex parent = graph.AddLink("link1", model_instance);
  graph.AddJoint("pin1", model_instance, kRevoluteType,
                 graph.world_link_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    LinkIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, kRevoluteType,
                   parent, child);
    parent = child;
  }

  // We cannot duplicate the name of a link or joint.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddLink("link3", model_instance),
                              std::runtime_error,
                              "AddLink\\(\\): Duplicate link name.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("pin3", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(2)),
      std::runtime_error, "AddJoint\\(\\): Duplicate joint name.*");

  // We cannot add an unregistered joint type.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddJoint("screw1", model_instance, "screw",
                                             LinkIndex(1), LinkIndex(2)),
                              std::runtime_error,
                              "AddJoint\\(\\): Unrecognized type.*");

  // Invalid parent/child link throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(9)),
      std::runtime_error,
      "AddJoint\\(\\): child link index for joint '.*' is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, LinkIndex(9),
                     LinkIndex(1)),
      std::runtime_error,
      "AddJoint\\(\\): parent link index for joint '.*' is invalid.");

  // Verify the world's name and model instance are registered correctly.
  EXPECT_EQ(graph.world_link_name(), kWorldLinkName);
  EXPECT_EQ(graph.world_model_instance(), kWorldModelInstance);
  // Link "0" is always the world.
  EXPECT_EQ(graph.get_link(LinkIndex(0)).name(), kWorldLinkName);
  EXPECT_EQ(graph.get_link(LinkIndex(0)).model_instance(), kWorldModelInstance);

  // Sanity check sizes.
  EXPECT_EQ(graph.num_links(), 6);  // this includes the world link.
  EXPECT_EQ(graph.num_joints(), 5);
}

// We build a model.
// Island A (forms closed loop):
//  - WeldJoint(1, 13)
//  - WeldJoint(1, 4)
//  - WeldJoint(4, 13)
//
// Island B (forms closed loop):
//  - WeldJoint(6, 10)
//  - WeldJoint(6, 8)
//  - WeldJoint(8, 10)
//
// Island C:
//  - WeldJoint(5, 7)
//  - WeldJoint(5, 12)
//
// Non-weld joints kinematic loop:
//  - PrimaticJoint(2, 11)
//  - PrimaticJoint(2, 7)
//  - RevoluteJoint(7, 11)
//
// Additionally we have the following non-weld joints:
//  - RevoluteJoint(3, 13): connects link 3 to island A.
//  - PrimaticJoint(1, 10): connects island A and B.
// 
// Therefore we expect the following islands, in no particular oder, but with
// the "world" island first:
//   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
GTEST_TEST(MultibodyGraph, WeldedIslands) {
  MultibodyGraph graph;
  graph.RegisterJointType(kRevoluteType);
  graph.RegisterJointType(kPrismaticType);
  EXPECT_EQ(graph.num_joint_types(), 3);  // weld, revolute and prismatic.

  // The first link added defines the world's name and model instance.
  graph.AddLink(kWorldLinkName, kWorldModelInstance);

  // We'll add links and joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the model.
  for (int i = 1; i <= 13; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  // Add joints.
  int j = 0;

  // Island A: formed by links 1, 4, 13.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(1), LinkIndex(13));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(4), LinkIndex(1));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(13), LinkIndex(4));

  // Link 3 connects to island A via a revolute joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 LinkIndex(3), LinkIndex(13));

  // Island B: formed by links 8, 6, 10.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(10), LinkIndex(6));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(10), LinkIndex(8));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(6), LinkIndex(8));

  // Link 1 (in island A) and link 10 (in island B) connect through a prismatic
  // joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(1), LinkIndex(10));

  // Closed kinematic loop of non-weld joints.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(2), LinkIndex(11));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(7), LinkIndex(2));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 LinkIndex(7), LinkIndex(11));

  // Island C: formed by links 5, 7, 12.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(5), LinkIndex(7));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(5),
                 graph.world_link_index());
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(12), LinkIndex(5));

  EXPECT_EQ(graph.num_links(), 14);  // this includes the world link.

  const std::vector<std::set<LinkIndex>> welded_islands =
      graph.FindIslandsOfWeldedLinks();

  // Verify number of expected islands.  
  EXPECT_EQ(welded_islands.size(), 7);

  // The first island must contain the world.
  const std::set<LinkIndex> world_island = welded_islands[0];
  EXPECT_EQ(world_island.count(graph.world_link_index()), 1);

  // Build the expected set of islands.  
  std::set<std::set<LinkIndex>> expected_islands;
  //   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
  const std::set<LinkIndex>& expected_world_island =
      *expected_islands
           .insert({LinkIndex(0), LinkIndex(5), LinkIndex(7), LinkIndex(12)})
           .first;
  expected_islands.insert({LinkIndex(1), LinkIndex(4), LinkIndex(13)});
  expected_islands.insert({LinkIndex(6), LinkIndex(8), LinkIndex(10)});
  expected_islands.insert({LinkIndex(3)});
  expected_islands.insert({LinkIndex(9)});
  expected_islands.insert({LinkIndex(2)});
  expected_islands.insert({LinkIndex(11)});

  // We do expect the first island to correspond to the set of links welded to
  // the world.
  EXPECT_EQ(world_island, expected_world_island);

  // In order to compare the computed list of welded links against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed islands to a set.
  const std::set<std::set<LinkIndex>> welded_islands_set(welded_islands.begin(),
                                                         welded_islands.end());
  EXPECT_EQ(welded_islands_set, expected_islands);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake