#include "drake/multibody/plant/multibody_graph.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace internal {

const std::string kRevoluteType = "revolute";

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

}  // namespace internal
}  // namespace multibody
}  // namespace drake