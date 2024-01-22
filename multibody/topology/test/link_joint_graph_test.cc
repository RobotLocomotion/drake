/* clang-format off to disable clang-format-includes */
#include "drake/multibody/topology/graph.h"
/* clang-format on */

#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

// Verify that the enums in link_joint_graph_defs.h work properly as bitmaps.
GTEST_TEST(LinkJointGraph, FlagsAndOptions) {
  const auto is_static = internal::LinkFlags::Static;
  const auto massless = internal::LinkFlags::TreatAsMassless;

  // Or-ed flags still have LinkFlags type.
  auto link_flags = is_static | massless;
  static_assert(std::is_same_v<decltype(link_flags), internal::LinkFlags>);

  // And is bitwise but still returns LinkFlags, convertible to bool.
  EXPECT_EQ(link_flags & is_static, internal::LinkFlags::Static);
  EXPECT_EQ(link_flags & massless, internal::LinkFlags::TreatAsMassless);
  EXPECT_FALSE(
      static_cast<bool>(link_flags & internal::LinkFlags::MustBeBaseBody));

  // Repeat for Modeling Options.
  const auto use_fixed_base = internal::ModelingOptions::UseFixedBase;
  const auto combine_links = internal::ModelingOptions::CombineLinkComposites;
  auto modeling_options = use_fixed_base | combine_links;
  static_assert(
      std::is_same_v<decltype(modeling_options), internal::ModelingOptions>);
  EXPECT_EQ(modeling_options & use_fixed_base,
            internal::ModelingOptions::UseFixedBase);
  EXPECT_EQ(modeling_options & combine_links,
            internal::ModelingOptions::CombineLinkComposites);
  EXPECT_FALSE(static_cast<bool>(
      modeling_options & internal::ModelingOptions::UseRpyFloatingJoints));

  // Only one option in JointFlags currently.
  auto joint_flags = internal::JointFlags::MustBeModeled;
  static_assert(std::is_same_v<decltype(joint_flags), internal::JointFlags>);
  EXPECT_EQ(joint_flags & internal::JointFlags::MustBeModeled,
            internal::JointFlags::MustBeModeled);
  EXPECT_EQ(joint_flags | internal::JointFlags::MustBeModeled,
            internal::JointFlags::MustBeModeled);
}

// LinkJointGraph supports copy/move/assign, including the already-built
// SpanningForest. That takes some sleight-of-hand to get back pointers right;
// make sure it works.
GTEST_TEST(LinkJointGraph, CopyMoveAssignTest) {
  LinkJointGraph graph;
  LinkJointGraph* graph_ptr = &graph;  // To avoid warnings below.
  const SpanningForest* original_graph_forest = &graph.BuildForest();
  EXPECT_EQ(&graph.forest().graph(), &graph);
  EXPECT_TRUE(graph.forest_is_valid());
  graph.AddLink("link1", ModelInstanceIndex(19));
  EXPECT_FALSE(graph.forest_is_valid());

  // Self-assign and self-move-assign should be no-ops.
  graph = *graph_ptr;
  graph = std::move(*graph_ptr);
  EXPECT_EQ(ssize(graph.links()), 2);  // World + link1
  EXPECT_EQ(&graph.forest(), original_graph_forest);

  LinkJointGraph graph_copy(graph);  // Copy constructor.
  const SpanningForest* graph_copy_forest = &graph_copy.forest();
  EXPECT_TRUE(graph_copy.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_FALSE(graph_copy.forest_is_valid());
  EXPECT_NE(graph_copy_forest, original_graph_forest);
  EXPECT_EQ(&graph_copy.forest().graph(), &graph_copy);

  LinkJointGraph graph_assign;
  graph_assign = graph;  // Copy assignment.
  EXPECT_TRUE(graph_assign.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_FALSE(graph_assign.forest_is_valid());
  EXPECT_NE(&graph_assign.forest(), &graph.forest());
  EXPECT_EQ(&graph_assign.forest().graph(), &graph_assign);

  graph.BuildForest();  // Mark forest valid again.
  EXPECT_TRUE(graph.forest_is_valid());
  EXPECT_EQ(ssize(graph.links()), 2);  // World + link1

  LinkJointGraph graph_move(std::move(graph));             // Move constructor.
  EXPECT_EQ(&graph_move.forest(), original_graph_forest);  // Stolen!
  EXPECT_TRUE(graph_move.forest_is_valid());
  EXPECT_TRUE(graph_move.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_EQ(ssize(graph_move.links()), 2);  // World + link1
  EXPECT_EQ(ssize(graph.links()), 1);       // Back to default state.
  EXPECT_NE(&graph.forest(), nullptr);      // Should have a new Forest.
  EXPECT_NE(&graph.forest(), original_graph_forest);

  LinkJointGraph graph_move_assign;
  graph_move_assign = std::move(graph_copy);                  // Move assignent.
  EXPECT_EQ(&graph_move_assign.forest(), graph_copy_forest);  // Stolen!
  EXPECT_FALSE(graph_move_assign.forest_is_valid());
  EXPECT_TRUE(graph_move_assign.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_EQ(ssize(graph_move_assign.links()), 2);  // World + link1
  EXPECT_EQ(ssize(graph_copy.links()), 1);         // Back to default state.
  EXPECT_NE(&graph_copy.forest(), nullptr);        // Should have a new Forest.
  EXPECT_NE(&graph_copy.forest(), graph_copy_forest);
}

// A default-constructed LinkJointGraph should contain a predefined World
// Link, predefined joint types, and be suitable for generating a matching
// SpanningForest.
GTEST_TEST(LinkJointGraph, EmptyTest) {
  LinkJointGraph graph;

  // World is predefined.
  EXPECT_EQ(ssize(graph.links()), 1);
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(graph.world_link().name(), "world");
  EXPECT_EQ(graph.world_link().model_instance(), world_model_instance());
  EXPECT_EQ(graph.world_link().index(), BodyIndex(0));

  // Topologically important joint types are predefined.
  EXPECT_EQ(ssize(graph.joint_types()), 3);
  const LinkJointGraph::JointType& weld_type =
      graph.joint_types(LinkJointGraph::weld_joint_type_index());
  EXPECT_EQ(weld_type.type_name, "weld");
  EXPECT_EQ(weld_type.nq, 0);
  EXPECT_EQ(weld_type.nv, 0);
  EXPECT_EQ(weld_type.has_quaternion, false);
  const LinkJointGraph::JointType& quaternion_floating_type =
      graph.joint_types(LinkJointGraph::quaternion_floating_joint_type_index());
  EXPECT_EQ(quaternion_floating_type.type_name, "quaternion_floating");
  EXPECT_EQ(quaternion_floating_type.nq, 7);
  EXPECT_EQ(quaternion_floating_type.nv, 6);
  EXPECT_EQ(quaternion_floating_type.has_quaternion, true);
  const LinkJointGraph::JointType& rpy_floating_type =
      graph.joint_types(LinkJointGraph::rpy_floating_joint_type_index());
  EXPECT_EQ(rpy_floating_type.type_name, "rpy_floating");
  EXPECT_EQ(rpy_floating_type.nq, 6);
  EXPECT_EQ(rpy_floating_type.nv, 6);
  EXPECT_EQ(rpy_floating_type.has_quaternion, false);

  EXPECT_FALSE(graph.forest_is_valid());

  const SpanningForest& forest = graph.BuildForest();
  EXPECT_EQ(&forest.graph(), &graph);
  EXPECT_TRUE(graph.forest_is_valid());

  // Check that Clear() puts the graph back to default-constructed condition.
  // First add some junk to the graph.
  graph.RegisterJointType("revolute", 1, 1);
  const BodyIndex dummy_index =
      graph.AddLink("dummy", default_model_instance());
  EXPECT_TRUE(graph.HasLinkNamed("dummy", default_model_instance()));
  EXPECT_EQ(dummy_index, BodyIndex(1));
  EXPECT_EQ(ssize(graph.links()), 2);
  EXPECT_EQ(ssize(graph.joints()), 0);
  EXPECT_EQ(ssize(graph.joint_types()), 4);
  // Now get rid of that junk.
  graph.Clear();
  EXPECT_EQ(ssize(graph.links()), 1);  // World
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(ssize(graph.joint_types()), 3);  // Predefineds
  EXPECT_FALSE(graph.forest_is_valid());

  // Make sure Clear() saved the existing forest.
  const SpanningForest& same_forest = graph.BuildForest();
  EXPECT_EQ(&same_forest, &forest);
  EXPECT_EQ(&forest.graph(), &graph);
  EXPECT_TRUE(graph.forest_is_valid());
}

// Make sure AddLink() rejects obvious errors.
GTEST_TEST(LinkJointGraph, AddLinkErrors) {
  LinkJointGraph graph;

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("not_world", world_model_instance()),
      "AddLink.*Model instance.*0.*reserved for.*World.*");

  graph.AddLink("link1", ModelInstanceIndex(3));
  graph.AddLink("link1", ModelInstanceIndex(4));  // OK, different instance.

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("link1", ModelInstanceIndex(3)),
      "AddLink.*already a link named.*link1.*model instance.*3.*");

  // Addlink accepts flags, but not Shadow which is set internally only.
  const BodyIndex link2_index =
      graph.AddLink("link2", ModelInstanceIndex(3),
                    LinkFlags::TreatAsMassless | LinkFlags::MustBeBaseBody);
  const LinkJointGraph::Link& link2 = graph.links(link2_index);
  EXPECT_TRUE(link2.treat_as_massless() && link2.must_be_base_body());
  EXPECT_FALSE(link2.is_static() || link2.is_shadow());

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("link3", ModelInstanceIndex(3), LinkFlags::Shadow),
      "AddLink.*Can't add.*link3.*Shadow flag.*");
}

// Check operation of the public members of the Link subclass.
GTEST_TEST(LinkJoinGraph, LinkTest) {
  LinkJointGraph::Link link5(BodyIndex(5), "link5", ModelInstanceIndex(7),
                             LinkFlags::MustBeBaseBody);
  EXPECT_EQ(link5.index(), BodyIndex(5));
  EXPECT_EQ(link5.model_instance(), ModelInstanceIndex(7));
  EXPECT_EQ(link5.name(), "link5");

  // Check flags.
  EXPECT_FALSE(link5.is_world());
  EXPECT_FALSE(link5.is_anchored());
  EXPECT_FALSE(link5.is_static());
  EXPECT_FALSE(link5.treat_as_massless());
  EXPECT_FALSE(link5.is_shadow());
  EXPECT_TRUE(link5.must_be_base_body());
  link5.set_flags(internal::LinkFlags::Static);
  EXPECT_TRUE(link5.is_static());
  EXPECT_TRUE(link5.is_anchored());  // Static links are anchored to World.
  EXPECT_TRUE(link5.must_be_base_body());  // Unchanged by set_flags().

  // Only LinkJointGraph set these; no public interface.
  EXPECT_TRUE(link5.joints().empty());
  EXPECT_TRUE(link5.joints_as_parent().empty());
  EXPECT_TRUE(link5.joints_as_child().empty());
  EXPECT_EQ(link5.num_shadows(), 0);
  EXPECT_FALSE(link5.primary_link().is_valid());
  EXPECT_FALSE(link5.mobod_index().is_valid());
  EXPECT_FALSE(link5.inboard_joint_index().is_valid());
  EXPECT_FALSE(link5.composite().is_valid());
}

// Check operation of the public members of the Joint subclass.
GTEST_TEST(LinkJointGraph, JointTest) {
  LinkJointGraph::Joint joint3(JointIndex(3), "joint3", ModelInstanceIndex(9),
                               LinkJointGraph::rpy_floating_joint_type_index(),
                               /*parent=*/BodyIndex(1), /*child=*/BodyIndex(2),
                               JointFlags::MustBeModeled);
  EXPECT_EQ(joint3.index(), JointIndex(3));
  EXPECT_EQ(joint3.model_instance(), ModelInstanceIndex(9));
  EXPECT_EQ(joint3.name(), "joint3");
  EXPECT_EQ(joint3.parent_link(), BodyIndex(1));
  EXPECT_EQ(joint3.child_link(), BodyIndex(2));
  EXPECT_FALSE(joint3.is_weld());
  EXPECT_EQ(joint3.type_index(),
            LinkJointGraph::rpy_floating_joint_type_index());
  EXPECT_TRUE(joint3.connects(BodyIndex(1)));
  EXPECT_TRUE(joint3.connects(BodyIndex(2)));
  EXPECT_FALSE(joint3.connects(BodyIndex(3)));
  EXPECT_TRUE(joint3.connects(BodyIndex(1), BodyIndex(2)));
  EXPECT_TRUE(joint3.connects(BodyIndex(2), BodyIndex(1)));
  EXPECT_FALSE(joint3.connects(BodyIndex(1), BodyIndex(3)));
  EXPECT_TRUE(joint3.must_be_modeled());
  EXPECT_FALSE(joint3.mobod_index().is_valid());
  EXPECT_FALSE(joint3.has_been_processed());
  EXPECT_EQ(joint3.other_link_index(BodyIndex(1)), BodyIndex(2));
  EXPECT_EQ(joint3.other_link_index(BodyIndex(2)), BodyIndex(1));
  joint3.set_flags(JointFlags::Default);
  EXPECT_FALSE(joint3.must_be_modeled());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
