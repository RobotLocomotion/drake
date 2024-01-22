/* clang-format off to disable clang-format-includes */
#include "drake/multibody/topology/graph.h"
/* clang-format on */

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

using std::pair;

const char kRevoluteType[] = "revolute";
const char kPrismaticType[] = "prismatic";

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

  EXPECT_FALSE(graph.forest_is_valid());

  const SpanningForest& forest = graph.BuildForest();
  forest.SanityCheckForest();
  EXPECT_TRUE(graph.forest_is_valid());
  EXPECT_EQ(ssize(forest.mobods()), 1);
  EXPECT_TRUE(forest.mobods(MobodIndex(0)).is_world());

  // Check that Clear() puts the graph back to default-constructed condition.
  // First add some junk to the graph.
  graph.RegisterJointType(kRevoluteType, 1, 1);
  const BodyIndex dummy_index =
      graph.AddLink("dummy", default_model_instance());
  graph.AddJoint("joint0", default_model_instance(), kRevoluteType,
                 world_index(), dummy_index);
  EXPECT_EQ(ssize(graph.links()), 2);
  EXPECT_EQ(ssize(graph.joints()), 1);
  EXPECT_EQ(ssize(graph.joint_types()), 4);
  // Now get rid of that junk.
  graph.Clear();
  EXPECT_EQ(ssize(graph.links()), 1);
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(ssize(graph.joint_types()), 3);
  EXPECT_FALSE(graph.forest_is_valid());
  // Not required that the SpanningForest instance is the same one as before
  // the Clear().
  const SpanningForest& new_forest = graph.BuildForest();
  new_forest.SanityCheckForest();
  EXPECT_TRUE(graph.forest_is_valid());
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

/* Test a straightforward serial chain of bodies connected by
revolute joints: world->body1->body2->body3->body4->body5.
We perform a number of sanity checks on the provided API. */
GTEST_TEST(LinkJointGraph, SerialChain) {
  LinkJointGraph graph;

  graph.RegisterJointType(kRevoluteType, 1, 1);
  EXPECT_EQ(ssize(graph.joint_types()), 4);  // plus "revolute"

  // Verify what joint types were registered.
  EXPECT_TRUE(graph.IsJointTypeRegistered(kRevoluteType));
  EXPECT_FALSE(graph.IsJointTypeRegistered(kPrismaticType));

  // We'll add the chain to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Put static bodies in this model instance.
  const ModelInstanceIndex static_model_instance(100);

  // We cannot register any link to the world model instance.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("InvalidLink", world_model_instance()),
      fmt::format("AddLink\\(\\): Model instance index {} is reserved.*",
                  world_model_instance()));

  BodyIndex parent = graph.AddLink("link1", model_instance);
  graph.AddJoint("pin1", model_instance, kRevoluteType, world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    BodyIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, kRevoluteType,
                   parent, child);
    parent = child;
  }

  // We cannot duplicate the name of a Link or Joint.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddLink("link3", model_instance),
                              "AddLink\\(\\): Duplicate link name.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("pin3", model_instance, kRevoluteType, BodyIndex(1),
                     BodyIndex(2)),
      "AddJoint\\(\\): Duplicate joint name.*");

  // We cannot add a redundant joint.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("other", model_instance, kRevoluteType, BodyIndex(1),
                     BodyIndex(2)),
      "AddJoint\\(\\): This LinkJointGraph already has a joint 'pin2'"
      " connecting 'link1' to 'link2'. Therefore adding joint 'other'"
      " connecting 'link1' to 'link2' is not allowed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("reverse", model_instance, kRevoluteType, BodyIndex(2),
                     BodyIndex(1)),
      "AddJoint\\(\\): This LinkJointGraph already has a joint 'pin2'"
      " connecting 'link1' to 'link2'. Therefore adding joint 'reverse'"
      " connecting 'link2' to 'link1' is not allowed.");

  // We cannot add an unregistered joint type.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddJoint("screw1", model_instance, "screw",
                                             BodyIndex(1), BodyIndex(2)),
                              "AddJoint\\(\\): Unrecognized type.*");

  // Invalid parent/child Link throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, BodyIndex(1),
                     BodyIndex(9)),
      "AddJoint\\(\\): child link index 9 for joint '.*' is out of range.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, BodyIndex(9),
                     BodyIndex(1)),
      "AddJoint\\(\\): parent link index 9 for joint '.*' is out of range.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("joint_to_self", model_instance, kRevoluteType,
                     BodyIndex(5), BodyIndex(5)),
      "AddJoint\\(\\): Can't add a joint from link 'link5' to itself.");

  // Sanity check sizes.
  EXPECT_EQ(ssize(graph.links()), 6);  // This includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 5);

  // Verify we can get bodies/joints.
  EXPECT_EQ(graph.links(BodyIndex(3)).name(), "link3");
  EXPECT_EQ(graph.joints(JointIndex(3)).name(), "pin4");
  EXPECT_THROW(graph.links(BodyIndex(9)), std::exception);
  EXPECT_THROW(graph.joints(JointIndex(9)), std::exception);

  // Verify we can query if a Link/Joint is in the graph.
  const ModelInstanceIndex kInvalidModelInstance(666);
  EXPECT_TRUE(graph.HasLinkNamed("link3", model_instance));
  EXPECT_FALSE(graph.HasLinkNamed("link3", kInvalidModelInstance));
  EXPECT_FALSE(graph.HasLinkNamed("invalid_link_name", model_instance));
  EXPECT_TRUE(graph.HasJointNamed("pin3", model_instance));
  EXPECT_FALSE(graph.HasJointNamed("pin3", kInvalidModelInstance));
  EXPECT_FALSE(graph.HasJointNamed("invalid_joint_name", model_instance));

  // We can add a Static Link with no Joint, or attach it to World with an
  // explicit Weld, but we can't use any other kind of Joint to World.
  graph.AddLink("static6", static_model_instance);
  const BodyIndex static7_index =
      graph.AddLink("static7", static_model_instance);
  const BodyIndex static8_index =
      graph.AddLink("static8", model_instance, LinkFlags::Static);
  graph.AddJoint("static7_weld", model_instance,  // OK
                 "weld", graph.world_link().index(), static7_index);
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("static8_pin", model_instance, kRevoluteType,
                     graph.world_link().index(), static8_index),
      "AddJoint\\(\\): can't connect.*'static8' to World.*revolute.*"
      "only a weld.*");

  // Now add a free link and a free-floating pair.
  graph.AddLink("free9", model_instance);

  const BodyIndex link10_index = graph.AddLink("link10", model_instance);
  const BodyIndex base11_index =
      graph.AddLink("base11", model_instance, LinkFlags::MustBeBaseBody);
  const JointIndex joint_10_11_index =
      graph.AddJoint("weld", model_instance, graph.weld_type_name(),
                     link10_index, base11_index);

  /* The LinkJointGraph looks like this:

  world->link1->link2->link3->link4->link5
      static6         (no joint, static forest instance)
      ->static7       (weld provided, static forest instance)
      static8         (no joint, static link)
      free9           (no joint)
      link10->base11* (these are welded together)

    * link10 would be the preferred base link but link11 is marked
      "must be base link" so we have to use a reversed mobilizer there

  The SpanningForest should get built with 6 trees of Mobods in this order:
   (legend: -> given joint, => added joint, [mobods], {composite links})

   tree      world [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6]    ->static7
     2 [7]    =>static6        (added weld)
     3 [8]    =>static8        (added weld)
     4 [9-10] =>base11->link10 (added 6dof, reversed the weld)
     5 [11]   =>free9          (added 6dof, free bodies are always last)

   Link Composites:  {0 7 6 8} {11 10}
   Composite Mobods: [0 6 7 8] [9 10]

   The particular ordering results from (a) user-supplied Joints get processed
   before added ones, and (b) static forest instance Links get welded prior
   to individually-specified static Links in non-static forest instances.

   If instead we ask to combined welded Links we should get

    tree      {world static7 static6 static8} [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6]    =>{base11->link10} (added 6dof, unmodeled weld)
     2 [7]    =>free9            (added 6dof, free bodies are always last)

   Link Composites: {0 7 6 8} {11 10}
   Composite Mobods: [0]  (just the World "composite")
   */

  const SpanningForest& forest =
      graph.BuildForest(ModelingOptions::Default,
                        {{static_model_instance, ModelingOptions::Static}});
  graph.DumpGraph("SerialChain (not combined)");
  forest.SanityCheckForest();
  forest.DumpForest("SerialChain (not combined)");

  EXPECT_EQ(ssize(graph.joints()) - graph.num_user_joints(), 4);
  const std::vector<BodyIndex> link_composites0{BodyIndex(0), BodyIndex(7),
                                                BodyIndex(6), BodyIndex(8)};
  const std::vector<BodyIndex> link_composites1{BodyIndex(11), BodyIndex(10)};
  const std::vector expected_link_composites{link_composites0,
                                             link_composites1};
  EXPECT_EQ(graph.link_composites(), expected_link_composites);

  EXPECT_EQ(ssize(forest.mobods()), 12);
  EXPECT_EQ(ssize(forest.trees()), 6);
  EXPECT_EQ(forest.num_positions(), 19);
  EXPECT_EQ(forest.num_velocities(), 17);

  // Check inboard & outboard coordinate counts.

  // Counts for World.
  EXPECT_EQ(forest.world_mobod().nq(), 0);
  EXPECT_EQ(forest.world_mobod().nv(), 0);
  EXPECT_EQ(forest.world_mobod().nq_inboard(), 0);
  EXPECT_EQ(forest.world_mobod().nv_inboard(), 0);
  EXPECT_EQ(forest.world_mobod().nq_outboard(), forest.num_positions());
  EXPECT_EQ(forest.world_mobod().nv_outboard(), forest.num_velocities());
  EXPECT_EQ(forest.world_mobod().num_subtree_mobods(), ssize(forest.mobods()));

  EXPECT_EQ(
      graph.FindSubtreeLinks(graph.world_link().index()),
      (std::vector{BodyIndex(0), BodyIndex(1), BodyIndex(2), BodyIndex(3),
                   BodyIndex(4), BodyIndex(5), BodyIndex(7), BodyIndex(6),
                   BodyIndex(8), BodyIndex(11), BodyIndex(10), BodyIndex(9)}));

  // Counts for generic middle Mobod.
  const SpanningForest::Mobod& mobod_for_link3 =
      forest.mobods(graph.links(BodyIndex(3)).mobod_index());
  EXPECT_EQ(mobod_for_link3.link(), BodyIndex(3));
  EXPECT_EQ(mobod_for_link3.q_start(), 2);
  EXPECT_EQ(mobod_for_link3.v_start(), 2);
  EXPECT_EQ(mobod_for_link3.nq(), 1);
  EXPECT_EQ(mobod_for_link3.nv(), 1);
  EXPECT_EQ(mobod_for_link3.nq_inboard(), 3);
  EXPECT_EQ(mobod_for_link3.nv_inboard(), 3);
  EXPECT_EQ(mobod_for_link3.nq_outboard(), 2);
  EXPECT_EQ(mobod_for_link3.nv_outboard(), 2);
  EXPECT_EQ(mobod_for_link3.num_subtree_mobods(), 3);
  EXPECT_EQ(graph.FindSubtreeLinks(BodyIndex(3)),
            (std::vector{BodyIndex(3), BodyIndex(4), BodyIndex(5)}));

  // Counts for a Mobod with nq != nv.
  const SpanningForest::Mobod& mobod_for_base11 =
      forest.mobods(graph.links(BodyIndex(11)).mobod_index());
  EXPECT_EQ(mobod_for_base11.q_start(), 5);
  EXPECT_EQ(mobod_for_base11.v_start(), 5);
  EXPECT_EQ(mobod_for_base11.nq(), 7);
  EXPECT_EQ(mobod_for_base11.nv(), 6);
  EXPECT_EQ(mobod_for_base11.nq_inboard(), 7);
  EXPECT_EQ(mobod_for_base11.nv_inboard(), 6);
  EXPECT_EQ(mobod_for_base11.nq_outboard(), 0);
  EXPECT_EQ(mobod_for_base11.nv_outboard(), 0);
  EXPECT_EQ(mobod_for_base11.num_subtree_mobods(), 2);
  EXPECT_EQ(graph.FindSubtreeLinks(BodyIndex(11)),
            (std::vector{BodyIndex(11), BodyIndex(10)}));

  const std::vector<MobodIndex> welded_mobods0{MobodIndex(0), MobodIndex(6),
                                               MobodIndex(7), MobodIndex(8)};
  const std::vector<MobodIndex> welded_mobods1{MobodIndex(9), MobodIndex(10)};
  const std::vector expected{welded_mobods0, welded_mobods1};
  EXPECT_EQ(forest.welded_mobods(), expected);

  auto find_outv = [&forest](int mobod_index) -> pair<int, int> {
    return forest.mobods(MobodIndex(mobod_index)).outboard_velocities();
  };
  EXPECT_EQ(find_outv(0), pair(0, 17));                      // World
  EXPECT_TRUE(forest.mobods(MobodIndex(1)).is_base_body());  // Base bodies
  EXPECT_TRUE(forest.mobods(MobodIndex(7)).is_base_body());
  EXPECT_TRUE(forest.mobods(MobodIndex(9)).is_base_body());
  EXPECT_TRUE(forest.mobods(MobodIndex(11)).is_base_body());
  EXPECT_EQ(find_outv(1), pair(1, 4));
  EXPECT_EQ(find_outv(6), pair(5, 0));
  EXPECT_EQ(find_outv(9), pair(11, 0));
  EXPECT_EQ(find_outv(11), pair(17, 0));
  EXPECT_FALSE(forest.mobods(MobodIndex(3)).is_base_body());  // Generic case
  EXPECT_EQ(find_outv(3), pair(3, 2));

  graph.BuildForest(ModelingOptions::CombineLinkComposites,
                    {{model_instance, ModelingOptions::CombineLinkComposites},
                     {static_model_instance, ModelingOptions::Static}});
  graph.DumpGraph("SerialChain (combined)");
  forest.SanityCheckForest();
  forest.DumpForest("SerialChain (combined)");

  // The graph shouldn't have changed, but the forest will.
  EXPECT_EQ(ssize(graph.joints()) - graph.num_user_joints(), 4);
  EXPECT_EQ(graph.link_composites(), expected_link_composites);

  EXPECT_EQ(ssize(forest.mobods()), 8);
  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World

  // Starting with a Link somewhere in a composite, we should get all the
  // Links on that composite follwoed by anything outboard.
  EXPECT_EQ(graph.FindSubtreeLinks(BodyIndex(10)),
            (std::vector{BodyIndex(11), BodyIndex(10)}));
  EXPECT_EQ(
      graph.FindSubtreeLinks(BodyIndex(6)),
      (std::vector{BodyIndex(0), BodyIndex(7), BodyIndex(6), BodyIndex(8),
                   BodyIndex(1), BodyIndex(2), BodyIndex(3), BodyIndex(4),
                   BodyIndex(5), BodyIndex(11), BodyIndex(10), BodyIndex(9)}));

  // Now force one of the joints in the composite to be modeled (meaning it
  // should get its own Mobod). This should split the World composite into
  // two Mobods, which should be noted as composite Mobods.
  graph.change_joint_flags(joint_10_11_index, JointFlags::MustBeModeled);
  // Model with same options as above.
  graph.BuildForest(ModelingOptions::CombineLinkComposites,
                    {{model_instance, ModelingOptions::CombineLinkComposites},
                     {static_model_instance, ModelingOptions::Static}});
  forest.SanityCheckForest();
  forest.DumpForest("SerialChain (combined, split composite 10,11)");

  EXPECT_EQ(ssize(forest.mobods()), 9);
  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(ssize(forest.welded_mobods()), 2);
  const std::vector<MobodIndex> now_expected{MobodIndex(6), MobodIndex(7)};
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(1)), now_expected);

  /* Build again but with the FixedBase option for model_instance, and allow
  joint_10_11 to be part of a composite (default behavior). Now we expect

  World: {world static7 static6 static8 base11 link10 free9} [0]
  tree
    0  [1-5]  ->link1->link2->link3->link4->link5

  Link Composites: {0 7 6 8 11 10 9}
  Composite Mobods: [0]  (just World)
  */
  graph.change_joint_flags(joint_10_11_index, JointFlags::Default);
  graph.BuildForest(ModelingOptions::CombineLinkComposites,
                    {{model_instance, ModelingOptions::CombineLinkComposites |
                                          ModelingOptions::UseFixedBase},
                     {static_model_instance, ModelingOptions::Static}});
  graph.DumpGraph("SerialChain (combined, split composite 10,11, fixed base)");
  forest.SanityCheckForest();
  forest.DumpForest(
      "SerialChain (combined, split composite 10,11, fixed base)");

  EXPECT_EQ(ssize(forest.mobods()), 6);
  EXPECT_EQ(ssize(forest.trees()), 1);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World
  const std::vector<BodyIndex> expected_link_composite{
      BodyIndex(0),  BodyIndex(7),  BodyIndex(6), BodyIndex(8),
      BodyIndex(11), BodyIndex(10), BodyIndex(9)};
  EXPECT_EQ(ssize(graph.link_composites()), 1);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0)),
            expected_link_composite);
}

/* This is a straightforward graph with two trees each with multiple branches.
There are no welds or reverse joints or loops. We'll use this to test that
basic numbering and reporting works.

              Links                                Mobods

                           14                                   13
                     12  13                               11  12
            9  10      11                        5  6       10
        5    6         8   7      -->        3    4         9   14
           4             3                      2             8
           1             2                      1             7
            ......0......                        ......0......

Note the depth-first ordering in the Forest. We won't provide joints to World
but the modeler should be able to figure out that Links 1 and 2 need them.
*/
GTEST_TEST(LinkJointGraph, MultipleBranches) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  const ModelInstanceIndex model_instance(5);  // arbitrary

  // Define the forest.
  for (int i = 1; i <= 14; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  // Check against left-hand diagram above.
  const std::vector<std::pair<int, int>> joints{
      {1, 4}, {2, 3},  {3, 8},  {3, 7},   {4, 5},   {4, 6},
      {6, 9}, {6, 10}, {8, 11}, {11, 12}, {11, 13}, {13, 14}};
  for (int i = 0; i < ssize(joints); ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance, kRevoluteType,
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  const SpanningForest& forest = graph.BuildForest();  // Default options
  graph.DumpGraph("MultipleBranches");
  forest.DumpForest("MultipleBranches");

  EXPECT_EQ(forest.options(), ModelingOptions::Default);
  EXPECT_EQ(forest.options(model_instance), ModelingOptions::Default);
  EXPECT_EQ(&forest.graph(), &graph);  // check backpointer
  EXPECT_EQ(&forest.links(), &graph.links());
  EXPECT_EQ(&forest.joints(), &graph.joints());

  EXPECT_EQ(graph.num_user_links(), 15);   // includes World
  EXPECT_EQ(ssize(graph.links()), 15);     // no links added
  EXPECT_EQ(graph.num_user_joints(), 12);  // the ones added above
  EXPECT_EQ(ssize(graph.joints()), 14);    // modeling adds two floating joints
  EXPECT_EQ(ssize(graph.link_composites()), 1);  // just World

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(ssize(forest.mobods()), 15);        // includes World
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World
  EXPECT_EQ(forest.num_positions(), 26);  // 12 revolute, 2 x 17 quat floating
  EXPECT_EQ(forest.num_velocities(), 24);

  // Check Link->Mobod and Mobod->Link mappings.
  const std::vector<pair<int, int>> link_mobod_map{
      {0, 0}, {1, 1}, {2, 7},  {3, 8},   {4, 2},   {5, 3},   {6, 4},  {7, 14},
      {8, 9}, {9, 5}, {10, 6}, {11, 10}, {12, 11}, {13, 12}, {14, 13}};
  for (auto link_mobod : link_mobod_map) {
    EXPECT_EQ(graph.link_to_mobod(BodyIndex(link_mobod.first)),
              MobodIndex(link_mobod.second));
    EXPECT_EQ(forest.mobod_to_link(MobodIndex(link_mobod.second)),
              BodyIndex(link_mobod.first));
    EXPECT_EQ(ssize(forest.mobod_to_links(MobodIndex(link_mobod.second))), 1);
    EXPECT_EQ(forest.mobod_to_links(MobodIndex(link_mobod.second))[0],
              BodyIndex(link_mobod.first));
  }

  const SpanningForest::Tree& tree0 = forest.trees(TreeIndex(0));
  const SpanningForest::Tree& tree1 = forest.trees(TreeIndex(1));
  EXPECT_EQ(tree0.index(), TreeIndex(0));
  EXPECT_EQ(tree1.index(), TreeIndex(1));
  EXPECT_EQ(tree0.height(), 4);
  EXPECT_EQ(tree1.height(), 6);
  EXPECT_EQ(tree0.base_mobod(), MobodIndex(1));
  EXPECT_EQ(tree0.last_mobod(), MobodIndex(6));
  EXPECT_EQ(tree1.base_mobod(), MobodIndex(7));
  EXPECT_EQ(tree1.last_mobod(), MobodIndex(14));
  EXPECT_EQ(tree0.num_mobods(), 6);
  EXPECT_EQ(tree1.num_mobods(), 8);
  EXPECT_EQ(tree0.nq(), 12);
  EXPECT_EQ(tree0.nv(), 11);
  EXPECT_EQ(tree1.nq(), 14);
  EXPECT_EQ(tree1.nv(), 13);
  EXPECT_EQ(&tree0.front(), &forest.mobods(MobodIndex(1)));
  EXPECT_EQ(&tree0.back(), &forest.mobods(MobodIndex(6)));
  EXPECT_EQ(&tree1.front(), &forest.mobods(MobodIndex(7)));
  EXPECT_EQ(&tree1.back(), &forest.mobods(MobodIndex(14)));
  EXPECT_EQ(tree0.begin(), &tree0.front());
  EXPECT_EQ(tree0.end(), &tree0.back() + 1);
  EXPECT_EQ(tree1.begin(), &tree1.front());
  EXPECT_EQ(tree1.end(), &tree1.back() + 1);
  EXPECT_EQ(tree0.q_start(), 0);
  EXPECT_EQ(tree0.nq(), 12);
  EXPECT_EQ(tree1.q_start(), 12);
  EXPECT_EQ(tree1.nq(), 14);
  EXPECT_EQ(tree0.v_start(), 0);
  EXPECT_EQ(tree0.nv(), 11);
  EXPECT_EQ(tree1.v_start(), 11);
  EXPECT_EQ(tree1.nv(), 13);

  // Sample some q's and v's to see if they can find their tree and mobod.
  EXPECT_EQ(forest.q_to_tree(9), TreeIndex(0));
  EXPECT_EQ(forest.q_to_tree(19), TreeIndex(1));
  EXPECT_EQ(forest.v_to_tree(9), TreeIndex(0));
  EXPECT_EQ(forest.v_to_tree(19), TreeIndex(1));
  EXPECT_EQ(forest.q_to_mobod(9), MobodIndex(4));
  EXPECT_EQ(forest.q_to_mobod(20), MobodIndex(9));
  EXPECT_EQ(forest.v_to_mobod(9), MobodIndex(5));
  EXPECT_EQ(forest.v_to_mobod(20), MobodIndex(11));

  auto find_v = [&forest](int mobod_index) -> pair<int, int> {
    return forest.mobods(MobodIndex(mobod_index)).subtree_velocities();
  };
  EXPECT_EQ(find_v(0), pair(0, 24));   // World
  EXPECT_EQ(find_v(1), pair(0, 11));   // base tree0
  EXPECT_EQ(find_v(7), pair(11, 13));  // base tree1
  EXPECT_EQ(find_v(3), pair(7, 1));    // terminal
  EXPECT_EQ(find_v(4), pair(8, 3));    // nonterminal
  EXPECT_EQ(find_v(8), pair(17, 7));   // tree1 nonterminal
  EXPECT_EQ(find_v(10), pair(19, 4));
  EXPECT_EQ(find_v(14), pair(23, 1));  // tree1 terminal

  // "Outboard" velocities don't include those of the selected Mobod.
  auto find_outv = [&forest](int mobod_index) -> pair<int, int> {
    return forest.mobods(MobodIndex(mobod_index)).outboard_velocities();
  };
  EXPECT_EQ(find_outv(0), pair(0, 24));  // same Mobods as above
  EXPECT_EQ(find_outv(1), pair(6, 5));
  EXPECT_EQ(find_outv(7), pair(17, 7));
  EXPECT_EQ(find_outv(3), pair(8, 0));
  EXPECT_EQ(find_outv(4), pair(9, 2));
  EXPECT_EQ(find_outv(8), pair(18, 6));
  EXPECT_EQ(find_outv(10), pair(20, 3));
  EXPECT_EQ(find_outv(14), pair(24, 0));

  const std::vector<MobodIndex> expected_path_from_14{
      {MobodIndex(0)}, {MobodIndex{7}}, {MobodIndex(8)}, {MobodIndex(14)}};
  EXPECT_EQ(forest.FindPathFromWorld(MobodIndex(14)), expected_path_from_14);

  // Mobods on different trees have only World as a common ancestor.
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(5), MobodIndex(12)),
            MobodIndex(0));
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(13), MobodIndex(14)),
            MobodIndex(8));  // See right hand drawing above.
}

/* We build a Forest containing a number of kinematic loops and subgraphs of
welded bodies. Joints are expressed in (parent, child) order.

subgraph A (forms closed loop):
 - WeldJoint(1, 13)
 - WeldJoint(4, 1)
 - WeldJoint(13, 4)

subgraph B (forms closed loop):
 - WeldJoint(10, 6)
 - WeldJoint(10, 8)
 - WeldJoint(6, 8)

subgraph C (the "world" subgraph):
 - WeldJoint(5, 7)
 - WeldJoint(5, 0)
 - WeldJoint(12, 5)

Non-weld joints kinematic loop:
 - PrimaticJoint(2, 11)
 - PrimaticJoint(7, 2)
 - RevoluteJoint(7, 11)

Additionally we have the following non-weld joints:
 - RevoluteJoint(3, 13): connects Link 3 to subgraph A.
 - PrimaticJoint(1, 10): connects subgraph A and B.

The input is given as three unconnected graphs. Joints are shown with
parent->child direction. Double bars are welds, single bars are moving joints.
Links {0-13} are shown in braces, joint numbers 0-13 are plain. Subgraphs
A, B, C formed by welds are labeled to match the description above.

            C
         12    11     9
    {0}<==={5}===>{7}--->{2}
  World     ^      |10    |
          13‖      v      |8
           {12}   {11}<---+                  Link/Joint graph as input

        3        0      7      4
    {3}--->{13}<==={1}--->{10}===>{6}
            2‖  A   ^      5‖  B   ‖
             v      ‖1      v      ‖6
            {4}=====+      {8}<====+

    {9}

When we build the forest, we have to provide every link with a path to World.
We'll first process the upper graph which already starts at World. Then we
have to pick a base body for the next graph. Link {3} should be chosen since
it appears only as a parent; it gets floating joint 14. Link {9} will also be
a base body; it gets floating joint 15.

There are three loops in this graph: {7-2-11}, {13-1-4}, and {10-6-8}. The last
two are formed entirely of welds. When modeling in the mode where every Joint
gets a Mobod all of these must be broken by adding shadow links. Because of
the processing order, Link {11} will be split with shadow link {14*} on Joint 8,
Link {1} gets shadow {15*} on weld Joint 1, and Link {8} gets shadow {16*} on
weld Joint 6. (Link {1} gets split rather than {4} to preserve parent->child
order of Joint 1.)

Therefore we expect the following Composites, with the "World" Composite first:
  {0, 5, 7, 12}, {13, 1, 4, 15*}, {10, 6, 8, 16*}
and the remaining non-composite links are {3}, {9}, {2}, {11}, {14*}.

Forest building should start with Link {5} since that is the only direct
connection to World in the input ({3} and {9} get connected later). If we're
giving every Link its own mobilizers (rather than making composites from
welded-together ones) we expect this forest of 3 trees and 17 Mobods:

      level 6                 12{16}
      level 5                  11{6}  13{8}
      level 4  4{14}             10{10}     15{15}
      level 3   3{2} 5{11}         9{1}   14{4}
      level 2      2{7}   6{12}       8{13}
  base mobods          1{5}           7{3}            16{9}
                          \            |               /
        World              ...........0{0}.............

Some of the Links are welded together. We call those Link Composites even
though each has its own Mobod. Those are:
{0 5 7 12} {13 1 4 15} {10 6 8 16}
The corresponding Mobods are Composite Mobods:
[0 1 2 6] [8 9 14 15] [10 11 13 12]

Remodeling with composite link combining turned on should immediately create
composite {0 5 7 12} on mobod 0, then see outboard links {2} and {11} as new
base bodies and grow those two trees, discovering a loop at joint 8. As before,
Link {11} gets split with a shadow link {14} for joint 8. Then it
should choose link {3} as a base link and add floating joint 14, and grow that
tree. Finally it makes free link {9} a base body. The forest should then look
like this:


      level 3                         6{10 6 8}
      level 2      2{14}              5{13 1 4}
  base mobods       1{2}    3{11}     4{3}        7{9}  (four trees)
                      \       \        |           /
        World          ...........0{0 5 7 12}......

In this case we don't need to split the all-Weld loops since they are now
just composite links {0 5 7 12} {13 1 4} {10 6 8}. There are no Composite
Mobods (except World alone). */
GTEST_TEST(LinkJointGraph, Weldedsubgraphs) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);
  EXPECT_EQ(ssize(graph.joint_types()), 5);  // predefined, revolute, prismatic.

  // We'll add Links and Joints to this forest instance.
  const ModelInstanceIndex model_instance(5);

  // Define the forest.
  for (int i = 1; i <= 13; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
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

  // Link 3 connects to subgraph A via a revolute joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 BodyIndex(3), BodyIndex(13));

  // subgraph B: formed by bodies 6, 8, 10.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(10), BodyIndex(6));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(10), BodyIndex(8));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(6), BodyIndex(8));

  // Link 1 (in subgraph A) and Link 10 (in subgraph B) connect through a
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

  // subgraph C: formed by links 5, 7, 12 and World 0.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(5), BodyIndex(7));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(5), world_index());
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), BodyIndex(12), BodyIndex(5));

  EXPECT_EQ(ssize(graph.links()), 14);  // this includes the world Link.

  // We can calculate the subgraphs without first building a Forest. This
  // function should only consider user Links, not shadow links even if they
  // have already been created.
  std::vector<std::set<BodyIndex>> expected_before_subgraphs;
  // {0 5 7 12} {1 4 13} {6 8 10} {2} {3} {9} {11}  (see first drawing above)
  expected_before_subgraphs.push_back(
      std::set{BodyIndex(0), BodyIndex(5), BodyIndex(7), BodyIndex(12)});
  expected_before_subgraphs.push_back(
      std::set{BodyIndex(1), BodyIndex(4), BodyIndex(13)});
  expected_before_subgraphs.push_back(
      std::set{BodyIndex(6), BodyIndex(8), BodyIndex(10)});
  expected_before_subgraphs.push_back(std::set{BodyIndex(2)});
  expected_before_subgraphs.push_back(std::set{BodyIndex(3)});
  expected_before_subgraphs.push_back(std::set{BodyIndex(9)});
  expected_before_subgraphs.push_back(std::set{BodyIndex(11)});
  const std::vector<std::set<BodyIndex>> before_subgraphs =
      graph.CalcSubgraphsOfWeldedLinks();
  EXPECT_EQ(before_subgraphs.size(), 7);
  EXPECT_EQ(before_subgraphs, expected_before_subgraphs);

  graph.BuildForest();
  graph.DumpGraph("WeldedSubgraphs (not combined)");
  const SpanningForest& forest = graph.forest();
  forest.SanityCheckForest();
  forest.DumpForest("WeldedSubgraphs (not combined)");

  // After building the Forest and adding shadow bodies, the non-Forest-using
  // subgraph method should not change its result. The Forest-using fast one
  // will include Shadow Links as well as the user's.
  EXPECT_EQ(graph.CalcSubgraphsOfWeldedLinks(),
            expected_before_subgraphs);  // no change

  const std::vector<std::set<BodyIndex>> welded_subgraphs =
      graph.GetSubgraphsOfWeldedLinks();

  // Verify number of expected subgraphs.
  EXPECT_EQ(welded_subgraphs.size(), 8);

  // The first subgraph must contain the world.
  const std::set<BodyIndex> world_subgraph = welded_subgraphs[0];
  EXPECT_EQ(world_subgraph.count(world_index()), 1);

  // Build the expected set of subgraphs (see above).
  std::set<std::set<BodyIndex>> expected_subgraphs;
  // {0, 5, 7, 12}, {1, 4, 13, 15}, {6, 8, 10, 16}, {3}, {9}, {2}, {11}, {14}
  const std::set<BodyIndex>& expected_world_subgraph =
      *expected_subgraphs
           .insert({BodyIndex(0), BodyIndex(5), BodyIndex(7), BodyIndex(12)})
           .first;
  const std::set<BodyIndex>& expected_subgraphA =
      *expected_subgraphs
           .insert({BodyIndex(1), BodyIndex(4), BodyIndex(13), BodyIndex(15)})
           .first;
  const std::set<BodyIndex>& expected_subgraphB =
      *expected_subgraphs
           .insert({BodyIndex(6), BodyIndex(8), BodyIndex(10), BodyIndex(16)})
           .first;
  expected_subgraphs.insert({BodyIndex(3)});
  expected_subgraphs.insert({BodyIndex(9)});
  expected_subgraphs.insert({BodyIndex(2)});
  expected_subgraphs.insert({BodyIndex(11)});
  expected_subgraphs.insert({BodyIndex(14)});

  // We do expect the first subgraph to correspond to the set of bodies welded
  // to the world.
  EXPECT_EQ(world_subgraph, expected_world_subgraph);

  // In order to compare the computed list of welded bodies against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed subgraphs to a set.
  const std::set<std::set<BodyIndex>> welded_subgraphs_set(
      welded_subgraphs.begin(), welded_subgraphs.end());
  EXPECT_EQ(welded_subgraphs_set, expected_subgraphs);

  // Verify we can query the list of bodies welded to a particular Link.
  EXPECT_EQ(graph.GetLinksWeldedTo(BodyIndex(9)).size(), 1);
  EXPECT_EQ(graph.GetLinksWeldedTo(BodyIndex(11)).size(), 1);
  EXPECT_EQ(graph.GetLinksWeldedTo(BodyIndex(4)), expected_subgraphA);
  EXPECT_EQ(graph.GetLinksWeldedTo(BodyIndex(13)), expected_subgraphA);
  EXPECT_EQ(graph.GetLinksWeldedTo(BodyIndex(10)), expected_subgraphB);
  EXPECT_EQ(graph.GetLinksWeldedTo(BodyIndex(6)), expected_subgraphB);

  // Now let's verify that we got the expected SpanningForest. To understand,
  // refer to the taller (max level 6) diagram above.
  EXPECT_EQ(ssize(forest.mobods()), 17);

  // Note that this is a question about how these Links got modeled, not
  // about the original graph.
  EXPECT_EQ(graph.FindFirstCommonAncestor(BodyIndex(11), BodyIndex(12)),
            BodyIndex(5));
  EXPECT_EQ(graph.FindFirstCommonAncestor(BodyIndex(16), BodyIndex(15)),
            BodyIndex(13));
  EXPECT_EQ(graph.FindFirstCommonAncestor(BodyIndex(10), BodyIndex(2)),
            BodyIndex(0));

  // Expected level for each mobod in forest (index by MobodIndex).
  std::array<int, 17> expected_level{0, 1, 2, 3, 4, 3, 2, 1, 2,
                                     3, 4, 5, 6, 5, 3, 4, 1};
  for (auto& mobod : forest.mobods()) {
    EXPECT_EQ(mobod.level(), expected_level[mobod.index()]);
  }

  EXPECT_EQ(ssize(forest.welded_mobods()), 3);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0)),
            (std::vector<MobodIndex>{MobodIndex(0), MobodIndex(1),
                                     MobodIndex(2), MobodIndex(6)}));
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(1)),
            (std::vector<MobodIndex>{MobodIndex(8), MobodIndex(9),
                                     MobodIndex(14), MobodIndex(15)}));
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(2)),
            (std::vector<MobodIndex>{MobodIndex(10), MobodIndex(11),
                                     MobodIndex(13), MobodIndex(12)}));

  // Now combine composites so they get a single Mobod.
  graph.BuildForest(ModelingOptions::CombineLinkComposites);
  graph.DumpGraph("WeldedSubgraphs (combined)");
  forest.SanityCheckForest();
  forest.DumpForest("WeldedSubgraphs (combined)");

  EXPECT_EQ(ssize(graph.links()), 15);  // Only one added shadow.
  EXPECT_EQ(ssize(graph.link_composites()), 3);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0)),
            (std::vector<BodyIndex>{BodyIndex(0), BodyIndex(5), BodyIndex(7),
                                    BodyIndex(12)}));
  EXPECT_EQ(
      graph.link_composites(LinkCompositeIndex(1)),
      (std::vector<BodyIndex>{BodyIndex(13), BodyIndex(1), BodyIndex(4)}));
  EXPECT_EQ(
      graph.link_composites(LinkCompositeIndex(2)),
      (std::vector<BodyIndex>{BodyIndex(10), BodyIndex(6), BodyIndex(8)}));

  // Now let's verify that we got the expected SpanningForest. To understand,
  // refer to the shorter (max level 3) diagram above.
  EXPECT_EQ(ssize(forest.mobods()), 8);
  std::array<int, 8> expected_level_combined{0, 1, 2, 1, 1, 2, 3, 1};
  for (auto& mobod : forest.mobods()) {
    EXPECT_EQ(mobod.level(), expected_level_combined[mobod.index()]);
  }

  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World
  EXPECT_EQ(ssize(forest.welded_mobods(WeldedMobodsIndex(0))), 1);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0))[0], MobodIndex(0));
}

// Ten links, 8 in a tree and 2 free ones.
GTEST_TEST(LinkJointGraph, SimpleTrees) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);

  // We'll add Links and Joints to this forest instance.
  const ModelInstanceIndex model_instance(5);

  // Define the forest.
  for (int i = 1; i <= 10; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }
  const std::vector<std::pair<int, int>> joints{
      {3, 1}, {3, 2}, {8, 3}, {10, 8}, {10, 9}, {9, 4}, {9, 7}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance, kRevoluteType,
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 11);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  // TODO(sherm1) Move to its own test suite.
  graph.BuildForest(ModelingOptions::UseRpyFloatingJoints);
  graph.DumpGraph("SimpleTrees");
  const SpanningForest& forest = graph.forest();
  forest.SanityCheckForest();
  forest.DumpForest("SimpleTrees");

  // Now connect the "leftmost" and "rightmost" Links (1 and 7 resp.)
  // to World explicitly, forming a loop.
  graph.AddJoint("world_joint_7", model_instance, kRevoluteType, BodyIndex(0),
                 BodyIndex(1));
  graph.AddJoint("world_joint_8", model_instance, kRevoluteType, BodyIndex(0),
                 BodyIndex(7));
  graph.BuildForest();
  graph.DumpGraph("SimpleTrees with 1 & 7 connected to World");

  forest.SanityCheckForest();
  forest.DumpForest("SimpleTrees with 1 & 7 connected to World");
}

/* Massless bodies should alter tree-building strategy.

LinkJointGraph:
  World -0-> {1} -1-> {2} -2-> {3}*
                                |
                                3
                                v
  World -4-> {5} -5-> {6} -6-> {4}*

With all massful Links, loop should be broken at Joint 3 (between links
{3} and {4} since that minimizes the maximum chain length. In that case we
have trees {1234s} and {564} where {4s} is the shadow of {4}.

If we make {3}* massless we'll have to extend the first chain
to link {4} before breaking the loop at Joint 6, giving trees
{1234} and {564s} ({4} is still split since it is the child of Joint 6).

If we make _both_ {3}* and {4}* massless, modeling should start with {12} and
{56} but then next extend the first tree to {12346s} because we can't stop at
{3} or {4}. Joint 6 is the loop joint but the mobilizer has to be reversed so
that we end with a massful shadow link {6s} rather than the massless {4}. */
GTEST_TEST(LinkJointGraph, MasslessLinksChangeLoopBreaking) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);

  // We'll add Links and Joints to this forest instance.
  const ModelInstanceIndex model_instance(5);

  // Define the forest.
  for (int i = 1; i <= 6; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }
  const std::vector<std::pair<int, int>> joints{{0, 1}, {1, 2}, {2, 3}, {3, 4},
                                                {0, 5}, {5, 6}, {6, 4}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance, kRevoluteType,
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 7);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  // TODO(sherm1) Move to its own test suite.
  graph.BuildForest();
  graph.DumpGraph("Massful loop");
  const SpanningForest& forest = graph.forest();
  forest.SanityCheckForest();
  forest.DumpForest("Massful loop");

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 4);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 3);

  graph.change_link_flags(BodyIndex(3), LinkFlags::TreatAsMassless);
  graph.BuildForest();
  graph.DumpGraph("Massless loop");
  forest.SanityCheckForest();
  forest.DumpForest("Massless loop");

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 4);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 3);

  graph.change_link_flags(BodyIndex(4), LinkFlags::TreatAsMassless);
  graph.BuildForest();
  graph.DumpGraph("Massless x 2 loop");
  forest.SanityCheckForest();
  forest.DumpForest("Massless x 2 loop");

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 5);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 2);
}

/* Here is a tricky case that should be handled correctly and without warnings.
We have a short loop consisting of two massless base Links and a single massful
Link. The massful Link should be split into two half-massful bodies which are
sufficient to prevent both massless Links from being terminal.

        =====                                =====          =====
        # 3 #              massful           # 2 # - Weld -># 4 #
        =====                                =====          =====
      🡕 2     🡔 3                              🡑              🡑
  ---           ---                           ---            ---
 | 1 |         | 2 |       massless          | 1 |          | 3 |
  ---           ---                           ---            ---
   🡑 0           🡑 1                           🡑 T0           🡑 T1
 ===================        ---->            ====================
        World                                        World

On the left we show the Link and Joint numbers as input, on the right we show
the Tree numbers and mobilized body numbers in proper depth-first order.
Arrows show the parent->child and inboard->outboard directions. We
expect to process Link 1 before Link 2 so we expect Tree 0 to contain the
primary Mobod (2) for Link 3, with Tree 1 containing its shadow Mobod (4).
*/
GTEST_TEST(LinkJointGraph, MasslessBodiesShareSplitLink) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);
  const ModelInstanceIndex model_instance(19);

  graph.AddLink("massless_1", model_instance, LinkFlags::TreatAsMassless);
  graph.AddLink("massless_2", model_instance, LinkFlags::TreatAsMassless);
  graph.AddLink("link_3", model_instance);

  graph.AddJoint("prismatic_0", model_instance, kPrismaticType, world_index(),
                 BodyIndex(1));
  graph.AddJoint("prismatic_1", model_instance, kPrismaticType, world_index(),
                 BodyIndex(2));
  graph.AddJoint("revolute_2", model_instance, kRevoluteType, BodyIndex(1),
                 BodyIndex(3));
  graph.AddJoint("revolute_3", model_instance, kRevoluteType, BodyIndex(2),
                 BodyIndex(3));

  EXPECT_EQ(ssize(graph.links()), 4);  // Before modeling (includes World).
  EXPECT_EQ(graph.num_user_links(), 4);

  const SpanningForest& forest = graph.BuildForest();
  graph.DumpGraph("MasslessBodiesShareSplitLink");
  forest.SanityCheckForest();
  forest.DumpForest("MasslessBodiesShareSplitLink");

  EXPECT_EQ(ssize(graph.links()), 5);  // After modeling.
  EXPECT_EQ(graph.num_user_links(), 4);
  EXPECT_EQ(ssize(graph.constraints()), 1);
  EXPECT_EQ(graph.num_user_constraints(), 0);

  const auto& shadow_link = graph.links(BodyIndex(4));
  EXPECT_TRUE(shadow_link.is_shadow());
  EXPECT_EQ(shadow_link.primary_link(), BodyIndex(3));
  EXPECT_EQ(shadow_link.mobod_index(), MobodIndex(4));
  EXPECT_EQ(shadow_link.inboard_joint_index(), JointIndex(3));
  EXPECT_EQ(ssize(shadow_link.joints()), 1);
  EXPECT_TRUE(shadow_link.joints_as_parent().empty());
  EXPECT_EQ(shadow_link.joints_as_child()[0], JointIndex(3));
  EXPECT_EQ(shadow_link.joints()[0], JointIndex(3));

  EXPECT_EQ(graph.links(BodyIndex(3)).num_shadows(), 1);
  EXPECT_EQ(graph.links(BodyIndex(2)).num_shadows(), 0);
  EXPECT_EQ(graph.links(BodyIndex(4)).num_shadows(), 0);

  EXPECT_EQ(ssize(forest.mobods()), 5);
  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees(TreeIndex(0)).num_mobods(), 2);
  EXPECT_EQ(forest.trees(TreeIndex(1)).num_mobods(), 2);
}

/* Here we have a floating double loop requiring two shadows of the same Link:

     {2} ------> {5}           Link numbers are in {}
      ^     3     | 6          Joint numbers are plain
    0 |           v            Arrows show parent->child direction
     {1}-->{3}-->{6}           All Links are massful
    1 |  2     5  ^            All Joints are articulated (no welds)
      v           | 7
     {4} ------> {7}
            4

Because Link {1} is never used as a child, it will be the preferred base Link
and get attached to World by a free Joint. The heuristic that tries to
minimize branch length should grow the three branches from {1} like this:
  {12}    {14}     {13}
  {125}   {147}    {136}
  {1256s} {1476ss}  where 6s and 6ss are shadows 1 and 2 of Link 6.

The expected as-modeled graph and spanning forest model are:

            {2}--->{5}--->{8}                  [2]-->[3]-->[4]  branch 1
           0 ^  3      6   # weld 0             ^
  World      |             #            World   |
   {0}----->{1}--->{3}--->{6}            [0]-->[1]-->[8]-->[9]  branch 3
        8    |  2      5   # weld 1             |
           1 v             #                    v
            {4}--->{7}--->{9}                  [5]-->[6]-->[7]  branch 2
                4      7

            Links & Joints                     Mobilized bodies

Notes:
  - Joint numbering determines branch ordering in the tree so the
    middle branch gets modeled last.
  - Model Joint 8 is the added floating joint to World.
  - Link {8} is {6s} (shadow 1 of Link {6}); {9} is {6ss} (shadow 2).
  - Welds should be oriented 6->8 and 6->9.
  - Mobilized bodies (Mobods) are numbered depth-first.
*/
GTEST_TEST(LinkJointGraph, DoubleLoop) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);
  const ModelInstanceIndex model_instance(19);

  for (int i = 1; i <= 7; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  const std::vector<std::pair<int, int>> joints{{1, 2}, {1, 4}, {1, 3}, {2, 5},
                                                {4, 7}, {3, 6}, {5, 6}, {7, 6}};
  for (int i = 0; i < 8; ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance, kRevoluteType,
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 8);  // Before modeling (includes World).
  EXPECT_EQ(ssize(graph.joints()), 8);
  EXPECT_EQ(ssize(graph.constraints()), 0);

  const SpanningForest& forest = graph.BuildForest();
  graph.DumpGraph("DoubleLoop");
  forest.SanityCheckForest();
  forest.DumpForest("DoubleLoop");

  EXPECT_EQ(ssize(graph.links()), 10);  // After modeling.
  EXPECT_EQ(ssize(graph.joints()), 9);
  EXPECT_EQ(ssize(graph.constraints()), 2);
  EXPECT_EQ(graph.num_user_links(), 8);
  EXPECT_EQ(graph.num_user_joints(), 8);
  EXPECT_EQ(graph.num_user_constraints(), 0);

  EXPECT_EQ(graph.links(BodyIndex(6)).num_shadows(), 2);
  EXPECT_TRUE(graph.links(BodyIndex(8)).is_shadow());
  EXPECT_TRUE(graph.links(BodyIndex(9)).is_shadow());

  EXPECT_EQ(graph.links(BodyIndex(5)).num_shadows(), 0);
  EXPECT_EQ(graph.links(BodyIndex(7)).num_shadows(), 0);

  EXPECT_EQ(ssize(forest.mobods()), 10);
  EXPECT_EQ(ssize(forest.trees()), 1);
  const SpanningForest::Tree& tree = forest.trees(TreeIndex(0));
  EXPECT_EQ(tree.num_mobods(), 9);
  EXPECT_EQ(tree.height(), 4);
  EXPECT_EQ(tree.base_mobod(), MobodIndex(1));
  EXPECT_EQ(tree.last_mobod(), MobodIndex(9));

  // ith entry gives the modeled Link or Joint for Mobod i (see picture above).
  const std::array mobod2link{0, 1, 2, 5, 8, 4, 7, 9, 3, 6};
  const std::array mobod2joint{-1, 8, 0, 3, 6, 1, 4, 7, 2, 5};
  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    EXPECT_EQ(mobod.link(), BodyIndex(mobod2link[mobod.index()]));
    if (mobod.is_world()) continue;  // No joint for World mobod.
    EXPECT_EQ(mobod.joint(), JointIndex(mobod2joint[mobod.index()]));
  }

  const LinkJointGraph::Constraint& weld0 =
      graph.constraints(ConstraintIndex(0));
  const LinkJointGraph::Constraint& weld1 =
      graph.constraints(ConstraintIndex(1));
  EXPECT_EQ(weld0.index(), 0);
  EXPECT_EQ(weld1.index(), 1);
  EXPECT_EQ(weld0.model_instance(), model_instance);
  EXPECT_EQ(weld1.model_instance(), model_instance);
  EXPECT_EQ(weld0.parent_link(), BodyIndex(6));
  EXPECT_EQ(weld0.child_link(), BodyIndex(8));
  EXPECT_EQ(weld1.parent_link(), BodyIndex(6));
  EXPECT_EQ(weld1.child_link(), BodyIndex(9));

  // Added welds should be named the same as their shadow Link.
  EXPECT_EQ(weld0.name(), graph.links(BodyIndex(8)).name());
  EXPECT_EQ(weld1.name(), graph.links(BodyIndex(9)).name());
}

/* Composite bodies should be treated the same as single bodies while
building the trees a level at a time. We'll create a loop out of two
trees, one composed of two-body composites and the other single bodies.
Our loop-splitting algorithm should result in two trees of equal length in
mobilized bodies though unequal in links.

              0               1             2
      +--> {1}==>{2} ---> {3}==>{4} ---> {5}==>{6}
  {0} | 3             4              5          | 10   {Links} & Joints
      |                                         v
      +--->   {7}  --->  {8}  --->  {9}  ---> {10}
        6           7          8          9


      +---> [1] ---> [2] ---> [3] ---> [4] {10}
      |    {1,2}    {3,4}    {5,6}      #
  [0] |                                 # Weld            [Mobods]
      |     {7}      {8}      {9}       V
      +---> [5] ---> [6] ---> [7] ---> [8] {10s}
*/

// TODO(sherm1) Need to check proper massless/massful behavior for massless
//  Composites.
GTEST_TEST(LinkJointGraph, LoopWithComposites) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);
  const ModelInstanceIndex model_instance(19);

  for (int i = 1; i <= 10; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  const std::vector<std::pair<int, int>> weld_joints{{1, 2}, {3, 4}, {5, 6}};
  const std::vector<std::pair<int, int>> revolute_joints{
      {0, 1}, {2, 3}, {4, 5}, {0, 7}, {7, 8}, {8, 9}, {9, 10}, {6, 10}};
  for (int i = 0; i < ssize(weld_joints); ++i) {
    graph.AddJoint("weld_joint_" + std::to_string(i), model_instance, "weld",
                   BodyIndex(weld_joints[i].first),
                   BodyIndex(weld_joints[i].second));
  }
  for (int i = 0; i < ssize(revolute_joints); ++i) {
    const int j = ssize(weld_joints) + i;  // joint number
    graph.AddJoint("revolute_joint_" + std::to_string(j), model_instance,
                   kRevoluteType, BodyIndex(revolute_joints[i].first),
                   BodyIndex(revolute_joints[i].second));
  }

  // Before modeling
  EXPECT_EQ(ssize(graph.links()), 11);  // counting World
  EXPECT_EQ(ssize(graph.joints()), 11);
  EXPECT_EQ(ssize(graph.constraints()), 0);

  const SpanningForest& forest =
      graph.BuildForest(ModelingOptions::CombineLinkComposites);
  forest.SanityCheckForest();

  // After modeling
  EXPECT_EQ(ssize(graph.links()), 12);           // split one, added shadow
  EXPECT_EQ(ssize(graph.joints()), 11);          // no change
  EXPECT_EQ(ssize(graph.constraints()), 1);      // welded shadow to primary
  EXPECT_EQ(ssize(graph.link_composites()), 4);  // World + 3

  EXPECT_EQ(ssize(forest.mobods()), 9);
  EXPECT_EQ(ssize(forest.loop_constraints()), 1);
  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World

  EXPECT_EQ(forest.mobods(MobodIndex(1)).follower_links(),
            (std::vector<BodyIndex>{BodyIndex(1), BodyIndex(2)}));
  EXPECT_EQ(forest.mobods(MobodIndex(2)).follower_links(),
            (std::vector<BodyIndex>{BodyIndex(3), BodyIndex(4)}));
  EXPECT_EQ(forest.mobods(MobodIndex(3)).follower_links(),
            (std::vector<BodyIndex>{BodyIndex(5), BodyIndex(6)}));

  const SpanningForest::Tree tree0 = forest.trees(TreeIndex(0)),
                             tree1 = forest.trees(TreeIndex(1));
  EXPECT_EQ(tree0.num_mobods(), 4);
  EXPECT_EQ(tree0.nq(), 4);
  EXPECT_EQ(tree1.num_mobods(), 4);
  EXPECT_EQ(tree1.nq(), 4);

  graph.DumpGraph("LoopWithComposites");
  forest.DumpForest("LoopWithComposites");

  // Sanity checks for graph copying and assignment. These are mostly
  // compiler-generated so we just need to test any field and that the
  // bespoke backpointer adjustments get done correctly.
  LinkJointGraph graph_copy(graph);
  EXPECT_EQ(ssize(graph_copy.links()), 12);
  EXPECT_TRUE(graph_copy.forest_is_valid());
  const SpanningForest& copy_model = graph_copy.forest();
  copy_model.SanityCheckForest();
  EXPECT_NE(&copy_model, &forest);
  EXPECT_EQ(&copy_model.graph(), &graph_copy);  // backpointer

  LinkJointGraph graph_assign;
  graph_assign = graph;
  EXPECT_EQ(ssize(graph_assign.links()), 12);
  EXPECT_TRUE(graph_assign.forest_is_valid());
  EXPECT_NE(&graph_assign.forest(), &forest);
  graph_assign.forest().SanityCheckForest();
  EXPECT_EQ(&graph_assign.forest().graph(), &graph_assign);

  LinkJointGraph graph_move(std::move(graph));
  EXPECT_EQ(ssize(graph_move.links()), 12);
  EXPECT_EQ(ssize(graph.links()), 1);  // Just world now.
  EXPECT_EQ(&graph_move.forest(), &forest);
  graph_move.forest().SanityCheckForest();
  EXPECT_EQ(&graph_move.forest().graph(), &graph_move);
  // graph is now default-constructed so still has a forest
  EXPECT_NE(&graph.forest(), &forest);
  EXPECT_FALSE(graph.forest_is_valid());
  EXPECT_EQ(&graph.forest().graph(), &graph);
  graph.forest().SanityCheckForest();  // Should be empty but OK

  LinkJointGraph graph_move_assign;
  graph_move_assign = std::move(graph_copy);
  EXPECT_EQ(ssize(graph_move_assign.links()), 12);
  EXPECT_TRUE(graph_move_assign.forest_is_valid());
  EXPECT_EQ(&graph_move_assign.forest(), &copy_model);
  graph_move_assign.forest().SanityCheckForest();
  EXPECT_EQ(&graph_move_assign.forest().graph(), &graph_move_assign);
  // graph_copy is now default-constructed. Should have world and a
  // new (empty) forest.
  EXPECT_EQ(ssize(graph_copy.links()), 1);
  EXPECT_NE(&graph_copy.forest(), &copy_model);
  EXPECT_FALSE(graph_copy.forest_is_valid());
  EXPECT_EQ(&graph_copy.forest().graph(), &graph_copy);
  graph_copy.forest().SanityCheckForest();  // Should be empty but OK.
}

/* For both link_composites and welded_mobods: the World composite must
come first (even if nothing is welded to World). This graph's first branch has
a composite that could be seen prior to the weld to World. We'll attempt
to trick it into following that path by using a massless body, requiring it
to extend the first branch to Link {2} before moving on to the next branch.
But we want to see the {0,3} composite before the {1,2} composite.

          +---> {1*} ===> {2}
      {0} | 0         1                {Links} & Joints
    World |                            ===> is a weld
          +===> {3}                    * Link 1 is massless
          | 2
          |
          +---> {4}
            3
*/
GTEST_TEST(LinkJointGraph, WorldCompositeComesFirst) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  const ModelInstanceIndex model_instance(5);  // arbitrary

  graph.AddLink("massless_link_1", model_instance, LinkFlags::TreatAsMassless);
  graph.AddLink("link2", model_instance);
  graph.AddLink("link3", model_instance);
  graph.AddLink("link4", model_instance);

  const auto& world = graph.links(BodyIndex(0));
  const auto& massless_link = graph.links(BodyIndex(1));
  const auto& link2 = graph.links(BodyIndex(2));
  const auto& link3 = graph.links(BodyIndex(3));
  const auto& link4 = graph.links(BodyIndex(4));

  graph.AddJoint("joint0", model_instance, kRevoluteType, world.index(),
                 massless_link.index());
  graph.AddJoint("joint1", model_instance, "weld", massless_link.index(),
                 link2.index());
  graph.AddJoint("joint2", model_instance, "weld", world.index(),
                 link3.index());
  graph.AddJoint("joint4", model_instance, kRevoluteType, world.index(),
                 link4.index());

  const SpanningForest& forest = graph.BuildForest();  // Default options
  forest.SanityCheckForest();
  graph.DumpGraph("WorldCompositeComesFirst");
  forest.DumpForest("WorldCompositeComesFirst");

  EXPECT_EQ(ssize(graph.links()), 5);
  EXPECT_EQ(ssize(forest.mobods()), 5);  // Because we're not combining.

  // "Anchored" means "fixed to World" (by welds).
  EXPECT_TRUE(world.is_anchored());
  EXPECT_FALSE(massless_link.is_anchored());
  EXPECT_FALSE(link2.is_anchored());
  EXPECT_TRUE(link3.is_anchored());
  EXPECT_FALSE(link4.is_anchored());

  EXPECT_EQ(ssize(graph.link_composites()), 2);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0)),
            (std::vector<BodyIndex>{BodyIndex(0), BodyIndex(3)}));
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(1)),
            (std::vector<BodyIndex>{BodyIndex(1), BodyIndex(2)}));

  EXPECT_EQ(ssize(forest.welded_mobods()), 2);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0)),
            (std::vector<MobodIndex>{MobodIndex(0), MobodIndex(3)}));
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(1)),
            (std::vector<MobodIndex>{MobodIndex(1), MobodIndex(2)}));

  // Remodel making single Mobods for composite links.
  graph.BuildForest(ModelingOptions::CombineLinkComposites);
  forest.SanityCheckForest();
  forest.DumpForest("WorldCompositeComesFirst -- combining");
  EXPECT_EQ(ssize(forest.mobods()), 3);  // Because we're combining.
  EXPECT_EQ(forest.mobods(MobodIndex(0)).follower_links(),
            (std::vector<BodyIndex>{BodyIndex(0), BodyIndex(3)}));
  EXPECT_EQ(forest.mobods(MobodIndex(1)).follower_links(),
            (std::vector<BodyIndex>{BodyIndex(1), BodyIndex(2)}));
  EXPECT_EQ(forest.mobods(MobodIndex(2)).follower_links(),
            (std::vector<BodyIndex>{BodyIndex(4)}));

  EXPECT_EQ(ssize(graph.link_composites()), 2);  // no change expected
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);   // just World now
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
