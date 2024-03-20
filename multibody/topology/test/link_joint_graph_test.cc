/* clang-format off to disable clang-format-includes */
#include "drake/multibody/topology/graph.h"
/* clang-format on */

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

// Tests here are limited to those that can be done without a definition
// for the SpanningForest class which is only forward declared. (We can still
// build a forest here and see its effects on the LinkJointGraph, but we
// can't look into the forest.) See spanning_forest_test.cc for
// tests that require looking at the SpanningForest details.

namespace drake {
namespace multibody {
namespace internal {

using std::pair;

// Built-in joint types are "weld", "quaternion_floating", "rpy_floating".
// We'll register other types in some of the tests below.

// This class is a friend of subclasses that have private constructors and
// member functions so that we can test those APIs standalone.
class LinkJointGraphTester {
 public:
  static LinkJointGraph::Link MakeLink(BodyIndex index, std::string name,
                                       ModelInstanceIndex model_instance,
                                       LinkFlags flags) {
    return LinkJointGraph::Link(index, std::move(name), model_instance, flags);
  }

  static LinkJointGraph::Joint MakeJoint(JointIndex index, std::string name,
                                         ModelInstanceIndex model_instance,
                                         JointTypeIndex joint_type_index,
                                         BodyIndex parent_link_index,
                                         BodyIndex child_link_index,
                                         JointFlags flags) {
    return LinkJointGraph::Joint(index, std::move(name), model_instance,
                                 joint_type_index, parent_link_index,
                                 child_link_index, flags);
  }

  static LinkFlags set_link_flags(LinkFlags to_set,
                                  LinkJointGraph::Link* link) {
    return link->set_flags(to_set);
  }

  static JointFlags set_joint_flags(JointFlags to_set,
                                    LinkJointGraph::Joint* joint) {
    return joint->set_flags(to_set);
  }

  static LoopConstraintIndex AddLoopClosingWeldConstraint(
      BodyIndex primary_link_index, BodyIndex shadow_link_index,
      LinkJointGraph* graph) {
    return graph->AddLoopClosingWeldConstraint(primary_link_index,
                                               shadow_link_index);
  }

  static std::vector<BodyIndex> static_links(const LinkJointGraph& graph) {
    return graph.static_links();
  }

  static std::vector<BodyIndex> non_static_must_be_base_body_links(
      const LinkJointGraph& graph) {
    return graph.non_static_must_be_base_body_links();
  }
};

namespace {

// Verify that the enums in link_joint_graph_defs.h work properly as bitmaps.
GTEST_TEST(LinkJointGraph, FlagsAndOptions) {
  const auto is_static = LinkFlags::kStatic;
  const auto massless = LinkFlags::kTreatAsMassless;

  // Or-ed flags still have LinkFlags type.
  auto link_flags = is_static | massless;
  static_assert(std::is_same_v<decltype(link_flags), LinkFlags>);

  // And is bitwise but still returns LinkFlags, convertible to bool.
  EXPECT_EQ(link_flags & is_static, LinkFlags::kStatic);
  EXPECT_EQ(link_flags & massless, LinkFlags::kTreatAsMassless);
  EXPECT_FALSE(static_cast<bool>(link_flags & LinkFlags::kMustBeBaseBody));
  EXPECT_EQ(link_flags & LinkFlags::kMustBeBaseBody, LinkFlags::kDefault);

  // Repeat for Modeling Options.
  const auto use_fixed_base = ForestBuildingOptions::kUseFixedBase;
  const auto combine_links = ForestBuildingOptions::kCombineLinkComposites;
  auto forest_building_options = use_fixed_base | combine_links;
  static_assert(
      std::is_same_v<decltype(forest_building_options), ForestBuildingOptions>);
  EXPECT_EQ(forest_building_options & use_fixed_base,
            ForestBuildingOptions::kUseFixedBase);
  EXPECT_EQ(forest_building_options & combine_links,
            ForestBuildingOptions::kCombineLinkComposites);
  EXPECT_FALSE(static_cast<bool>(forest_building_options &
                                 ForestBuildingOptions::kUseRpyFloatingJoints));
  EXPECT_EQ(
      forest_building_options & ForestBuildingOptions::kUseRpyFloatingJoints,
      ForestBuildingOptions::kDefault);

  // Only one option in JointFlags currently.
  auto joint_flags = JointFlags::kMustBeModeled;
  static_assert(std::is_same_v<decltype(joint_flags), JointFlags>);
  EXPECT_EQ(joint_flags & JointFlags::kMustBeModeled,
            JointFlags::kMustBeModeled);
  EXPECT_EQ(joint_flags | JointFlags::kMustBeModeled,
            JointFlags::kMustBeModeled);
}

// Verify that forest building options can be set and reset properly. This
// is not checking that they have the intended effects; just that we can
// record them correctly.
GTEST_TEST(LinkJointGraph, SpecifyForestBuildingOptions) {
  LinkJointGraph graph;

  const ForestBuildingOptions default_options = ForestBuildingOptions::kDefault;
  const ForestBuildingOptions two_options =
      ForestBuildingOptions::kCombineLinkComposites |
      ForestBuildingOptions::kUseRpyFloatingJoints;

  // If we haven't said anything, global and all ModelInstance options are
  // just default.
  EXPECT_EQ(graph.get_global_forest_building_options(), default_options);
  EXPECT_EQ(graph.get_forest_building_options_in_use(default_model_instance()),
            default_options);
  EXPECT_EQ(graph.get_forest_building_options_in_use(ModelInstanceIndex(73)),
            default_options);

  // Global options should be used for any unrecognized model instance but
  // should be overridden by instance-specific options.
  graph.SetGlobalForestBuildingOptions(two_options);
  graph.SetForestBuildingOptions(ModelInstanceIndex(9),
                                 ForestBuildingOptions::kStatic);
  EXPECT_EQ(graph.get_global_forest_building_options(), two_options);
  EXPECT_EQ(graph.get_forest_building_options_in_use(ModelInstanceIndex(3)),
            two_options);  // Nothing set for this instance.
  EXPECT_EQ(graph.get_forest_building_options_in_use(ModelInstanceIndex(9)),
            ForestBuildingOptions::kStatic);  // Overridden.

  // Reset should put everything back as it was.
  graph.ResetForestBuildingOptions();
  EXPECT_EQ(graph.get_global_forest_building_options(), default_options);
  EXPECT_EQ(graph.get_forest_building_options_in_use(ModelInstanceIndex(9)),
            default_options);
}

// LinkJointGraph supports copy/move/assign, including the already-built
// SpanningForest. That takes some sleight-of-hand to get back pointers right;
// make sure it works. Note that the actual copying and moving of data members
// is implemented using the corresponding compiler-generated defaults for the
// Data structs in LinkJointGraph and SpanningForest. We can assume those are
// working correctly so don't need to check every member.
GTEST_TEST(LinkJointGraph, CopyMoveAssignTest) {
  LinkJointGraph graph;
  LinkJointGraph* graph_ptr = &graph;  // To avoid warnings below.
  const SpanningForest* original_graph_forest_ptr = &graph.forest();

  // These first checks don't use copy/move/assign but are here to make it
  // clear what we have before we start with those.
  EXPECT_FALSE(graph.forest_is_valid());
  graph.BuildForest();
  EXPECT_TRUE(graph.forest_is_valid());
  graph.AddLink("link1", ModelInstanceIndex(19));  // Should invalidate forest.
  EXPECT_FALSE(graph.forest_is_valid());
  graph.BuildForest();  // Update the forest.

  // Remember the memory address of link1 so we can see if we're re-using
  // the same memory when we expect to be.
  const LinkJointGraph::Link& original_link1 = graph.links()[1];

  // Self copy-assign and self move-assign should be no-ops. It's hard to verify
  // that no self-copying has occurred but we can at least see that we're still
  // using the same memory afterwards.
  graph = *graph_ptr;                  // Self copy-assign.
  EXPECT_EQ(ssize(graph.links()), 2);  // World + link1.
  EXPECT_EQ(&graph.forest(), original_graph_forest_ptr);
  EXPECT_EQ(&graph.links()[1], &original_link1);

  graph = std::move(*graph_ptr);       // Self move-assign.
  EXPECT_EQ(ssize(graph.links()), 2);  // World + link1.
  EXPECT_EQ(&graph.forest(), original_graph_forest_ptr);
  EXPECT_EQ(&graph.links()[1], &original_link1);

  LinkJointGraph graph_copy(graph);  // Copy constructor.
  const SpanningForest* graph_copy_forest_ptr = &graph_copy.forest();
  EXPECT_TRUE(graph_copy.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_TRUE(graph_copy.forest_is_valid());
  EXPECT_NE(graph_copy_forest_ptr, original_graph_forest_ptr);
  EXPECT_NE(&graph_copy.links()[1], &original_link1);

  LinkJointGraph graph_assign;
  graph_assign = graph;  // Copy assignment.
  EXPECT_TRUE(graph_assign.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_TRUE(graph_assign.forest_is_valid());
  EXPECT_NE(&graph_assign.forest(), &graph.forest());
  EXPECT_NE(&graph_assign.links()[1], &original_link1);

  EXPECT_TRUE(graph.forest_is_valid());  // Unchanged by assign-from.
  EXPECT_EQ(ssize(graph.links()), 2);    // World + link1

  LinkJointGraph graph_move(std::move(graph));  // Move constructor.
  EXPECT_EQ(&graph_move.forest(), original_graph_forest_ptr);  // Stolen!
  EXPECT_TRUE(graph_move.forest_is_valid());
  EXPECT_TRUE(graph_move.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_EQ(ssize(graph_move.links()), 2);             // World + link1
  EXPECT_EQ(&graph_move.links()[1], &original_link1);  // Stolen!
  EXPECT_EQ(ssize(graph.links()), 1);                  // Back to default state.

  EXPECT_FALSE(graph.forest_is_valid());  // It was stolen.
  // This should return a new SpanningForest object.
  EXPECT_NE(&graph.forest(), original_graph_forest_ptr);

  LinkJointGraph graph_move_assign;
  const LinkJointGraph::Link& copy_link1 = graph_copy.links()[1];
  graph_move_assign = std::move(graph_copy);  // Move assignent.
  EXPECT_EQ(&graph_move_assign.forest(), graph_copy_forest_ptr);  // Stolen!
  EXPECT_TRUE(graph_move_assign.forest_is_valid());
  EXPECT_TRUE(graph_move_assign.HasLinkNamed("link1", ModelInstanceIndex(19)));
  EXPECT_EQ(ssize(graph_move_assign.links()), 2);         // World + link1
  EXPECT_EQ(&graph_move_assign.links()[1], &copy_link1);  // Stolen!
  EXPECT_EQ(ssize(graph_copy.links()), 1);  // Back to default state.
  // Should have a new forest.
  EXPECT_NE(&graph_copy.forest(), graph_copy_forest_ptr);
}

// A default-constructed LinkJointGraph should contain a predefined World
// Link, predefined joint types, and be suitable for generating a matching
// SpanningForest.
GTEST_TEST(LinkJointGraph, WorldOnlyTest) {
  LinkJointGraph graph;

  // World is predefined.
  EXPECT_EQ(ssize(graph.links()), 1);
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(graph.world_link().name(), "world");
  EXPECT_EQ(graph.world_link().model_instance(), world_model_instance());
  const BodyIndex world_link_index = graph.world_link().index();
  EXPECT_EQ(world_link_index, BodyIndex(0));

  // Topologically important joint types are predefined.
  EXPECT_EQ(ssize(graph.joint_types()), 3);
  const LinkJointGraph::JointType& weld_type =
      graph.joint_types(LinkJointGraph::weld_joint_type_index());
  EXPECT_EQ(weld_type.name, "weld");
  EXPECT_EQ(weld_type.nq, 0);
  EXPECT_EQ(weld_type.nv, 0);
  EXPECT_EQ(weld_type.has_quaternion, false);
  const LinkJointGraph::JointType& quaternion_floating_type =
      graph.joint_types(LinkJointGraph::quaternion_floating_joint_type_index());
  EXPECT_EQ(quaternion_floating_type.name, "quaternion_floating");
  EXPECT_EQ(quaternion_floating_type.nq, 7);
  EXPECT_EQ(quaternion_floating_type.nv, 6);
  EXPECT_EQ(quaternion_floating_type.has_quaternion, true);
  const LinkJointGraph::JointType& rpy_floating_type =
      graph.joint_types(LinkJointGraph::rpy_floating_joint_type_index());
  EXPECT_EQ(rpy_floating_type.name, "rpy_floating");
  EXPECT_EQ(rpy_floating_type.nq, 6);
  EXPECT_EQ(rpy_floating_type.nv, 6);
  EXPECT_EQ(rpy_floating_type.has_quaternion, false);

  EXPECT_FALSE(graph.forest_is_valid());

  // With no forest built, there are no composites.
  EXPECT_TRUE(graph.link_composites().empty());

  graph.BuildForest();
  const SpanningForest& forest = graph.forest();
  EXPECT_TRUE(graph.forest_is_valid());

  // With the forest built, we can access more information about the graph.
  // "Find" and "Get" methods require that the forest is valid.
  EXPECT_TRUE(graph.world_link().is_anchored());
  EXPECT_EQ(graph.link_to_mobod(world_link_index), MobodIndex(0));
  EXPECT_EQ(ssize(graph.link_composites()), 1);
  EXPECT_EQ(ssize(graph.link_composites(LinkCompositeIndex(0))), 1);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[0], world_link_index);

  // Check that Clear() puts the graph back to default-constructed condition.
  // First add some junk to the graph.
  graph.RegisterJointType("revolute", 1, 1);
  const BodyIndex dummy_index =
      graph.AddLink("dummy", default_model_instance());
  graph.AddJoint("joint0", default_model_instance(), "revolute", world_index(),
                 dummy_index);
  EXPECT_TRUE(graph.HasLinkNamed("dummy", default_model_instance()));
  EXPECT_EQ(dummy_index, BodyIndex(1));
  EXPECT_EQ(ssize(graph.links()), 2);
  EXPECT_EQ(ssize(graph.joints()), 1);
  EXPECT_EQ(ssize(graph.joint_types()), 4);
  // Now get rid of that junk.
  graph.Clear();
  EXPECT_EQ(ssize(graph.links()), 1);  // World
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(ssize(graph.joint_types()), 3);  // Predefineds
  EXPECT_FALSE(graph.forest_is_valid());

  // Make sure Clear() saved the existing forest.
  const SpanningForest& same_forest = graph.forest();
  EXPECT_EQ(&same_forest, &forest);
  EXPECT_FALSE(graph.forest_is_valid());
  graph.BuildForest();  // OK to build even with just World in graph.
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
                    LinkFlags::kTreatAsMassless | LinkFlags::kMustBeBaseBody);
  const LinkJointGraph::Link& link2 = graph.links(link2_index);
  EXPECT_TRUE(link2.treat_as_massless() && link2.must_be_base_body());
  EXPECT_FALSE(link2.is_static() || link2.is_shadow());

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("link3", ModelInstanceIndex(3), LinkFlags::kShadow),
      "AddLink.*Can't add.*link3.*Shadow flag.*");
}

// LinkJointGraph collects and organizes some input information to facilitate
// efficent forest building. Make sure that's done correctly.
GTEST_TEST(LinkJointGraph, InternalListsAreBuiltCorrectly) {
  LinkJointGraph graph;

  const BodyIndex link1_index =
      graph.AddLink("link1", default_model_instance(), LinkFlags::kStatic);
  const BodyIndex link2_index = graph.AddLink("link2", default_model_instance(),
                                              LinkFlags::kMustBeBaseBody);
  const BodyIndex link3_index =
      graph.AddLink("link3", default_model_instance(),
                    LinkFlags::kStatic | LinkFlags::kMustBeBaseBody);

  // Links 1 and 3 are static.
  EXPECT_EQ(LinkJointGraphTester::static_links(graph),
            (std::vector<BodyIndex>{link1_index, link3_index}));
  // But only link 2 should be on the non-static must be base body list.
  EXPECT_EQ(LinkJointGraphTester::non_static_must_be_base_body_links(graph),
            std::vector<BodyIndex>{link2_index});
}

// Check operation of the public members of the Link subclass.
GTEST_TEST(LinkJoinGraph, LinkAPITest) {
  LinkJointGraph::Link link5 = LinkJointGraphTester::MakeLink(
      BodyIndex(5), "link5", ModelInstanceIndex(7), LinkFlags::kMustBeBaseBody);
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
  LinkJointGraphTester::set_link_flags(LinkFlags::kStatic, &link5);
  EXPECT_TRUE(link5.is_static());
  EXPECT_TRUE(link5.is_anchored());  // Static links are anchored to World.
  EXPECT_TRUE(link5.must_be_base_body());   // Unchanged.
  EXPECT_FALSE(link5.treat_as_massless());  // Unchanged.

  // Only LinkJointGraph sets these; no public interface.
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
GTEST_TEST(LinkJointGraph, JointAPITest) {
  const BodyIndex parent_index(1);
  const BodyIndex child_index(2);
  const BodyIndex other_body_index(3);  // Not connected by joint3.
  LinkJointGraph::Joint joint3 = LinkJointGraphTester::MakeJoint(
      JointIndex(3), "joint3", ModelInstanceIndex(9),
      LinkJointGraph::rpy_floating_joint_type_index(), parent_index,
      child_index, JointFlags::kMustBeModeled);
  EXPECT_EQ(joint3.index(), JointIndex(3));
  EXPECT_EQ(joint3.model_instance(), ModelInstanceIndex(9));
  EXPECT_EQ(joint3.name(), "joint3");
  EXPECT_EQ(joint3.parent_link(), parent_index);
  EXPECT_EQ(joint3.child_link(), child_index);
  EXPECT_FALSE(joint3.is_weld());
  EXPECT_EQ(joint3.type_index(),
            LinkJointGraph::rpy_floating_joint_type_index());
  EXPECT_TRUE(joint3.connects(parent_index));
  EXPECT_TRUE(joint3.connects(child_index));
  EXPECT_FALSE(joint3.connects(other_body_index));
  EXPECT_TRUE(joint3.connects(parent_index, child_index));
  EXPECT_TRUE(joint3.connects(child_index, parent_index));
  EXPECT_FALSE(joint3.connects(parent_index, other_body_index));
  EXPECT_TRUE(joint3.must_be_modeled());
  EXPECT_FALSE(joint3.mobod_index().is_valid());
  EXPECT_FALSE(joint3.has_been_processed());
  EXPECT_EQ(joint3.other_link_index(parent_index), child_index);
  EXPECT_EQ(joint3.other_link_index(child_index), parent_index);
  LinkJointGraphTester::set_joint_flags(JointFlags::kDefault, &joint3);
  EXPECT_FALSE(joint3.must_be_modeled());
}

// Verify that we can define a serial chain, some static and floating links,
// and some simple composites, and correctly reject improper attempts. We're
// mostly testing the LinkJointGraph API here; see spanning_forest_test.cc
// for validation of the generated forest for a similar graph.
GTEST_TEST(LinkJointGraph, SerialChainAndMore) {
  LinkJointGraph graph;

  const JointTypeIndex revolute_index =
      graph.RegisterJointType("revolute", 1, 1);
  EXPECT_EQ(ssize(graph.joint_types()), 4);  // built-in types plus "revolute"

  EXPECT_FALSE(graph.IsJointTypeRegistered("prismatic"));
  EXPECT_THROW(graph.joint_types(JointTypeIndex(99)), std::exception);

  // Verify that the revolute joint was correctly registered.
  EXPECT_TRUE(graph.IsJointTypeRegistered("revolute"));
  const LinkJointGraph::JointType& revolute_joint_type =
      graph.joint_types(revolute_index);
  EXPECT_EQ(revolute_joint_type.name, "revolute");
  EXPECT_EQ(revolute_joint_type.nq, 1);
  EXPECT_EQ(revolute_joint_type.nv, 1);
  EXPECT_FALSE(revolute_joint_type.has_quaternion);

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
  graph.AddJoint("pin1", model_instance, "revolute", world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    BodyIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, "revolute",
                   parent, child);
    parent = child;
  }

  // Later these will be used to distinguish user elements from ephemeral ones.
  // For now just make sure they are getting updated properly.
  EXPECT_EQ(graph.num_user_links(), 6);  // Includes World.
  EXPECT_EQ(graph.num_user_joints(), 5);

  // Check that the links see their joints.
  const LinkJointGraph::Link& link4 = graph.links(BodyIndex(4));
  EXPECT_EQ(link4.joints(),
            (std::vector<JointIndex>{JointIndex(3), JointIndex(4)}));
  EXPECT_EQ(link4.joints_as_parent(), std::vector<JointIndex>{JointIndex(4)});
  EXPECT_EQ(link4.joints_as_child(), std::vector<JointIndex>{JointIndex(3)});

  // Just for testing, we'll tell the graph that we added a loop-closing
  // weld constraint, and check that it properly updates the relevant links.
  // (This is a private function normally used only by SpanningForest as it
  // breaks loops; users can't add constraints to the graph.)
  const LinkJointGraph::Link& link5 = graph.links(BodyIndex(5));
  const LoopConstraintIndex constraint0_index =
      LinkJointGraphTester::AddLoopClosingWeldConstraint(
          link4.index(),  // primary link
          link5.index(),  // shadow link
          &graph);
  EXPECT_EQ(constraint0_index, LoopConstraintIndex(0));
  EXPECT_EQ(ssize(graph.loop_constraints()), 1);
  const LinkJointGraph::LoopConstraint& constraint0 =
      graph.loop_constraints(constraint0_index);
  EXPECT_EQ(constraint0.index(), constraint0_index);
  EXPECT_EQ(constraint0.name(), link5.name());  // Inherits the shadow's name.
  EXPECT_EQ(constraint0.model_instance(), model_instance);
  EXPECT_EQ(constraint0.primary_link(), link4.index());
  EXPECT_EQ(constraint0.shadow_link(), link5.index());
  // See if the involved links agree with the graph.
  EXPECT_EQ(link4.loop_constraints(),
            std::vector<LoopConstraintIndex>{constraint0_index});
  EXPECT_EQ(link5.loop_constraints(),
            std::vector<LoopConstraintIndex>{constraint0_index});

  // Out of range should throw.
  EXPECT_THROW(graph.loop_constraints(LoopConstraintIndex(99)), std::exception);

  // We cannot duplicate the name of a Link or Joint.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("link3", model_instance),
      "AddLink.*already a link named.*link3.*model instance.*5.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("pin3", model_instance, "revolute", BodyIndex(1),
                     BodyIndex(2)),
      "AddJoint.*already a joint named.*pin3.*model instance.*5.*");

  // We cannot add a redundant joint.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("other", model_instance, "revolute", BodyIndex(1),
                     BodyIndex(2)),
      "AddJoint\\(\\):.*already has joint.*pin2.*instance 5.*"
      " connecting.*link1.*link2.*adding joint.*other.*instance 5.*"
      " connecting.*link1.*link2.*is not allowed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("reverse", model_instance, "revolute", BodyIndex(2),
                     BodyIndex(1)),
      "AddJoint\\(\\):.*already has joint.*pin2.*instance 5.*"
      " connecting.*link1.*link2.*adding joint.*reverse.*instance 5.*"
      " connecting.*link2.*link1.*is not allowed.");

  // We cannot add an unregistered joint type.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddJoint("screw1", model_instance, "screw",
                                             BodyIndex(1), BodyIndex(2)),
                              "AddJoint\\(\\): Unrecognized type.*");

  // Invalid parent/child Link throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, "revolute", BodyIndex(1),
                     BodyIndex(9)),
      "AddJoint\\(\\): child link index 9 for joint '.*' is out of range.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, "revolute", BodyIndex(9),
                     BodyIndex(1)),
      "AddJoint\\(\\): parent link index 9 for joint '.*' is out of range.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("joint_to_self", model_instance, "revolute", BodyIndex(5),
                     BodyIndex(5)),
      "AddJoint\\(\\): Joint.*joint_to_self.*instance 5.*would connect.*link5.*"
      "to itself.");

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
      graph.AddLink("static8", model_instance, LinkFlags::kStatic);
  graph.AddJoint("static7_weld", model_instance,  // OK
                 "weld", graph.world_link().index(), static7_index);
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("static8_pin", model_instance, "revolute",
                     graph.world_link().index(), static8_index),
      "AddJoint\\(\\): can't connect.*'static8' to World.*revolute.*"
      "only a weld.*");

  // Now add a free link and a free-floating pair.
  graph.AddLink("free9", model_instance);

  const BodyIndex link10_index = graph.AddLink("link10", model_instance);
  const BodyIndex base11_index =
      graph.AddLink("base11", model_instance, LinkFlags::kMustBeBaseBody);
  const JointIndex joint_10_11_index = graph.AddJoint(
      "weld", model_instance, "weld", link10_index, base11_index);

  EXPECT_EQ(graph.MaybeGetJointBetween(link10_index, base11_index),
            joint_10_11_index);
  EXPECT_EQ(graph.MaybeGetJointBetween(base11_index, link10_index),
            joint_10_11_index);
  EXPECT_FALSE(
      graph.MaybeGetJointBetween(world_index(), link10_index).is_valid());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
