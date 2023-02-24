#include "drake/multibody/topology/link_joint_graph.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/topology/spanning_forest_model.h"

namespace drake {
namespace multibody {
namespace internal {

using std::pair;

const char kRevoluteType[] = "revolute";
const char kPrismaticType[] = "prismatic";

// Arbitrary world name for testing.
const char kWorldLinkName[] = "DefaultWorldLinkName";

// Test a straightforward serial chain of bodies connected by
// revolute joints: world->body1->body2->body3->body4->body5.
// We perform a number of sanity checks on the provided API.
GTEST_TEST(LinkJointGraph, SerialChain) {
  LinkJointGraph graph;
  EXPECT_EQ(graph.num_joint_types(), 3);  // predefined types thus far
  graph.RegisterJointType(kRevoluteType, 1, 1);
  EXPECT_EQ(graph.num_joint_types(), 4);  // plus "revolute"

  // Verify what joint types were registered.
  EXPECT_TRUE(graph.IsJointTypeRegistered(kRevoluteType));
  EXPECT_FALSE(graph.IsJointTypeRegistered(kPrismaticType));

  // The first Link added defines the world's name and model instance.
  // We'll verify their values below.
  graph.AddLink(kWorldLinkName, world_model_instance());

  // We'll add the chain to this model.
  const ModelInstanceIndex model_instance(5);

  // Put static bodies in this model instance.
  const ModelInstanceIndex static_model_instance(100);

  // We cannot register to the world model instance, unless it's the first call
  // to AddLink().
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("InvalidLink", world_model_instance()),
      fmt::format("AddLink\\(\\): Model instance index = {}.*",
                  world_model_instance()));

  LinkIndex parent = graph.AddLink("link1", model_instance);
  graph.AddJoint("pin1", model_instance, kRevoluteType, world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    LinkIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, kRevoluteType,
                   parent, child);
    parent = child;
  }

  // We cannot duplicate the name of a Link or Joint.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddLink("link3", model_instance),
                              "AddLink\\(\\): Duplicate link name.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("pin3", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(2)),
      "AddJoint\\(\\): Duplicate joint name.*");

  // We cannot add a redundant joint.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("other", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(2)),
      "AddJoint\\(\\): This LinkJointGraph already has a joint 'pin2'"
      " connecting 'link1' to 'link2'. Therefore adding joint 'other'"
      " connecting 'link1' to 'link2' is not allowed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("reverse", model_instance, kRevoluteType, LinkIndex(2),
                     LinkIndex(1)),
      "AddJoint\\(\\): This LinkJointGraph already has a joint 'pin2'"
      " connecting 'link1' to 'link2'. Therefore adding joint 'reverse'"
      " connecting 'link2' to 'link1' is not allowed.");

  // We cannot add an unregistered joint type.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddJoint("screw1", model_instance, "screw",
                                             LinkIndex(1), LinkIndex(2)),
                              "AddJoint\\(\\): Unrecognized type.*");

  // Invalid parent/child Link throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(9)),
      "AddJoint\\(\\): child link index 9 for joint '.*' is out of range.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, LinkIndex(9),
                     LinkIndex(1)),
      "AddJoint\\(\\): parent link index 9 for joint '.*' is out of range.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("joint_to_self", model_instance, kRevoluteType,
                     LinkIndex(5), LinkIndex(5)),
       "AddJoint\\(\\): Can't add a joint from link 'link5' to itself.");

  // Verify the world's name and model instance are registered correctly.
  EXPECT_EQ(graph.world_link().name(), kWorldLinkName);
  EXPECT_EQ(graph.world_link().model_instance(), world_model_instance());
  // Link "0" is always the world.
  EXPECT_EQ(graph.links(LinkIndex(0)).name(), kWorldLinkName);
  EXPECT_EQ(graph.links(LinkIndex(0)).model_instance(),
            world_model_instance());

  // Sanity check sizes.
  EXPECT_EQ(ssize(graph.links()), 6);  // This includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 5);

  // Verify we can get bodies/joints.
  EXPECT_EQ(graph.links(LinkIndex(3)).name(), "link3");
  EXPECT_EQ(graph.joints(JointIndex(3)).name(), "pin4");
  EXPECT_THROW(graph.links(LinkIndex(9)), std::exception);
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
  const LinkIndex static7_index =
      graph.AddLink("static7", static_model_instance);
  const LinkIndex static8_index =
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

  const LinkIndex link10_index = graph.AddLink("link10", model_instance);
  const LinkIndex base11_index =
      graph.AddLink("base11", model_instance, LinkFlags::MustBeBaseBody);
  const JointIndex joint_10_11_index =
      graph.AddJoint("weld", model_instance, graph.weld_type_name(),
                     link10_index, base11_index);

  /* The LinkJointGraph looks like this:

  world->link1->link2->link3->link4->link5
      static6         (no joint, static model instance)
      ->static7       (weld provided, static model instance)
      static8         (no joint, static link)
      free9           (no joint)
      link10->base11* (these are welded together)

    * link10 would be the preferred base link but link11 is marked
      "must be base link" so we have to use a reversed mobilizer there

  The SpanningForestModel should get built with 6 trees of Mobods in this order:
   (legend: -> given joint, => added joint, [mobods], {composite links})

   tree      world [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6]    ->static7
     2 [7]    =>static6        (added weld)
     3 [8]    =>static8        (added weld)
     4 [9-10] =>base11->link10 (added 6dof, reversed the weld)
     5 [11]   =>free9          (added 6dof, free bodies are always last)

   Composite Links:  {0 7 6 8} {11 10}
   Composite Mobods: [0 6 7 8] [9 10]

   The particular ordering results from (a) user-supplied Joints get processed
   before added ones, and (b) static model instance Links get welded prior
   to individually-specified static Links in non-static model instances.

   If instead we ask to combined welded Links we should get

    tree      {world static7 static6 static8} [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6]    =>{base11->link10} (added 6dof, unmodeled weld)
     2 [7]    =>free9            (added 6dof, free bodies are always last)

   Composite Links: {0 7 6 8} {11 10}
   Composite Mobods: none
   */

  // TODO(sherm1) Move to its own test suite.
  const SpanningForestModel& model =
      graph.BuildModel(ModelingOptions::Default,
                       {{static_model_instance, ModelingOptions::Static}});
  graph.DumpGraph("SerialChain (not combined)");
  model.SanityCheckForest();
  model.DumpModel("SerialChain (not combined)");

  EXPECT_EQ(ssize(graph.joints()) - graph.num_user_joints(), 4);
  const std::vector<LinkIndex> composite_links0{LinkIndex(0), LinkIndex(7),
                                                LinkIndex(6), LinkIndex(8)};
  const std::vector<LinkIndex> composite_links1{LinkIndex(11), LinkIndex(10)};
  const std::vector expected_composite_links{composite_links0,
                                             composite_links1};
  EXPECT_EQ(graph.composite_links(), expected_composite_links);

  EXPECT_EQ(ssize(model.mobods()), 12);
  EXPECT_EQ(ssize(model.trees()), 6);
  EXPECT_EQ(model.num_positions(), 19);
  EXPECT_EQ(model.num_velocities(), 17);
  const std::vector<MobodIndex> composite_mobods0{MobodIndex(0), MobodIndex(6),
                                                  MobodIndex(7), MobodIndex(8)};
  const std::vector<MobodIndex> composite_mobods1{MobodIndex(9),
                                                  MobodIndex(10)};
  const std::vector expected{composite_mobods0, composite_mobods1};
  EXPECT_EQ(model.composite_mobods(), expected);

  // Test FindNumOutboardVelocities() for the two fast cases and generic.
  auto find_outv = [&model](int mobod_index) -> pair<int, int> {
    return model.FindOutboardVelocities(MobodIndex(mobod_index));
  };
  EXPECT_EQ(find_outv(0), pair(0, 17));  // World
  EXPECT_TRUE(model.mobods(MobodIndex(1)).is_base_body());  // Base bodies
  EXPECT_TRUE(model.mobods(MobodIndex(7)).is_base_body());
  EXPECT_TRUE(model.mobods(MobodIndex(9)).is_base_body());
  EXPECT_TRUE(model.mobods(MobodIndex(11)).is_base_body());
  EXPECT_EQ(find_outv(1), pair(1, 4));
  EXPECT_EQ(find_outv(6), pair(5, 0));
  EXPECT_EQ(find_outv(9), pair(11, 0));
  EXPECT_EQ(find_outv(11), pair(17, 0));
  EXPECT_FALSE(model.mobods(MobodIndex(3)).is_base_body());  // Generic case
  EXPECT_EQ(find_outv(3), pair(3, 2));

  graph.BuildModel(ModelingOptions::CombineCompositeLinks,
                   {{model_instance, ModelingOptions::CombineCompositeLinks},
                    {static_model_instance, ModelingOptions::Static}});
  graph.DumpGraph("SerialChain (combined)");
  model.SanityCheckForest();
  model.DumpModel("SerialChain (combined)");

  // The graph shouldn't have changed, but the forest model will.
  EXPECT_EQ(ssize(graph.joints()) - graph.num_user_joints(), 4);
  EXPECT_EQ(graph.composite_links(), expected_composite_links);

  EXPECT_EQ(ssize(model.mobods()), 8);
  EXPECT_EQ(ssize(model.trees()), 3);
  EXPECT_TRUE(model.composite_mobods().empty());

  // Now force one of the joints in the composite to be modeled (meaning it
  // should get its own Mobod). This should split the World composite into
  // two Mobods, which should be noted as composite Mobods.
  graph.change_joint_flags(joint_10_11_index, JointFlags::MustBeModeled);
  // Model with same options as above.
  graph.BuildModel(ModelingOptions::CombineCompositeLinks,
                   {{model_instance,
                     ModelingOptions::CombineCompositeLinks},
                    {static_model_instance, ModelingOptions::Static}});
  model.SanityCheckForest();
  model.DumpModel("SerialChain (combined, split composite 10,11)");

  EXPECT_EQ(ssize(model.mobods()), 9);
  EXPECT_EQ(ssize(model.trees()), 3);
  EXPECT_EQ(ssize(model.composite_mobods()), 1);
  const std::vector<MobodIndex> now_expected{MobodIndex(6), MobodIndex(7)};
  EXPECT_EQ(model.composite_mobods()[0], now_expected);

  /* Build again but with the FixedBase option for model_instance, and allow
  joint_10_11 to be part of a composite (default behavior). Now we expect

  World: {world static7 static6 static8 base11 link10 free9} [0]
  tree
    0  [1-5]  ->link1->link2->link3->link4->link5

  Composite Links: {0 7 6 8 11 10 9}
  Composite Mobods: none
  */
  graph.change_joint_flags(joint_10_11_index, JointFlags::Default);
  graph.BuildModel(ModelingOptions::CombineCompositeLinks,
                   {{model_instance, ModelingOptions::CombineCompositeLinks |
                                         ModelingOptions::UseFixedBase},
                    {static_model_instance, ModelingOptions::Static}});
  graph.DumpGraph("SerialChain (combined, split composite 10,11, fixed base)");
  model.SanityCheckForest();
  model.DumpModel("SerialChain (combined, split composite 10,11, fixed base)");

  EXPECT_EQ(ssize(model.mobods()), 6);
  EXPECT_EQ(ssize(model.trees()), 1);
  EXPECT_EQ(ssize(model.composite_mobods()), 0);
  const std::vector<LinkIndex> expected_composite_link{
    LinkIndex(0), LinkIndex(7), LinkIndex(6), LinkIndex(8),
    LinkIndex(11), LinkIndex(10), LinkIndex(9)};
  EXPECT_EQ(ssize(graph.composite_links()), 1);
  EXPECT_EQ(graph.composite_links()[0], expected_composite_link);
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

  // The first Link added defines the world's name and model instance.
  graph.AddLink("world", world_model_instance());

  // Define the model.
  for (int i = 1; i <= 14; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  // Check against left-hand diagram above.
  const std::vector<std::pair<int, int>> joints
      {{1, 4}, {2, 3}, {3, 8}, {3, 7}, {4, 5}, {4, 6}, {6, 9},
       {6, 10}, {8, 11}, {11, 12}, {11, 13}, {13, 14}};
  for (int i = 0; i < ssize(joints); ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance,
                   kRevoluteType, LinkIndex(joints[i].first),
                   LinkIndex(joints[i].second));
  }

  const SpanningForestModel& model = graph.BuildModel();  // Default options
  graph.DumpGraph("MultipleBranches");
  model.DumpModel("MultipleBranches");

  EXPECT_EQ(model.options(), ModelingOptions::Default);
  EXPECT_EQ(model.options(model_instance), ModelingOptions::Default);
  EXPECT_EQ(&model.graph(), &graph);  // check backpointer
  EXPECT_EQ(&model.links(), &graph.links());
  EXPECT_EQ(&model.joints(), &graph.joints());

  EXPECT_EQ(graph.num_user_links(), 15);  // includes World
  EXPECT_EQ(ssize(graph.links()), 15);  // no links added
  EXPECT_EQ(graph.num_user_joints(), 12);  // the ones added above
  EXPECT_EQ(ssize(graph.joints()), 14);  // modeling adds two floating joints
  EXPECT_TRUE(graph.composite_links().empty());

  EXPECT_EQ(ssize(model.trees()), 2);
  EXPECT_EQ(ssize(model.mobods()), 15);  // includes World
  EXPECT_TRUE(model.composite_mobods().empty());
  EXPECT_EQ(model.num_positions(), 26);  // 12 revolute, 2 x 17 quat floating
  EXPECT_EQ(model.num_velocities(), 24);

  // Check Link->Mobod and Mobod->Link mappings.
  const std::vector<pair<int, int>> link_mobod_map
      {{0, 0}, {1, 1}, {2, 7}, {3, 8}, {4, 2}, {5, 3}, {6, 4}, {7, 14}, {8, 9},
       {9, 5}, {10, 6}, {11, 10}, {12, 11}, {13, 12}, {14, 13}};
  for (auto link_mobod : link_mobod_map) {
    EXPECT_EQ(graph.link_to_mobod(LinkIndex(link_mobod.first)),
              MobodIndex(link_mobod.second));
    EXPECT_EQ(model.mobod_to_link(MobodIndex(link_mobod.second)),
              LinkIndex(link_mobod.first));
    EXPECT_EQ(ssize(model.mobod_to_links(MobodIndex(link_mobod.second))), 1);
    EXPECT_EQ(model.mobod_to_links(MobodIndex(link_mobod.second))[0],
              LinkIndex(link_mobod.first));
  }

  const SpanningForestModel::Tree& tree0 = model.trees(TreeIndex(0));
  const SpanningForestModel::Tree& tree1 = model.trees(TreeIndex(1));
  EXPECT_EQ(tree0.index(), TreeIndex(0));
  EXPECT_EQ(tree1.index(), TreeIndex(1));
  EXPECT_EQ(tree0.height(), 4); EXPECT_EQ(tree1.height(), 6);
  EXPECT_EQ(tree0.base_mobod(), MobodIndex(1));
  EXPECT_EQ(tree0.last_mobod(), MobodIndex(6));
  EXPECT_EQ(tree1.base_mobod(), MobodIndex(7));
  EXPECT_EQ(tree1.last_mobod(), MobodIndex(14));
  EXPECT_EQ(tree0.num_mobods(), 6); EXPECT_EQ(tree1.num_mobods(), 8);
  EXPECT_EQ(tree0.nq(), 12); EXPECT_EQ(tree0.nv(), 11);
  EXPECT_EQ(tree1.nq(), 14); EXPECT_EQ(tree1.nv(), 13);
  EXPECT_EQ(&tree0.front(), &model.mobods(MobodIndex(1)));
  EXPECT_EQ(&tree0.back(), &model.mobods(MobodIndex(6)));
  EXPECT_EQ(&tree1.front(), &model.mobods(MobodIndex(7)));
  EXPECT_EQ(&tree1.back(), &model.mobods(MobodIndex(14)));
  EXPECT_EQ(tree0.begin(), &tree0.front());
  EXPECT_EQ(tree0.end(), &tree0.back() + 1);
  EXPECT_EQ(tree1.begin(), &tree1.front());
  EXPECT_EQ(tree1.end(), &tree1.back() + 1);
  EXPECT_EQ(tree0.q_start(), 0);  EXPECT_EQ(tree0.nq(), 12);
  EXPECT_EQ(tree1.q_start(), 12); EXPECT_EQ(tree1.nq(), 14);
  EXPECT_EQ(tree0.v_start(), 0);  EXPECT_EQ(tree0.nv(), 11);
  EXPECT_EQ(tree1.v_start(), 11); EXPECT_EQ(tree1.nv(), 13);

  // Sample some q's and v's to see if they can find their tree and mobod.
  EXPECT_EQ(model.q_to_tree(9), TreeIndex(0));
  EXPECT_EQ(model.q_to_tree(19), TreeIndex(1));
  EXPECT_EQ(model.v_to_tree(9), TreeIndex(0));
  EXPECT_EQ(model.v_to_tree(19), TreeIndex(1));
  EXPECT_EQ(model.q_to_mobod(9), MobodIndex(4));
  EXPECT_EQ(model.q_to_mobod(20), MobodIndex(9));
  EXPECT_EQ(model.v_to_mobod(9), MobodIndex(5));
  EXPECT_EQ(model.v_to_mobod(20), MobodIndex(11));

  // The FindSubtreeVelocities() algorithm depends on depth-first ordering
  // so that it only has to examine the rightmost branch.
  auto find_v = [&model](int mobod_index) -> pair<int, int> {
    return model.FindSubtreeVelocities(MobodIndex(mobod_index));
  };
  EXPECT_EQ(find_v(0), pair(0, 24));   // World
  EXPECT_EQ(find_v(1), pair(0, 11));   // base tree0
  EXPECT_EQ(find_v(7), pair(11, 13));  // base tree1
  EXPECT_EQ(find_v(3), pair(7, 1));    // terminal
  EXPECT_EQ(find_v(4), pair(8, 3));    // nonterminal
  EXPECT_EQ(find_v(8), pair(17, 7));   // tree1 nonterminal
  EXPECT_EQ(find_v(10), pair(19, 4));
  EXPECT_EQ(find_v(14), pair(23, 1));  // tree1 terminal

  // FindOutboardVelocities() uses FindSubtreeVelocities() but removes
  // the dofs associated with the given body.
  auto find_outv = [&model](int mobod_index) -> pair<int, int> {
    return model.FindOutboardVelocities(MobodIndex(mobod_index));
  };
  EXPECT_EQ(find_outv(0), pair(0, 24));   // same Mobods as above
  EXPECT_EQ(find_outv(1), pair(6, 5));
  EXPECT_EQ(find_outv(7), pair(17, 7));
  EXPECT_EQ(find_outv(3), pair(8, 0));
  EXPECT_EQ(find_outv(4), pair(9, 2));
  EXPECT_EQ(find_outv(8), pair(18, 6));
  EXPECT_EQ(find_outv(10), pair(20, 3));
  EXPECT_EQ(find_outv(14), pair(24, 0));

  const std::vector<MobodIndex> expected_path_from_14{
      {MobodIndex(14)}, {MobodIndex{8}}, {MobodIndex(7)}, {MobodIndex(0)}};
  EXPECT_EQ(model.FindPathToWorld(MobodIndex(14)), expected_path_from_14);

  // Mobods on different trees have only World as a common ancestor.
  EXPECT_EQ(model.FindFirstCommonAncestor(MobodIndex(5), MobodIndex(12)),
            MobodIndex(0));
  EXPECT_EQ(model.FindFirstCommonAncestor(MobodIndex(13), MobodIndex(14)),
            MobodIndex(8));  // See right hand drawing above.
}

/* We build a model containing a number of kinematic loops and subgraphs of
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

In the picture double bars are welds, single moving joints, braces are links:

                                  {9}
  A       {4}                    /15*                {11}    * = added joint
       /1/   \2\                '                  /10   \8
      {1}==0=={13}-3-{3} -14*- {0}==12=={5}==11=={7}--9--{2}
       \7                     World     \13\
       {10}                          C   {12}
     /4/   \5\   B
     {6}=6={8}

Therefore we expect the following Composites, with the "World" Composite first:
  {0, 5, 7, 12}, {13, 1, 4}, {6, 8, 10}, {3}, {9}, {2}, {11}.

Note that the all-weld loops 13-1-4 and 10-6-8 are harmless since the closing
weld can just be ignored. The loop 7-2-11 however requires breaking; we're
currently just ignoring the loop-closing joint 8.

Forest building should start with Link {5} since that is the only direct
connection to World in the input ({3} and {9} get connected later). If we're
giving every Link its own mobilizers (rather than making composites from
welded-together ones) we expect this forest of 3 trees and 14 Mobods:

      level 5                  10{6}  11{8}
      level 4   4{11}             9{10}
      level 3    3{2}              8{1}   12{4}
      level 2      2{7}   5{12}       7{13}
  base mobods          1{5}           6{3}            13{9}
                          \            |               /
        World              ...........0{0}.............

Some of the Links are welded together. We call those Composite Links even
though each has its own Mobod. Those are: {0 5 7 12} {13 1 4} {10 6 8}
The corresponding Mobods are Composite Mobods: [0 1 2 5] [7 8 12] [9 10 11]

That should immediately create composite {0 5 7 12} on
mobod 0, then see outboard links {2} and {11} as new base bodies and grow
those two trees, discovering a loop at joint 8. Then it should choose link [3]
as a base link and add floating joint 14, and grow that tree. Finally it makes
free link {9} a base body. The forest should then look like this:


      level 3                         5{10 6 8}
      level 2                         4{13 1 4}
  base mobods       1{2}    2{11}     3{3}        6{9}  (four trees)
                      \       \        |           /
        World          ...........0{0 5 7 12}......

In this case the Composite Links are the same, but there are no Composite
Mobods (except World alone).
*/
GTEST_TEST(LinkJointGraph, Weldedsubgraphs) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);
  EXPECT_EQ(graph.num_joint_types(), 5);  // predefined + revolute & prismatic.

  // The first Link added defines the world's name and model instance.
  graph.AddLink(kWorldLinkName, world_model_instance());

  // We'll add Links and Joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the model.
  for (int i = 1; i <= 13; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  // Add joints.
  int j = 0;

  // subgraph A: formed by bodies 1, 4, 13.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(1), LinkIndex(13));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(4), LinkIndex(1));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(13), LinkIndex(4));

  // Link 3 connects to subgraph A via a revolute joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 LinkIndex(3), LinkIndex(13));

  // subgraph B: formed by bodies 8, 6, 10.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(10), LinkIndex(6));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(10), LinkIndex(8));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(6), LinkIndex(8));

  // Link 1 (in subgraph A) and Link 10 (in subgraph B) connect through a
  // prismatic joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(1), LinkIndex(10));

  // Closed kinematic loop of non-weld joints.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(2), LinkIndex(11));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(7), LinkIndex(2));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 LinkIndex(7), LinkIndex(11));

  // subgraph C: formed by links 5, 7, 12 and World 0.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(5), LinkIndex(7));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(5), world_index());
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(12), LinkIndex(5));

  EXPECT_EQ(ssize(graph.links()), 14);  // this includes the world Link.

  const std::vector<std::set<LinkIndex>> welded_subgraphs =
      graph.FindSubgraphsOfWeldedLinks();

  // Verify number of expected subgraphs.
  EXPECT_EQ(welded_subgraphs.size(), 7);

  // The first subgraph must contain the world.
  const std::set<LinkIndex> world_subgraph = welded_subgraphs[0];
  EXPECT_EQ(world_subgraph.count(world_index()), 1);

  // Build the expected set of subgraphs.
  std::set<std::set<LinkIndex>> expected_subgraphs;
  //   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
  const std::set<LinkIndex>& expected_world_subgraph =
      *expected_subgraphs
           .insert({LinkIndex(0), LinkIndex(5), LinkIndex(7), LinkIndex(12)})
           .first;
  const std::set<LinkIndex>& expected_subgraphA =
      *expected_subgraphs.insert({LinkIndex(1), LinkIndex(4), LinkIndex(13)})
           .first;
  const std::set<LinkIndex>& expected_subgraphB =
      *expected_subgraphs.insert({LinkIndex(6), LinkIndex(8), LinkIndex(10)})
           .first;
  expected_subgraphs.insert({LinkIndex(3)});
  expected_subgraphs.insert({LinkIndex(9)});
  expected_subgraphs.insert({LinkIndex(2)});
  expected_subgraphs.insert({LinkIndex(11)});

  // We do expect the first subgraph to correspond to the set of bodies welded
  // to the world.
  EXPECT_EQ(world_subgraph, expected_world_subgraph);

  // In order to compare the computed list of welded bodies against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed subgraphs to a set.
  const std::set<std::set<LinkIndex>> welded_subgraphs_set(
      welded_subgraphs.begin(), welded_subgraphs.end());
  EXPECT_EQ(welded_subgraphs_set, expected_subgraphs);

  // Verify we can query the list of bodies welded to a particular Link.
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(9)).size(), 1);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(11)).size(), 1);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(4)), expected_subgraphA);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(13)), expected_subgraphA);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(10)), expected_subgraphB);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(6)), expected_subgraphB);

  // TODO(sherm1) Move to its own test suite.
  graph.BuildModel();
  graph.DumpGraph("WeldedSubgraphs (not combined)");
  const SpanningForestModel& model = graph.model();
  model.SanityCheckForest();
  model.DumpModel("WeldedSubgraphs (not combined)");

  // Note that this is a question about how these Links got modeled, not
  // about the original graph.
  EXPECT_EQ(graph.FindFirstCommonAncestor(LinkIndex(11), LinkIndex(12)),
            LinkIndex(5));

  // Now combine composites so they get a single Mobod.
  graph.BuildModel(ModelingOptions::CombineCompositeLinks);
  graph.DumpGraph("WeldedSubgraphs (combined)");
  model.SanityCheckForest();
  model.DumpModel("WeldedSubgraphs (combined)");
}

// Ten links, 8 in a tree and 2 free ones.
GTEST_TEST(LinkJointGraph, SimpleTrees) {
  LinkJointGraph graph;
  graph.RegisterJointType(kRevoluteType, 1, 1);
  graph.RegisterJointType(kPrismaticType, 1, 1);

  // The first Link added defines the world's name and model instance.
  graph.AddLink(kWorldLinkName, world_model_instance());

  // We'll add Links and Joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the model.
  for (int i = 1; i <= 10; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }
  const std::vector<std::pair<int, int>> joints
      {{3, 1}, {3, 2}, {8, 3}, {10, 8}, {10, 9}, {9, 4}, {9, 7}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance,
                   kRevoluteType, LinkIndex(joints[i].first),
                   LinkIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 11);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  // TODO(sherm1) Move to its own test suite.
  graph.BuildModel(ModelingOptions::UseRpyFloatingJoints);
  graph.DumpGraph("SimpleTrees");
  const SpanningForestModel& model = graph.model();
  model.SanityCheckForest();
  model.DumpModel("SimpleTrees");

  // Now connect the "leftmost" and "rightmost" Links (1 and 7 resp.)
  // to World explicitly, forming a loop.
  graph.AddJoint("world_joint_7", model_instance, kRevoluteType,
                 LinkIndex(0), LinkIndex(1));
  graph.AddJoint("world_joint_8", model_instance, kRevoluteType,
                 LinkIndex(0), LinkIndex(7));
  graph.BuildModel();
  graph.DumpGraph("SimpleTrees with 1 & 7 connected to World");

  model.SanityCheckForest();
  model.DumpModel("SimpleTrees with 1 & 7 connected to World");
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

  // The first Link added defines the world's name and model instance.
  graph.AddLink(kWorldLinkName, world_model_instance());

  // We'll add Links and Joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the model.
  for (int i = 1; i <= 6; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }
  const std::vector<std::pair<int, int>> joints
      {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {0, 5}, {5, 6}, {6, 4}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance,
                   kRevoluteType, LinkIndex(joints[i].first),
                   LinkIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 7);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  // TODO(sherm1) Move to its own test suite.
  graph.BuildModel();
  graph.DumpGraph("Massful loop");
  const SpanningForestModel& model = graph.model();
  model.SanityCheckForest();
  model.DumpModel("Massful loop");

  EXPECT_EQ(ssize(model.trees()), 2);
  EXPECT_EQ(model.trees()[0].num_mobods(), 4);
  EXPECT_EQ(model.trees()[1].num_mobods(), 3);

  graph.change_link_flags(LinkIndex(3), LinkFlags::TreatAsMassless);
  graph.BuildModel();
  graph.DumpGraph("Massless loop");
  model.SanityCheckForest();
  model.DumpModel("Massless loop");

  EXPECT_EQ(ssize(model.trees()), 2);
  EXPECT_EQ(model.trees()[0].num_mobods(), 4);
  EXPECT_EQ(model.trees()[1].num_mobods(), 3);

  graph.change_link_flags(LinkIndex(4), LinkFlags::TreatAsMassless);
  graph.BuildModel();
  graph.DumpGraph("Massless x 2 loop");
  model.SanityCheckForest();
  model.DumpModel("Massless x 2 loop");

  EXPECT_EQ(ssize(model.trees()), 2);
  EXPECT_EQ(model.trees()[0].num_mobods(), 5);
  EXPECT_EQ(model.trees()[1].num_mobods(), 2);
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

  graph.AddLink("World", world_model_instance());

  graph.AddLink("massless_1", model_instance, LinkFlags::TreatAsMassless);
  graph.AddLink("massless_2", model_instance, LinkFlags::TreatAsMassless);
  graph.AddLink("link_3", model_instance);

  graph.AddJoint("prismatic_0", model_instance, kPrismaticType,
                 world_index(), LinkIndex(1));
  graph.AddJoint("prismatic_1", model_instance, kPrismaticType,
                 world_index(), LinkIndex(2));
  graph.AddJoint("revolute_2", model_instance, kRevoluteType,
                 LinkIndex(1), LinkIndex(3));
  graph.AddJoint("revolute_3", model_instance, kRevoluteType,
                 LinkIndex(2), LinkIndex(3));

  EXPECT_EQ(ssize(graph.links()), 4);  // Before modeling (includes World).
  EXPECT_EQ(graph.num_user_links(), 4);

  const SpanningForestModel& model = graph.BuildModel();
  graph.DumpGraph("MasslessBodiesShareSplitLink");
  model.SanityCheckForest();
  model.DumpModel("MasslessBodiesShareSplitLink");

  EXPECT_EQ(ssize(graph.links()), 5);  // After modeling.
  EXPECT_EQ(graph.num_user_links(), 4);
  EXPECT_EQ(ssize(graph.constraints()), 1);
  EXPECT_EQ(graph.num_user_constraints(), 0);

  const auto& shadow_link = graph.links(LinkIndex(4));
  EXPECT_TRUE(shadow_link.is_shadow());
  EXPECT_EQ(shadow_link.primary_link(), LinkIndex(3));
  EXPECT_EQ(shadow_link.mobod_index(), MobodIndex(4));
  EXPECT_EQ(shadow_link.inboard_joint_index(), JointIndex(3));
  EXPECT_EQ(ssize(shadow_link.joints()), 1);
  EXPECT_TRUE(shadow_link.joints_as_parent().empty());
  EXPECT_EQ(shadow_link.joints_as_child()[0], JointIndex(3));
  EXPECT_EQ(shadow_link.joints()[0], JointIndex(3));

  EXPECT_EQ(graph.links(LinkIndex(3)).num_shadows(), 1);
  EXPECT_EQ(graph.links(LinkIndex(2)).num_shadows(), 0);
  EXPECT_EQ(graph.links(LinkIndex(4)).num_shadows(), 0);

  EXPECT_EQ(ssize(model.mobods()), 5);
  EXPECT_EQ(ssize(model.trees()), 2);
  EXPECT_EQ(model.trees(TreeIndex(0)).num_mobods(), 2);
  EXPECT_EQ(model.trees(TreeIndex(1)).num_mobods(), 2);
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

  graph.AddLink("World", world_model_instance());
  for (int i = 1; i <= 7; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  const std::vector<std::pair<int, int>> joints
      {{1, 2}, {1, 4}, {1, 3}, {2, 5}, {4, 7}, {3, 6}, {5, 6}, {7, 6}};
  for (int i = 0; i < 8; ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance,
                   kRevoluteType, LinkIndex(joints[i].first),
                   LinkIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 8);  // Before modeling (includes World).
  EXPECT_EQ(ssize(graph.joints()), 8);
  EXPECT_EQ(ssize(graph.constraints()), 0);

  const SpanningForestModel& model = graph.BuildModel();
  graph.DumpGraph("DoubleLoop");
  model.SanityCheckForest();
  model.DumpModel("DoubleLoop");

  EXPECT_EQ(ssize(graph.links()), 10);  // After modeling.
  EXPECT_EQ(ssize(graph.joints()), 9);
  EXPECT_EQ(ssize(graph.constraints()), 2);
  EXPECT_EQ(graph.num_user_links(), 8);
  EXPECT_EQ(graph.num_user_joints(), 8);
  EXPECT_EQ(graph.num_user_constraints(), 0);

  EXPECT_EQ(graph.links(LinkIndex(6)).num_shadows(), 2);
  EXPECT_TRUE(graph.links(LinkIndex(8)).is_shadow());
  EXPECT_TRUE(graph.links(LinkIndex(9)).is_shadow());

  EXPECT_EQ(graph.links(LinkIndex(5)).num_shadows(), 0);
  EXPECT_EQ(graph.links(LinkIndex(7)).num_shadows(), 0);

  EXPECT_EQ(ssize(model.mobods()), 10);
  EXPECT_EQ(ssize(model.trees()), 1);
  const SpanningForestModel::Tree& tree = model.trees(TreeIndex(0));
  EXPECT_EQ(tree.num_mobods(), 9);
  EXPECT_EQ(tree.height(), 4);
  EXPECT_EQ(tree.base_mobod(), MobodIndex(1));
  EXPECT_EQ(tree.last_mobod(), MobodIndex(9));

  // ith entry gives the modeled Link or Joint for Mobod i (see picture above).
  const std::array mobod2link{0, 1, 2, 5, 8, 4, 7, 9, 3, 6};
  const std::array mobod2joint{-1, 8, 0, 3, 6, 1, 4, 7, 2, 5};
  for (const SpanningForestModel::Mobod& mobod : model.mobods()) {
    EXPECT_EQ(mobod.link(), LinkIndex(mobod2link[mobod.index()]));
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
  EXPECT_EQ(weld0.parent_link(), LinkIndex(6));
  EXPECT_EQ(weld0.child_link(), LinkIndex(8));
  EXPECT_EQ(weld1.parent_link(), LinkIndex(6));
  EXPECT_EQ(weld1.child_link(), LinkIndex(9));

  // Added welds should be named the same as their shadow Link.
  EXPECT_EQ(weld0.name(), graph.links(LinkIndex(8)).name());
  EXPECT_EQ(weld1.name(), graph.links(LinkIndex(9)).name());
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

  graph.AddLink("World", world_model_instance());
  for (int i = 1; i <= 10; ++i) {
      graph.AddLink("link" + std::to_string(i), model_instance);
  }

  const std::vector<std::pair<int, int>> weld_joints{{1, 2}, {3, 4}, {5, 6}};
  const std::vector<std::pair<int, int>> revolute_joints{
    {0, 1}, {2, 3}, {4, 5}, {0, 7}, {7, 8}, {8, 9}, {9, 10}, {6, 10}};
  for (int i = 0; i < ssize(weld_joints); ++i) {
    graph.AddJoint("weld_joint_" + std::to_string(i), model_instance,
                   "weld", LinkIndex(weld_joints[i].first),
                   LinkIndex(weld_joints[i].second));
  }
  for (int i = 0; i < ssize(revolute_joints); ++i) {
    const int j = ssize(weld_joints) + i;  // joint number
    graph.AddJoint("revolute_joint_" + std::to_string(j), model_instance,
                   kRevoluteType, LinkIndex(revolute_joints[i].first),
                   LinkIndex(revolute_joints[i].second));
  }

  // Before modeling
  EXPECT_EQ(ssize(graph.links()), 11);  // counting World
  EXPECT_EQ(ssize(graph.joints()), 11);
  EXPECT_EQ(ssize(graph.constraints()), 0);

  const SpanningForestModel& model =
      graph.BuildModel(ModelingOptions::CombineCompositeLinks);
  model.SanityCheckForest();

  // After modeling
  EXPECT_EQ(ssize(graph.links()), 12);  // split one, added shadow
  EXPECT_EQ(ssize(graph.joints()), 11);  // no change
  EXPECT_EQ(ssize(graph.constraints()), 1);  // welded shadow to primary
  EXPECT_EQ(ssize(graph.composite_links()), 3);

  EXPECT_EQ(ssize(model.mobods()), 9);
  EXPECT_EQ(ssize(model.loop_constraints()), 1);
  EXPECT_EQ(ssize(model.trees()), 2);
  EXPECT_EQ(ssize(model.composite_mobods()), 0);

  EXPECT_EQ(model.mobods(MobodIndex(1)).follower_links(),
            (std::vector<LinkIndex>{LinkIndex(1), LinkIndex(2)}));
  EXPECT_EQ(model.mobods(MobodIndex(2)).follower_links(),
            (std::vector<LinkIndex>{LinkIndex(3), LinkIndex(4)}));
  EXPECT_EQ(model.mobods(MobodIndex(3)).follower_links(),
            (std::vector<LinkIndex>{LinkIndex(5), LinkIndex(6)}));

  const SpanningForestModel::Tree tree0 = model.trees(TreeIndex(0)),
  tree1 = model.trees(TreeIndex(1));
  EXPECT_EQ(tree0.num_mobods(), 4);
  EXPECT_EQ(tree0.nq(), 4);
  EXPECT_EQ(tree1.num_mobods(), 4);
  EXPECT_EQ(tree1.nq(), 4);

  graph.DumpGraph("LoopWithComposites");
  model.DumpModel("LoopWithComposites");

  // Sanity checks for graph copying and assignment. These are mostly
  // compiler-generated so we just need to test any field and that the
  // bespoke backpointer adjustments get done correctly.
  LinkJointGraph graph_copy(graph);
  EXPECT_EQ(ssize(graph_copy.links()), 12);
  EXPECT_TRUE(graph_copy.is_model_valid());
  const SpanningForestModel& copy_model = graph_copy.model();
  copy_model.SanityCheckForest();
  EXPECT_NE(&copy_model, &model);
  EXPECT_EQ(&copy_model.graph(), &graph_copy);  // backpointer

  LinkJointGraph graph_assign;
  graph_assign = graph;
  EXPECT_EQ(ssize(graph_assign.links()), 12);
  EXPECT_TRUE(graph_assign.is_model_valid());
  EXPECT_NE(&graph_assign.model(), &model);
  graph_assign.model().SanityCheckForest();
  EXPECT_EQ(&graph_assign.model().graph(), &graph_assign);

  LinkJointGraph graph_move(std::move(graph));
  EXPECT_EQ(ssize(graph_move.links()), 12);
  EXPECT_EQ(ssize(graph.links()), 0);
  EXPECT_EQ(&graph_move.model(), &model);
  graph_move.model().SanityCheckForest();
  EXPECT_EQ(&graph_move.model().graph(), &graph_move);
  // graph is now default-constructed so still has a model
  EXPECT_NE(&graph.model(), &model);
  EXPECT_FALSE(graph.is_model_valid());
  EXPECT_EQ(&graph.model().graph(), &graph);
  graph.model().SanityCheckForest();  // Should be empty but OK

  LinkJointGraph graph_move_assign;
  graph_move_assign = std::move(graph_copy);
  EXPECT_EQ(ssize(graph_move_assign.links()), 12);
  EXPECT_TRUE(graph_move_assign.is_model_valid());
  EXPECT_EQ(ssize(graph_copy.links()), 0);
  EXPECT_EQ(&graph_move_assign.model(), &copy_model);
  graph_move_assign.model().SanityCheckForest();
  EXPECT_EQ(&graph_move_assign.model().graph(), &graph_move_assign);
  // graph_copy is now default-constructed so still has a model
  EXPECT_NE(&graph_copy.model(), &copy_model);
  EXPECT_FALSE(graph_copy.is_model_valid());
  EXPECT_EQ(&graph_copy.model().graph(), &graph_copy);
  graph_copy.model().SanityCheckForest();  // Should be empty but OK
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
