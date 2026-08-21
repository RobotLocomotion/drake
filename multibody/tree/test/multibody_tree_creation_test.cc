#include "drake/common/test_utilities/expect_no_throw.h"
/* clang-format off to disable clang-format-includes */
#include "drake/multibody/tree/multibody_tree-inl.h"
/* clang-format on */

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {

class MultibodyElementTester {
 public:
  MultibodyElementTester() = delete;
  template <typename T>
  static bool has_parent_tree(const MultibodyElement<T>& element) {
    return element.has_parent_tree();
  }

  template <typename T>
  static const internal::MultibodyTree<T>& get_parent_tree(
      const MultibodyElement<T>& element) {
    return element.get_parent_tree();
  }
};

// Friend class for accessing Joint<T> protected/private internals.
class JointTester {
 public:
  JointTester() = delete;
  static const internal::RevoluteMobilizer<double>* get_mobilizer(
      const RevoluteJoint<double>& joint) {
    return &joint.get_mobilizer();
  }
};

namespace internal {
namespace {

using Eigen::Vector3d;
using std::make_unique;
using std::set;
using std::unique_ptr;

// Tests the basic MultibodyTree API to add bodies and joints.
GTEST_TEST(MultibodyTree, BasicAPIToAddBodiesAndJoints) {
  auto model = std::make_unique<MultibodyTree<double>>();

  // Initially there is only one body, the world.
  EXPECT_EQ(model->num_bodies(), 1);

  // Retrieves the world body.
  const RigidBody<double>& world_body = model->world_body();
  EXPECT_EQ(world_body.name(), "world");

  // Make sure the (dispreferred) Body alias is working.
  const Body<double>& also_world_body = model->world_body();
  EXPECT_EQ(also_world_body.name(), "world");

  // Creates a NaN SpatialInertia to instantiate the RigidBody links of the
  // pendulum. Using a NaN spatial inertia is ok so far since we are still
  // not performing any numerical computations. This is only to test API.
  // M_Bo_B is the spatial inertia about the body frame's origin Bo and
  // expressed in the body frame B.
  const auto M_Bo_B = SpatialInertia<double>::NaN();

  // Adds a new body to the world.
  const RigidBody<double>& pendulum = model->AddRigidBody("pendulum", M_Bo_B);
  EXPECT_EQ(pendulum.scoped_name().get_full(),
            "DefaultModelInstance::pendulum");
  EXPECT_EQ(pendulum.body_frame().scoped_name().get_full(),
            "DefaultModelInstance::pendulum");

  // Adds a revolute joint.
  DRAKE_EXPECT_NO_THROW((model->AddJoint<RevoluteJoint>(
      "joint0", world_body, {}, pendulum, {}, Vector3d::UnitZ())));

  // We cannot add another joint between the same two bodies.
  EXPECT_THROW((model->AddJoint<RevoluteJoint>(
                   "joint1", world_body, {}, pendulum, {}, Vector3d::UnitZ())),
               std::exception);

  // Even if connected in the opposite order.
  EXPECT_THROW((model->AddJoint<RevoluteJoint>(
                   "joint1", pendulum, {}, world_body, {}, Vector3d::UnitZ())),
               std::exception);

  // Verify we cannot add a joint between a body and itself.
  DRAKE_EXPECT_THROWS_MESSAGE(
      (model->AddJoint<RevoluteJoint>("joint1", pendulum, {}, pendulum, {},
                                      Vector3d::UnitZ())),
      "AddJoint.*joint1 would connect body pendulum to itself.*");

  // Adds a second pendulum.
  const RigidBody<double>& pendulum2 = model->AddRigidBody("pendulum2", M_Bo_B);
  model->AddJoint<RevoluteJoint>("joint1", world_body, {}, pendulum2, {},
                                 Vector3d::UnitZ());

  EXPECT_EQ(model->num_bodies(), 3);
  EXPECT_EQ(model->num_joints(), 2);

  // Topology is invalid before MultibodyTree::Finalize().
  EXPECT_FALSE(model->is_finalized());
  EXPECT_FALSE(model->graph().forest_is_valid());
  // Verifies that the topology of this model gets validated at finalize stage.
  DRAKE_EXPECT_NO_THROW(model->Finalize());
  EXPECT_TRUE(model->graph().forest_is_valid());
  EXPECT_TRUE(model->is_finalized());

  // Body identifiers are unique and are assigned by MultibodyTree in increasing
  // order starting with index = 0 (world_index()) for the "world" body.
  EXPECT_EQ(world_body.index(), world_index());
  EXPECT_EQ(pendulum.index(), BodyIndex(1));
  EXPECT_EQ(pendulum2.index(), BodyIndex(2));

  // Tests API to access bodies.
  EXPECT_EQ(model->get_body(BodyIndex(1)).index(), pendulum.index());
  EXPECT_EQ(model->get_body(BodyIndex(2)).index(), pendulum2.index());

  // Verifies that an exception is throw if a call to Finalize() is attempted to
  // an already finalized MultibodyTree.
  EXPECT_THROW(model->Finalize(), std::exception);

  // Verifies that after compilation no more bodies can be added.
  EXPECT_THROW(model->AddRigidBody("B", M_Bo_B), std::exception);
}

// Tests the basic MultibodyTree API to add bodies and joints.
// Tests we cannot currently create graph loops. See previous test for notes.
GTEST_TEST(MultibodyTree, TopologicalLoopDisallowed) {
  auto model = std::make_unique<MultibodyTree<double>>();
  const RigidBody<double>& world_body = model->world_body();
  const auto M_Bo_B = SpatialInertia<double>::NaN();
  const RigidBody<double>& pendulum = model->AddRigidBody("pendulum", M_Bo_B);
  model->AddJoint<RevoluteJoint>("joint0", world_body, {}, pendulum, {},
                                 Vector3d::UnitZ());
  const RigidBody<double>& pendulum2 = model->AddRigidBody("pendulum2", M_Bo_B);
  model->AddJoint<RevoluteJoint>("joint1", world_body, {}, pendulum2, {},
                                 Vector3d::UnitZ());

  EXPECT_EQ(model->num_bodies(), 3);
  EXPECT_EQ(model->num_joints(), 2);

  // Attempts to create a loop. Verify we get an exception at Finalize().
  model->AddJoint<RevoluteJoint>("joint2", pendulum, {}, pendulum2, {},
                                 Vector3d::UnitZ());

  EXPECT_EQ(model->num_bodies(), 3);
  EXPECT_EQ(model->num_joints(), 3);

  // Topology is invalid before MultibodyTree::Finalize() or after a
  // topological failure (unsupported loop in graph).
  EXPECT_FALSE(model->is_finalized());
  EXPECT_FALSE(model->graph().forest_is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->Finalize(),
      "The bodies and joints of this system form one or more loops.*");
  EXPECT_FALSE(model->graph().forest_is_valid());
  EXPECT_FALSE(model->is_finalized());
}

// Tests the correctness of MultibodyElement checks to verify one or more
// elements belong to a given MultibodyTree.
GTEST_TEST(MultibodyTree, MultibodyElementChecks) {
  auto model1 = std::make_unique<MultibodyTree<double>>();
  auto model2 = std::make_unique<MultibodyTree<double>>();

  // Creates a NaN SpatialInertia to instantiate the RigidBody links of the
  // pendulum. Using a NaN spatial inertia is ok so far since we are still
  // not performing any numerical computations. This is only to test API.
  // M_Bo_B is the spatial inertia about the body frame's origin Bo and
  // expressed in the body frame B.
  const auto M_Bo_B = SpatialInertia<double>::NaN();

  const RigidBody<double>& body1 = model1->AddRigidBody("body1", M_Bo_B);
  const RigidBody<double>& body2 = model2->AddRigidBody("body2", M_Bo_B);

  // Verify we can add a joint between body1 and the world of model1.
  const RevoluteJoint<double>& pin1 = model1->AddJoint<RevoluteJoint>(
      "pin1", model1->world_body(), std::nullopt /*inboard frame*/, body1,
      std::nullopt /*outboard frame*/, Vector3d::UnitZ() /*axis of rotation*/);

  // Verify we can't add a joint between bodies that belong to another plant.
  DRAKE_EXPECT_THROWS_MESSAGE(
      (model1->AddJoint<RevoluteJoint>(
          "nogood", model1->world_body(), std::nullopt,
          body2 /*body2 belongs to model2, not model1!!!*/, std::nullopt,
          Vector3d::UnitZ() /*axis of rotation*/)),
      "AddJoint.*can't add joint nogood.*world.*body2.*different.*Plant.*");

  // model1 is complete. Expect no-throw.
  DRAKE_EXPECT_NO_THROW(model1->Finalize());
  // model 1 has a single dof corresponding to the pin joint.
  EXPECT_EQ(model1->num_positions(), 1);
  EXPECT_EQ(model1->num_velocities(), 1);

  // model2->Finalize() shouldn't throw. Since body2 has no joint, we will
  // default it to be free and will assign a floating joint to it.
  DRAKE_EXPECT_NO_THROW(model2->Finalize());
  // The number of dofs should match a quaternion floating joint.
  EXPECT_EQ(model2->num_positions(), 7);
  EXPECT_EQ(model2->num_velocities(), 6);

  // Tests that the created multibody elements indeed do have a parent
  // MultibodyTree.
  EXPECT_TRUE(MultibodyElementTester::has_parent_tree(body1));
  EXPECT_TRUE(MultibodyElementTester::has_parent_tree(body2));
  EXPECT_TRUE(MultibodyElementTester::has_parent_tree(pin1));

  // Tests that bodies belong to distinct MultibodyTrees.
  const auto& body1_tree = MultibodyElementTester::get_parent_tree(body1);
  const auto& body2_tree = MultibodyElementTester::get_parent_tree(body2);
  EXPECT_NE(&body1_tree, &body2_tree);
  EXPECT_EQ(&MultibodyElementTester::get_parent_tree(model1->world_body()),
            &body1_tree);
  EXPECT_EQ(&MultibodyElementTester::get_parent_tree(model2->world_body()),
            &body2_tree);

  // Verify that bodies have the correct parent tree.
  EXPECT_EQ(&body1_tree, model1.get());
  EXPECT_EQ(&body2_tree, model2.get());
}

// This unit test builds a MultibodyTree as shown in the schematic below, where
// the number inside the boxes corresponds to each of the Links' indices as
// assigned by MultibodyTree in the order Links are created. The "j?" next to
// each connection denotes a Joint with the number corresponding to the Joint
// index (assigned by MultibodyTree in the order Joints are created). At
// Finalize(), a Mobilizer is created to implement each Joint, and a BodyNode is
// created per (body, inboard_mobilizer) pair. Body node indices, shown in
// parentheses, are assigned in DFT order, and we expect Mobilizer indices to be
// identical (a dummy Mobilizer 0 is created for the World Mobod). In addition,
// we know that the SpanningForest assigns mobilized bodies to each branch in
// the order Joints were added. For instance, in the schematic below, node (1)
// is created before node (2) because joint j2 was added before j4. Similarly,
// node (7) is created before (8) because joint j1 was added before j6. Notice
// that Links 8 and 9 are anchored to World. We have not selected the option to
// merge welded-together links onto a single mobilized body, so these will
// still form a tree, though with 0 dofs. Thus the full model has four trees,
// with bases at Links 7, 5, 9, and 4 (Mobods 1, 2, 4, 6 resp.).
//
//                 ┌───┐
//                 │ 0 │(0)                              Level 0 (root, world)
//                 └─┬─┘
//                   │
//     ┌─────────────┼──────────┬────────────────┐
//     │ j2          │ j4       │ j5 weld        │ j7
//   ┌─┴─┐         ┌─┴─┐      ┌─┴─┐            ┌─┴─┐
//   │ 7 │(1)      │ 5 │(2)   │ 9 │(4)         │ 4 │(6)         Level 1
//   └───┘         └─┬─┘      └─┬─┘            └─┬─┘
//                   │          │                │
//                   │          │          ┌─────┴──────┐
//                   │ j3       │ j8 weld  │ j1         │ j6
//                 ┌─┴─┐      ┌─┴─┐      ┌─┴─┐        ┌─┴─┐
//                 │ 3 │(3)   │ 8 │(5)   │ 2 │(7)     │ 1 │(8)  Level 2
//                 └───┘      └───┘      └───┘        └─┬─┘
//                                                      │ j0
//                                                    ┌─┴─┐
//                                                    │ 6 │(9)  Level 3
//                                                    └───┘
//
class TreeTopologyTests : public ::testing::Test {
 public:
  // Creates MultibodyTree according to the schematic above.
  void SetUp() override {
    model_ = std::make_unique<MultibodyTree<double>>();

    const int kNumRigidBodies = 10;
    bodies_.push_back(&model_->world_body());
    for (int i = 1; i < kNumRigidBodies; ++i) AddTestBody(i);

    // Adds Joints to connect bodies according to the following diagram:
    ConnectBodies(*bodies_[1], *bodies_[6]);  // joint 0
    ConnectBodies(*bodies_[4], *bodies_[2]);  // joint 1
    ConnectBodies(*bodies_[0], *bodies_[7]);  // joint 2
    ConnectBodies(*bodies_[5], *bodies_[3]);  // joint 3
    ConnectBodies(*bodies_[0], *bodies_[5]);  // joint 4
    WeldBodies(*bodies_[0], *bodies_[9]);     // joint 5
    ConnectBodies(*bodies_[4], *bodies_[1]);  // joint 6
    ConnectBodies(*bodies_[0], *bodies_[4]);  // joint 7
    WeldBodies(*bodies_[9], *bodies_[8]);     // joint 8
  }

  const RigidBody<double>* AddTestBody(int i) {
    // NaN SpatialInertia to instantiate the RigidBody objects.
    // It is safe here since this tests only focus on topological information.
    const auto M_Bo_B = SpatialInertia<double>::NaN();
    const RigidBody<double>* body =
        &model_->AddRigidBody(fmt::format("TestBody_{}", i), M_Bo_B);
    bodies_.push_back(body);
    return body;
  }

  void ConnectBodies(const RigidBody<double>& inboard,
                     const RigidBody<double>& outboard) {
    // Just for fun, here we explicitly state that the frame on body
    // "inboard" (frame P) IS the joint frame F, done by passing the empty
    // curly braces {}.
    // We DO want the model to have a frame M on body "outboard" (frame B)
    // with a pose X_BM = Identity. We therefore pass the identity transform.
    const auto* joint = &model_->AddJoint<RevoluteJoint>(
        fmt::format("FooJoint{}", joint_counter_++), inboard,
        {}, /* Model does not create frame F, and makes F = P.  */
        outboard,
        math::RigidTransformd::Identity(), /* Model creates frame M. */
        Vector3d::UnitZ());
    joints_.push_back(joint);
  }

  void WeldBodies(const RigidBody<double>& inboard,
                  const RigidBody<double>& outboard) {
    const Joint<double>* joint = &model_->AddJoint<WeldJoint>(
        fmt::format("WeldJoint{}", joint_counter_++), inboard, {}, outboard, {},
        math::RigidTransformd::Identity());
    joints_.push_back(joint);
  }

  void FinalizeModel() { model_->Finalize(); }

  // Performs a number of tests on the BodyNode corresponding to the
  // body indexed by `body`.
  static void TestBodyNode(const SpanningForest& forest, BodyIndex body) {
    // In case of merged composites, many links may follow the same mobod.
    // Make sure the link's mobod agrees that the link follows it.
    const LinkJointGraph::Link& link = forest.link_by_index(body);
    const MobodIndex mobod_index = link.mobod_index();
    const SpanningForest::Mobod& mobod = forest.mobods(mobod_index);
    EXPECT_EQ(mobod.index(), mobod_index);
    mobod.HasFollower(link.ordinal());

    const MobodIndex parent_mobod_index = mobod.inboard();
    EXPECT_TRUE(parent_mobod_index.is_valid() ^ link.is_world());

    if (link.is_world()) return;

    // Verify that the link's mobod is actually a child of its parent mobod.
    const SpanningForest::Mobod& parent_mobod =
        forest.mobods(parent_mobod_index);
    const std::vector<MobodIndex>& child_mobods = parent_mobod.outboards();
    EXPECT_TRUE(std::find(child_mobods.begin(), child_mobods.end(),
                          mobod_index) != child_mobods.end());
  }

  static void VerifyTopology(const SpanningForest& forest) {
    const int kNumRigidBodies = 10;

    EXPECT_EQ(forest.num_links(), kNumRigidBodies);
    EXPECT_EQ(forest.num_mobods(), kNumRigidBodies);
    EXPECT_EQ(forest.height(), 4);

    // These sets contain the indexes of the RigidBodies in each tree level.
    // The order of these indexes in each set is not important, but only the
    // fact that they belong to the appropriate set.
    const set<LinkIndex> expected_level0 = {LinkIndex(0)};
    const set<LinkIndex> expected_level1 = {LinkIndex(4), LinkIndex(7),
                                            LinkIndex(5), LinkIndex(9)};
    const set<LinkIndex> expected_level2 = {LinkIndex(2), LinkIndex(1),
                                            LinkIndex(3), LinkIndex(8)};
    const set<LinkIndex> expected_level3 = {LinkIndex(6)};

    // Comparison of sets. The order of the elements is not important.
    std::vector<std::set<LinkIndex>> levels(forest.height());
    for (const LinkJointGraph::Link& link : forest.links()) {
      const int level = forest.mobods(link.mobod_index()).level();
      levels[level].insert(link.index());
    }
    EXPECT_EQ(levels[0], expected_level0);
    EXPECT_EQ(levels[1], expected_level1);
    EXPECT_EQ(levels[2], expected_level2);
    EXPECT_EQ(levels[3], expected_level3);

    const std::map<LinkIndex, int> expected_num_outboards{
        {LinkIndex(0), 4}, {LinkIndex(1), 1}, {LinkIndex(2), 0},
        {LinkIndex(3), 0}, {LinkIndex(4), 2}, {LinkIndex(5), 1},
        {LinkIndex(6), 0}, {LinkIndex(7), 0}, {LinkIndex(8), 0},
        {LinkIndex(9), 1}};

    // Verifies the expected number of child nodes.
    for (const auto& [link_index, num_outboards] : expected_num_outboards) {
      const LinkOrdinal ordinal = forest.graph().index_to_ordinal(link_index);
      const LinkJointGraph::Link& link = forest.links(ordinal);
      const SpanningForest::Mobod& mobod = forest.mobods(link.mobod_index());
      EXPECT_EQ(ssize(mobod.outboards()), num_outboards);
    }

    // Checks the correctness of each BodyNode associated with a Link.
    for (BodyIndex index(0); index < kNumRigidBodies; ++index) {
      TestBodyNode(forest, index);
    }

    // We now verify the precise expected topology. We use our internal
    // knowledge that branches are created according to the order in which
    // Joints are added. Refer to schematic in the documentation of this
    // test fixture.
    const std::map<MobodIndex, LinkIndex> expected_mobod_to_link = {
        {MobodIndex(0), LinkIndex(0)}, {MobodIndex(1), LinkIndex(7)},
        {MobodIndex(2), LinkIndex(5)}, {MobodIndex(3), LinkIndex(3)},
        {MobodIndex(4), LinkIndex(9)}, {MobodIndex(5), LinkIndex(8)},
        {MobodIndex(6), LinkIndex(4)}, {MobodIndex(7), LinkIndex(2)},
        {MobodIndex(8), LinkIndex(1)}, {MobodIndex(9), LinkIndex(6)}};

    for (const auto& [mobod_index, primary_link_index] :
         expected_mobod_to_link) {
      const SpanningForest::Mobod& mobod = forest.mobods(mobod_index);
      const LinkJointGraph::Link& link = forest.links(mobod.link_ordinal());
      EXPECT_EQ(link.index(), primary_link_index);
    }

    // Verify the expected forest of trees.
    EXPECT_EQ(forest.num_trees(), 4);
    EXPECT_EQ(forest.trees(TreeIndex(0)).nv(), 1);
    EXPECT_EQ(forest.trees(TreeIndex(1)).nv(), 2);
    EXPECT_EQ(forest.trees(TreeIndex(2)).nv(), 0);
    EXPECT_EQ(forest.trees(TreeIndex(3)).nv(), 4);
    EXPECT_EQ(forest.trees(TreeIndex(0)).v_start(), 0);
    EXPECT_EQ(forest.trees(TreeIndex(1)).v_start(), 1);
    EXPECT_EQ(forest.trees(TreeIndex(2)).v_start(), 3);
    EXPECT_EQ(forest.trees(TreeIndex(3)).v_start(), 3);

    const std::map<LinkIndex, TreeIndex> expected_link_to_tree = {
        {LinkIndex(0), TreeIndex()},  {LinkIndex(1), TreeIndex(3)},
        {LinkIndex(2), TreeIndex(3)}, {LinkIndex(3), TreeIndex(1)},
        {LinkIndex(4), TreeIndex(3)}, {LinkIndex(5), TreeIndex(1)},
        {LinkIndex(6), TreeIndex(3)}, {LinkIndex(7), TreeIndex(0)},
        {LinkIndex(8), TreeIndex(2)}, {LinkIndex(9), TreeIndex(2)}};

    // The world body does not belong to a tree. Therefore the returned index is
    // invalid.
    for (const auto& [link_index, tree_index] : expected_link_to_tree) {
      if (link_index == world_index()) {
        EXPECT_FALSE(forest.link_to_tree_index(link_index).is_valid());
        continue;
      }
      EXPECT_EQ(forest.link_to_tree_index(link_index), tree_index);
    }

    // No velocities map to Tree 2.
    const std::map<int, TreeIndex> expected_velocity_to_tree = {
        {0, TreeIndex(0)}, {1, TreeIndex(1)}, {2, TreeIndex(1)},
        {3, TreeIndex(3)}, {4, TreeIndex(3)}, {5, TreeIndex(3)},
        {6, TreeIndex(3)}};

    EXPECT_EQ(forest.num_velocities(), 7);
    for (const auto& [velocity_index, tree_index] : expected_velocity_to_tree) {
      EXPECT_EQ(forest.v_to_tree_index(velocity_index), tree_index);
    }
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Bodies:
  std::vector<const RigidBody<double>*> bodies_;
  // Joints:
  std::vector<const Joint<double>*> joints_;

 private:
  int joint_counter_{0};  // Used to generate unique joint names.
};

// This unit tests verifies that the multibody topology is properly compiled.
TEST_F(TreeTopologyTests, Finalize) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 10);

  const SpanningForest& forest = model_->forest();
  EXPECT_EQ(forest.num_mobods(), 10);
  EXPECT_EQ(forest.height(), 4);

  VerifyTopology(forest);
}

// This unit tests verifies the correct number of generalized positions and
// velocities as well as the start indexes into the state vector.
TEST_F(TreeTopologyTests, SizesAndIndexing) {
  FinalizeModel();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 10);
  EXPECT_EQ(model_->num_joints(), 9);

  const SpanningForest& forest = model_->forest();

  EXPECT_EQ(forest.num_mobods(), 10);
  EXPECT_EQ(forest.height(), 4);
  EXPECT_EQ(forest.num_positions(), 7);
  EXPECT_EQ(forest.num_velocities(), 7);

  // In this case all mobilizers are RevoluteMobilizer objects with one
  // generalized position and one generalized velocity per mobilizer.
  int positions_index = 0;
  int velocities_index = 0;
  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    const LinkOrdinal active_link_ordinal = mobod.link_ordinal();
    const LinkJointGraph::Link active_link = forest.links(active_link_ordinal);

    EXPECT_EQ(active_link.mobod_index(), mobod.index());
    EXPECT_EQ(active_link.ordinal(), active_link_ordinal);

    EXPECT_EQ(positions_index, mobod.q_start());
    EXPECT_EQ(velocities_index, mobod.v_start());
    positions_index += mobod.nq();
    velocities_index += mobod.nv();
  }
  EXPECT_EQ(positions_index, forest.num_positions());
  EXPECT_EQ(velocities_index, forest.num_velocities());
}

// Verifies that the clone of a given MultibodyTree model created with
// MultibodyTree::Clone() has exactly the same topology as the original
// model.
TEST_F(TreeTopologyTests, Clone) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 10);
  EXPECT_EQ(model_->num_force_elements(), 1);
  const SpanningForest& forest = model_->forest();

  auto cloned_model = model_->Clone();
  EXPECT_EQ(cloned_model->num_bodies(), 10);
  EXPECT_EQ(cloned_model->num_mobilizers(), 10);
  EXPECT_EQ(cloned_model->num_force_elements(), 1);
  const SpanningForest& clone_forest = cloned_model->forest();

  // Verify the cloned topology actually is a different object.
  ASSERT_NE(&forest, &clone_forest);

  VerifyTopology(clone_forest);
}

// Verifies that the AutoDiffXd version of a given MultibodyTree model created
// with MultibodyTree::ToAutoDiffXd() has exactly the same topology as the
// original model.
TEST_F(TreeTopologyTests, ToAutoDiffXd) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 10);

  auto autodiff_model = model_->ToAutoDiffXd();
  EXPECT_EQ(autodiff_model->num_bodies(), 10);
  EXPECT_EQ(autodiff_model->num_mobilizers(), 10);
  const SpanningForest& autodiff_forest = autodiff_model->forest();

  VerifyTopology(autodiff_forest);
}

// Verifies that the symbolic version of a given MultibodyTree model created
// with MultibodyTree::ToSymbolic() has exactly the same topology as the
// original model.
TEST_F(TreeTopologyTests, ToSymbolic) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 10);

  auto symbolic_model = model_->CloneToScalar<symbolic::Expression>();
  EXPECT_EQ(symbolic_model->num_bodies(), 10);
  EXPECT_EQ(symbolic_model->num_mobilizers(), 10);
  const SpanningForest& symbolic_forest = symbolic_model->forest();

  VerifyTopology(symbolic_forest);
}

// Confirm that the topology methods SpanningForest::FindPathFromWorld() and
// LinkJointGraph::FindPathFromWorld() work as we expect them to by computing
// the path from a given Link to World of the tree on a known topology.
TEST_F(TreeTopologyTests, KinematicPathFromWorld) {
  FinalizeModel();
  const SpanningForest& forest = model_->forest();

  const BodyIndex body6_index(6);
  const MobodIndex body6_mobod_index =
      forest.link_by_index(body6_index).mobod_index();
  const SpanningForest::Mobod& body6_mobod = forest.mobods(body6_mobod_index);

  // Compute kinematic path from body 6 to the world. See documentation for the
  // test fixture TreeTopologyTests for details on the topology under test.
  const std::vector<MobodIndex> mobod_path_to_world =
      forest.FindPathFromWorld(body6_mobod_index);
  const std::vector<BodyIndex> link_path_to_world =
      model_->graph().FindPathFromWorld(body6_index);

  const int expected_path_size = body6_mobod.level() + 1;
  EXPECT_EQ(ssize(mobod_path_to_world), expected_path_size);
  EXPECT_EQ(ssize(link_path_to_world), expected_path_size);

  // These are the expected bodies in the path.
  const std::vector<BodyIndex> expected_links_path = {
      BodyIndex(0), BodyIndex(4), BodyIndex(1), BodyIndex(6)};
  EXPECT_EQ(link_path_to_world, expected_links_path);

  // The Mobod path should have Mobods that model the same Links.
  for (BodyIndex body_index : expected_links_path) {
    // The path is computed in terms of Mobod indexes. Therefore obtain
    // the expected value of the Mobod index from the expected value of the
    // body index.
    const MobodIndex expected_mobod_index =
        forest.link_by_index(body_index).mobod_index();
    const SpanningForest::Mobod& mobod = forest.mobods(expected_mobod_index);
    EXPECT_EQ(forest.links(mobod.link_ordinal()).index(), body_index);
    // Both expected and computed Mobods must be at the same level (depth) in
    // the tree.
    const int level = mobod.level();
    EXPECT_EQ(mobod_path_to_world[level], expected_mobod_index);
  }
}

TEST_F(TreeTopologyTests, GetTransitiveOutboardBodies) {
  FinalizeModel();
  const BodyIndex body1_index(1);
  const BodyIndex body2_index(2);
  const BodyIndex body4_index(4);
  const BodyIndex body6_index(6);
  const BodyIndex body8_index(8);
  const BodyIndex body9_index(9);

  const std::vector<BodyIndex> body4{body4_index};
  std::set<BodyIndex> expected_outboard_bodies{body1_index, body2_index,
                                               body4_index, body6_index};
  EXPECT_EQ(model_->GetBodiesOutboardOfBodies(body4), expected_outboard_bodies);

  const std::vector<BodyIndex> body14{body1_index, body4_index};
  EXPECT_EQ(model_->GetBodiesOutboardOfBodies(body14),
            expected_outboard_bodies);

  const std::vector<BodyIndex> body94{body9_index, body4_index};
  expected_outboard_bodies.insert(body8_index);
  expected_outboard_bodies.insert(body9_index);
  EXPECT_EQ(model_->GetBodiesOutboardOfBodies(body94),
            expected_outboard_bodies);

  const std::vector<BodyIndex> body6{body6_index};
  expected_outboard_bodies.clear();
  expected_outboard_bodies.insert(body6_index);
  EXPECT_EQ(model_->GetBodiesOutboardOfBodies(body6), expected_outboard_bodies);
}

// Unit test to verify that LinkJointGraph::GetSubgraphsOfWeldedLinks()
// behaves as we expect.
// This test creates a tree with a topology as shown below. Single vertical
// lines indicate bodies that are connected by a non-weld mobilizer. Double
// vertical lines indicate bodies that are welded.
// A "welded body" is defined as some connected sub-graph of the tree where the
// connected edges (i.e., mobilizers) are weld mobilizers (double vertical lines
// in the schematic below). According to this definition, welded bodies for the
// tree below are:
// - Welded body 0: w, l, m, n
// - Welded body 1: a, b
// - Welded body 2: c
// - Welded body 3: d, e, g, f
// - Welded body 4: i
// - Welded body 5: j, k
// - Welded body 6: h
//
// In no particular order however, welded body zero contains the world.
//
//
//                      +---+
//                      | w | (world body)
//          +-----------+-+-+-----------+-------------+
//          |             |             |             ║
//          |             |             |             ║
//          |             |             |             ║
//        +-v-+         +-v-+         +-v-+         +-v-+
//        | a |         | d |         | i |         | l |
//        +---+      +--+---+--+      +-+-+      +--+---+--+
//          ║        ║         ║        |        ║         ║
//          ║        ║         ║        |        ║         ║
//          ║        ║         ║        |        ║         ║
//        +-v-+    +-v-+     +-v-+    +-v-+    +-v-+     +-v-+
//        | b |    | e |     | g |    | j |    | m |     | n |
//        +---+    +---+     +---+    +-+-+    +---+     +---+
//         |         ║         |        ║
//         |         ║         |        ║
//         |         ║         |        ║
//       +-v-+     +-v-+     +-v-+    +-v-+
//       | c |     | f |     | h |    | k |
//       +---+     +---+     +---+    +-+-+
//
GTEST_TEST(WeldedBodies, CreateSubgraphsOfWeldedLinks) {
  MultibodyTree<double> model;

  // Helper to add a rigid body. For these tests the value of the spatial
  // inertia is not relevant and therefore we leave it un-initialized.
  auto AddRigidBody =
      [&model](const std::string& name) -> const RigidBody<double>& {
    return model.AddRigidBody(name, SpatialInertia<double>::NaN());
  };

  // Helper to add a joint between two bodies. For this test the actual type of
  // the joint is not relevant, only the fact that "is not" a WeldJoint.
  auto AddJoint = [&model](const std::string& name,
                           const RigidBody<double>& parent,
                           const RigidBody<double>& child) {
    model.AddJoint<RevoluteJoint>(name, parent, {}, child, {},
                                  Vector3<double>::UnitX());
  };

  // Helper method to add a WeldJoint between two bodies.
  auto AddWeldJoint = [&model](const std::string& name,
                               const RigidBody<double>& parent,
                               const RigidBody<double>& child) {
    model.AddJoint<WeldJoint>(name, parent, std::nullopt, child, std::nullopt,
                              math::RigidTransformd::Identity());
  };

  // Start building the test model.
  const RigidBody<double>& body_a = AddRigidBody("a");
  const RigidBody<double>& body_b = AddRigidBody("b");
  const RigidBody<double>& body_c = AddRigidBody("c");
  const RigidBody<double>& body_d = AddRigidBody("d");
  const RigidBody<double>& body_e = AddRigidBody("e");
  const RigidBody<double>& body_f = AddRigidBody("f");
  const RigidBody<double>& body_g = AddRigidBody("g");
  const RigidBody<double>& body_h = AddRigidBody("h");
  const RigidBody<double>& body_i = AddRigidBody("i");
  const RigidBody<double>& body_j = AddRigidBody("j");
  const RigidBody<double>& body_k = AddRigidBody("k");
  const RigidBody<double>& body_l = AddRigidBody("l");
  const RigidBody<double>& body_m = AddRigidBody("m");
  const RigidBody<double>& body_n = AddRigidBody("n");

  AddJoint("wa", model.world_body(), body_a);

  // Welded body formed by bodies a and b.
  AddWeldJoint("ab", body_a, body_b);

  AddJoint("bc", body_b, body_c);
  AddJoint("wd", model.world_body(), body_d);

  // Welded body formed by bodies d, e, f and, g.
  AddWeldJoint("de", body_d, body_e);
  AddWeldJoint("ef", body_e, body_f);
  AddWeldJoint("dg", body_d, body_g);

  AddJoint("gh", body_g, body_h);
  AddJoint("wi", model.world_body(), body_i);
  AddJoint("ij", body_i, body_j);

  // Welded body formed by bodies j and k.
  AddWeldJoint("jk", body_j, body_k);

  // Welded body formed by bodies w (world), l, m and n.
  AddWeldJoint("wl", model.world_body(), body_l);
  AddWeldJoint("lm", body_l, body_m);
  AddWeldJoint("ln", body_l, body_n);

  // We are done building the test model.
  model.Finalize();

  const LinkJointGraph& graph = model.graph();

  // Ask the graph to report the sets of composite (welded) links.
  std::vector<std::set<BodyIndex>> welded_bodies =
      graph.GetSubgraphsOfWeldedLinks();
  ASSERT_EQ(welded_bodies.size(), 7);

  // Welded body "0" must correspond to the set of bodies welded to the world.
  const std::set<BodyIndex>& world_welded_body = welded_bodies[0];
  // Therefore world_welded_body must contain the index of the world body.
  EXPECT_NE(world_welded_body.find(world_index()), world_welded_body.end());

  // Build the expected result.
  std::set<std::set<BodyIndex>> expected_welded_bodies;
  expected_welded_bodies.insert({body_a.index(), body_b.index()});
  expected_welded_bodies.insert({body_c.index()});
  expected_welded_bodies.insert(
      {body_d.index(), body_e.index(), body_f.index(), body_g.index()});
  expected_welded_bodies.insert({body_h.index()});
  expected_welded_bodies.insert({body_i.index()});
  expected_welded_bodies.insert({body_j.index(), body_k.index()});
  expected_welded_bodies.insert(
      {world_index(), body_l.index(), body_m.index(), body_n.index()});

  // In order to compare the computed list of welded bodies against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed list to a set.
  std::set<std::set<BodyIndex>> welded_bodies_set(welded_bodies.begin(),
                                                  welded_bodies.end());

  // Verify the computed list has the expected entries.
  EXPECT_EQ(welded_bodies_set, expected_welded_bodies);

  // All bodies in welded_bodies[0] are, by definition, anchored to the world.
  // Verify this by checking the LinkJointGraph (after building the Forest).
  for (size_t welded_body_index = 0; welded_body_index < welded_bodies.size();
       ++welded_body_index) {
    const std::set<BodyIndex>& welded_body = welded_bodies[welded_body_index];
    for (BodyIndex body_index : welded_body) {
      EXPECT_EQ(graph.link_by_index(body_index).is_anchored(),
                welded_body_index == 0 /* 'true' for anchored bodies. */);
    }
  }
}

// Helper function to create a unit inertia for a uniform-density cube B about
// Bo (B's origin point) from a given dimension (length).
// If length = 0, the spatial inertia is that of a particle.
// @param[in] length The length of any of the cube's edges.
// @retval M_BBo_B Cube B's unit inertia about point Bo (B's origin),
// expressed in terms of unit vectors Bx, By, Bz, each of which are parallel
// to sides (edges) of the cube. Point Bo is the centroid of the face of the
// cube whose outward normal is -Bx. Hence, the position vector from Bo to Bcm
// (B's center of mass) is p_BoBcm_B = Lx/2 Bx.
UnitInertia<double> MakeTestCubeUnitInertia(const double length = 1.0) {
  const UnitInertia<double> G_BBcm_B = UnitInertia<double>::SolidCube(length);
  const Vector3<double> p_BoBcm_B(length / 2, 0, 0);
  const UnitInertia<double> G_BBo_B =
      G_BBcm_B.ShiftFromCenterOfMass(-p_BoBcm_B);
  return G_BBo_B;
}

// Helper function to create a cube-shaped rigid body B and add it to a model.
// @param[in] model MultibodyTree to which body B is added.
// @param[in] body_name name of the body that is being added to the model.
// @param[in] link_length length, width, and depth of the cube-shaped body.
// @param[in] skip_validity_check setting which is `true` to skip the validity
//  check on the new body B's spatial inertia, which ensures an exception is not
//  thrown when setting body B's spatial inertia (which would otherwise occur if
//  mass or link_length is NaN). Avoiding this early exception allows for a
//  later exception to be thrown in a subsequent function and tested below.
const RigidBody<double>& AddCubicalLink(
    MultibodyTree<double>* model, const std::string& body_name,
    const double mass, const double link_length = 1.0,
    const bool skip_validity_check = false) {
  DRAKE_DEMAND(model != nullptr);
  const Vector3<double> p_BoBcm_B(link_length / 2, 0, 0);
  const UnitInertia<double> G_BBo_B = MakeTestCubeUnitInertia(link_length);
  const SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B,
                                       skip_validity_check);
  return model->AddRigidBody(body_name, M_BBo_B);
}

// Verify RigidBody::default_rotational_inertia() and related MultibodyTree
// methods.
GTEST_TEST(DefaultInertia, VerifyDefaultRotationalInertia) {
  // Create a model and add three rigid bodies, namely A, B, C.
  MultibodyTree<double> model;
  const double mA = 0, mB = 1, mC = 3;  // Mass of links A, B, C.
  const double length = 3;  // Length of each thin uniform-density link.
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);
  const RigidBody<double>& body_C = AddCubicalLink(&model, "bodyC", mC, length);

  // Verify the default mass for each of the bodies.
  EXPECT_EQ(body_A.default_mass(), mA);
  EXPECT_EQ(body_B.default_mass(), mB);
  EXPECT_EQ(body_C.default_mass(), mC);

  // Verify the default rotational inertia for each of the bodies.
  // To help with testing, create a RotationalInertia for a unit mass cube.
  const UnitInertia<double> G_SSo_S = MakeTestCubeUnitInertia(length);
  const RotationalInertia<double> I_A = body_A.default_rotational_inertia();
  const RotationalInertia<double> I_B = body_B.default_rotational_inertia();
  const RotationalInertia<double> I_C = body_C.default_rotational_inertia();
  EXPECT_EQ(I_A.CopyToFullMatrix3(), (mA * G_SSo_S).CopyToFullMatrix3());
  EXPECT_EQ(I_B.CopyToFullMatrix3(), (mB * G_SSo_S).CopyToFullMatrix3());
  EXPECT_EQ(I_C.CopyToFullMatrix3(), (mC * G_SSo_S).CopyToFullMatrix3());

  // Check if the default rotational inertia for each of rigid body is zero.
  EXPECT_TRUE(I_A.IsZero());
  EXPECT_FALSE(I_B.IsZero());
  EXPECT_FALSE(I_C.IsZero());

  // Create various sets of body indexes.
  std::vector<BodyIndex> bodies_AA, bodies_AB, bodies_BC, bodies_ABC;
  bodies_AA.assign({body_A.index()});
  bodies_AB.assign({body_A.index(), body_B.index()});
  bodies_BC.assign({body_B.index(), body_C.index()});
  bodies_ABC.assign({body_A.index(), body_B.index(), body_C.index()});

  // Verify the sum of the default masses in these sets of body indexes.
  const double mass_AA = model.CalcTotalDefaultMass(bodies_AA);
  const double mass_AB = model.CalcTotalDefaultMass(bodies_AB);
  const double mass_BC = model.CalcTotalDefaultMass(bodies_BC);
  const double mass_ABC = model.CalcTotalDefaultMass(bodies_ABC);
  EXPECT_EQ(mass_AA, mA);
  EXPECT_EQ(mass_AB, mA + mB);
  EXPECT_EQ(mass_BC, mB + mC);
  EXPECT_EQ(mass_ABC, mA + mB + mC);

  // Verify whether all default rotational inertia in these sets are zero.
  EXPECT_TRUE(model.AreAllDefaultRotationalInertiaZero(bodies_AA));
  EXPECT_FALSE(model.AreAllDefaultRotationalInertiaZero(bodies_AB));
  EXPECT_FALSE(model.AreAllDefaultRotationalInertiaZero(bodies_BC));
  EXPECT_FALSE(model.AreAllDefaultRotationalInertiaZero(bodies_ABC));
}

// Helper function to add a x-axis prismatic joint between two bodies.
void AddPrismaticJointX(MultibodyTree<double>* model, const std::string& name,
                        const RigidBody<double>& parent,
                        const RigidBody<double>& child) {
  DRAKE_DEMAND(model != nullptr);
  model->AddJoint<PrismaticJoint>(name, parent, {}, child, {},
                                  Vector3<double>::UnitX());
}

// Helper function to add a z-axis revolute joint between two bodies.
void AddRevoluteJointZ(MultibodyTree<double>* model, const std::string& name,
                       const RigidBody<double>& parent,
                       const RigidBody<double>& child) {
  DRAKE_DEMAND(model != nullptr);
  model->AddJoint<RevoluteJoint>(name, parent, {}, child, {},
                                 Vector3<double>::UnitZ());
}

// Helper function to add a weld joint between two bodies.
void AddWeldJoint(MultibodyTree<double>* model, const std::string& name,
                  const RigidBody<double>& parent,
                  const RigidBody<double>& child) {
  DRAKE_DEMAND(model != nullptr);
  model->AddJoint<WeldJoint>(name, parent, std::nullopt, child, std::nullopt,
                             math::RigidTransformd::Identity());
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() throws an exception
// if a sole composite rigid body can translate and has zero mass.
GTEST_TEST(WeldedBodies, ThrowErrorForDistalCompositeBodyWithZeroMass) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 0;  // Mass of link A or B (both zero).
  const double length = 3;  // Length of uniform-density link (arbitrary ≥ 0).
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a prismatic joint between the world body and bodyA.
  AddPrismaticJointX(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB.
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // We are done building the test model.
  model.Finalize();

  // The next function is usually called from MultibodyPlant::Finalize().
  const std::string expected_message =
      "Body bodyA is the active body for a terminal assembly that "
      "is massless, but its joint has a translational degree of freedom.";
  DRAKE_EXPECT_THROWS_MESSAGE(model.ThrowDefaultMassInertiaError(),
                              expected_message);
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() throws an exception
// if a sole composite rigid body can rotate but has zero inertia.
GTEST_TEST(WeldedBodies, ThrowErrorForDistalCompositeBodyWithZeroInertia) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 0;  // Mass of link A or B (both zero).
  const double length = 3;  // Length of uniform-density link (arbitrary ≥ 0).
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a revolute joint between the world body and bodyA.
  AddRevoluteJointZ(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB.
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // We are done building the test model.
  model.Finalize();

  // The next function is usually called from MultibodyPlant::Finalize().
  const std::string expected_message =
      "Body bodyA is the active body for a terminal assembly that "
      "has zero rotational inertia, but its joint has a rotational degree "
      "of freedom.";
  DRAKE_EXPECT_THROWS_MESSAGE(model.ThrowDefaultMassInertiaError(),
                              expected_message);
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() throws an exception
// if a sole composite rigid body can rotate but has NaN inertia.
GTEST_TEST(WeldedBodies, ThrowErrorForDistalCompositeBodyWithNaNInertia) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mass = std::numeric_limits<double>::quiet_NaN();
  const double length = 3;  // Length of uniform-density link (arbitrary ≥ 0).
  const bool skip_validity_check = true;
  const RigidBody<double>& body_A =
      AddCubicalLink(&model, "bodyA", mass, length, skip_validity_check);
  const RigidBody<double>& body_B =
      AddCubicalLink(&model, "bodyB", mass, length, skip_validity_check);

  // Add a revolute joint between the world body and bodyA.
  AddRevoluteJointZ(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB.
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // We are done building the test model.
  model.Finalize();

  // The next function is usually called from MultibodyPlant::Finalize().
  const std::string expected_message =
      "Body bodyA is the active body for a terminal assembly that "
      "has a NaN rotational inertia, but its joint has a rotational degree "
      "of freedom.";
  DRAKE_EXPECT_THROWS_MESSAGE(model.ThrowDefaultMassInertiaError(),
                              expected_message);
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() does not throw an
// exception if a sole composite rigid body has non-zero mass (due to a weld).
GTEST_TEST(WeldedBodies, NoErrorIfCompositeBodyHasMassDueToWeldedBody) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 1;  // Mass of link A (zero) and B (arbitrary > 0).
  const double length = 3;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a prismatic joint between the world body and bodyA (bodyA has mass 0).
  AddPrismaticJointX(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB (bodyB has mass 1).
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // Signal that we are done building the test model.
  model.Finalize();

  // No exception should be thrown due to default mass/inertia properties.
  EXPECT_NO_THROW(model.ThrowDefaultMassInertiaError());
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() does not throw exception
// if a sole composite rigid body has non-zero inertia (due to a weld).
GTEST_TEST(WeldedBodies, NoErrorIfCompositeBodyHasInertiaDueToWeldedBody) {
  // Create a model and add a few rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 1;  // Mass of link A (zero) or B (arbitrary > 0).
  const double length = 3;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a revolute joint between the world body and bodyA.
  AddRevoluteJointZ(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB (bodyB has non-zero inertia).
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // Signal that we are done building the test model.
  model.Finalize();

  // No exception should be thrown due to default mass/inertia properties.
  EXPECT_NO_THROW(model.ThrowDefaultMassInertiaError());
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() does not throw an
// exception if a zero-mass body is not the most distal body in the tree.
GTEST_TEST(TestDistalBody, NoThrowErrorIfZeroMassBodyIsNotDistal) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 0, mC = 0, mD = 1;  // Mass of links A, B, C, D.
  const double length = 3;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);
  const RigidBody<double>& body_C = AddCubicalLink(&model, "bodyC", mC, length);
  const RigidBody<double>& body_D = AddCubicalLink(&model, "bodyD", mD, length);

  // Add world to bodyA X-prismatic joint (bodyA has zero mass and inertia).
  AddPrismaticJointX(&model, "WA_prismatic_jointX", model.world_body(), body_A);

  // Add bodyA to bodyB Y-prismatic joint (bodyB has zero mass and inertia).
  model.AddJoint<PrismaticJoint>("AB_prismatic_jointY", body_A, {}, body_B, {},
                                 Vector3<double>::UnitY());

  // Add bodyB to bodyC Z-prismatic joint (bodyC has zero mass and inertia).
  model.AddJoint<PrismaticJoint>("BC_prismatic_jointZ", body_B, {}, body_C, {},
                                 Vector3<double>::UnitZ());

  // Add bodyC to bodyD Z-revolute joint (bodyD has non-zero mass and inertia).
  AddRevoluteJointZ(&model, "CD_revolute_joint", body_C, body_D);

  // Signal that we are done building the test model.
  model.Finalize();

  // No exception is thrown due to default mass/inertia properties. It is OK
  // that there are 3 successive prismatic joints that have no mass since their
  // prismatic joints are orthogonal. Alternatively, if two prismatic joints
  // are parallel (with no associated mass), expect numerical problems.
  EXPECT_NO_THROW(model.ThrowDefaultMassInertiaError());
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() does not throw an
// exception if a zero-inertia body is not the most distal body in the tree.
GTEST_TEST(TestDistalBody, NoThrowErrorIfZeroInertiaBodyIsNotDistal) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 1;  // Mass of link A (zero) or B (arbitrary > 0).
  const double length = 3;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a revolute joint from the world body to bodyA (bodyA has no inertia).
  AddRevoluteJointZ(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a revolute joint between bodyA and bodyB (bodyB has non-zero inertia).
  AddRevoluteJointZ(&model, "AB_revolute_joint", body_A, body_B);

  // Signal that we are done building the test model.
  model.Finalize();

  // No exception should be thrown due to default mass/inertia properties.
  EXPECT_NO_THROW(model.ThrowDefaultMassInertiaError());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
