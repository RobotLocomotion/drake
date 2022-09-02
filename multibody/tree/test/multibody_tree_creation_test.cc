#include "drake/common/test_utilities/expect_no_throw.h"
/* clang-format off to disable clang-format-includes */
#include "drake/multibody/tree/multibody_tree-inl.h"
/* clang-format on */

#include <algorithm>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {

class MultibodyElementTester {
 public:
  MultibodyElementTester() = delete;
  template <template <typename> class ElementType, typename T,
      typename ElementIndexType>
  static bool has_parent_tree(
      const MultibodyElement<ElementType, T, ElementIndexType>& element) {
    return element.has_parent_tree();
  }

  template <template <typename> class ElementType, typename T,
      typename ElementIndexType>
  static const internal::MultibodyTree<T>& get_parent_tree(
      const MultibodyElement<ElementType, T, ElementIndexType>& element) {
    return element.get_parent_tree();
  }
};

// Friend class for accessing Joint<T> protected/private internals.
class JointTester {
 public:
  JointTester() = delete;
  static const internal::RevoluteMobilizer<double>* get_mobilizer(
      const RevoluteJoint<double>& joint) {
    return joint.get_mobilizer();
  }
};

namespace internal {
namespace {

using Eigen::Vector3d;
using std::make_unique;
using std::set;
using std::unique_ptr;

// Tests the basic MultibodyTree API to add bodies and mobilizers.
// Tests we cannot create graph loops.
GTEST_TEST(MultibodyTree, BasicAPIToAddBodiesAndMobilizers) {
  auto model = std::make_unique<MultibodyTree<double>>();

  // Initially there is only one body, the world.
  EXPECT_EQ(model->num_bodies(), 1);

  // Retrieves the world body.
  const Body<double>& world_body = model->world_body();

  // Creates a NaN SpatialInertia to instantiate the RigidBody links of the
  // pendulum. Using a NaN spatial inertia is ok so far since we are still
  // not performing any numerical computations. This is only to test API.
  // M_Bo_B is the spatial inertia about the body frame's origin Bo and
  // expressed in the body frame B.
  SpatialInertia<double> M_Bo_B;

  // Adds a new body to the world.
  const RigidBody<double>& pendulum = model->AddBody<RigidBody>(
      "pendulum", M_Bo_B);

  // Adds a revolute mobilizer.
  DRAKE_EXPECT_NO_THROW((model->AddMobilizer<RevoluteMobilizer>(
      world_body.body_frame(), pendulum.body_frame(), Vector3d::UnitZ())));

  // We cannot add another mobilizer between the same two frames.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      world_body.body_frame(), pendulum.body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Even if connected in the opposite order.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      pendulum.body_frame(), world_body.body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Verify we cannot add a mobilizer between a frame and itself.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      pendulum.body_frame(), pendulum.body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Adds a second pendulum.
  const RigidBody<double>& pendulum2 = model->AddBody<RigidBody>(
      "pendulum2", M_Bo_B);
  model->AddMobilizer<RevoluteMobilizer>(
      model->world_frame(), pendulum2.body_frame(), Vector3d::UnitZ());

  EXPECT_EQ(model->num_bodies(), 3);
  EXPECT_EQ(model->num_mobilizers(), 2);

  // Attempts to create a loop. Verify we gen an exception.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      pendulum.body_frame(), pendulum2.body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Expect the number of bodies and mobilizers not to change after the above
  // (failed) call.
  EXPECT_EQ(model->num_bodies(), 3);
  EXPECT_EQ(model->num_mobilizers(), 2);

  // Topology is invalid before MultibodyTree::Finalize().
  EXPECT_FALSE(model->topology_is_valid());
  // Verifies that the topology of this model gets validated at finalize stage.
  DRAKE_EXPECT_NO_THROW(model->Finalize());
  EXPECT_TRUE(model->topology_is_valid());

  // Body identifiers are unique and are assigned by MultibodyTree in increasing
  // order starting with index = 0 (world_index()) for the "world" body.
  EXPECT_EQ(world_body.index(), world_index());
  EXPECT_EQ(pendulum.index(), BodyIndex(1));
  EXPECT_EQ(pendulum2.index(), BodyIndex(2));

  // Tests API to access bodies.
  EXPECT_EQ(model->get_body(BodyIndex(1)).index(), pendulum.index());
  EXPECT_EQ(model->get_body(BodyIndex(2)).index(), pendulum2.index());

  // Rigid bodies have no generalized coordinates.
  EXPECT_EQ(pendulum.get_num_flexible_positions(), 0);
  EXPECT_EQ(pendulum.get_num_flexible_velocities(), 0);

  // Verifies that an exception is throw if a call to Finalize() is attempted to
  // an already finalized MultibodyTree.
  EXPECT_THROW(model->Finalize(), std::logic_error);

  // Verifies that after compilation no more bodies can be added.
  EXPECT_THROW(model->AddBody<RigidBody>("B", M_Bo_B), std::logic_error);
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
  SpatialInertia<double> M_Bo_B;

  const RigidBody<double>& body1 = model1->AddBody<RigidBody>("body1", M_Bo_B);
  const RigidBody<double>& body2 = model2->AddBody<RigidBody>("body2", M_Bo_B);

  // Verifies we can add a mobilizer between body1 and the world of model1.
  const RevoluteMobilizer<double>& pin1 =
      model1->AddMobilizer<RevoluteMobilizer>(
          model1->world_body().body_frame(), /*inboard frame*/
          body1.body_frame() /*outboard frame*/,
          Vector3d::UnitZ() /*axis of rotation*/);

  // Verifies we cannot add a mobilizer between frames that belong to another
  // tree.
  EXPECT_THROW(
      (model1->AddMobilizer<RevoluteMobilizer>(
          model1->world_body().body_frame(), /*inboard frame*/
          body2.body_frame() /*body2 belongs to model2, not model1!!!*/,
          Vector3d::UnitZ() /*axis of rotation*/)),
      std::logic_error);

  // model1 is complete. Expect no-throw.
  DRAKE_EXPECT_NO_THROW(model1->Finalize());
  // model 1 has a single dof corresponding to the pin joint.
  EXPECT_EQ(model1->num_positions(), 1);
  EXPECT_EQ(model1->num_velocities(), 1);

  // model2->Finalize() is not expected to throw an exception. Since body2 has
  // no joint, MultibodyTree will default it to be free and will assign a free
  // mobilizer to it.
  DRAKE_EXPECT_NO_THROW(model2->Finalize());
  // We now verify the number of dofs corresponds to a quaternion free mobilizer
  // by default.
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

  // Verifies bodies have the correct parent tree.
  EXPECT_EQ(&body1_tree, model1.get());
  EXPECT_EQ(&body2_tree, model2.get());
}

// This unit test builds a MultibodyTree as shown in the schematic below, where
// the number inside the boxes corresponds to each of the bodies' indexes (
// assigned by MultibodyTree in the order bodies are created) and "m?" next to
// each connection denotes a mobilizer with the number corresponding to the
// mobilizer index (assigned by MultibodyTree in the order mobilizers are
// created).
// At Finalize(), a body node is created per (body, inboard_mobilizer) pair.
// Body node indexes, shown in parentheses, are assigned in DFT order. In
// addition, we know that internal::MultibodyTreeTopology assigns body nodes to
// each branch in the order mobilizers are added. For instance, in the schematic
// below, node (1) is created before node (2) because mobilizer m2 was added
// before m4. Similarly, node (7) is created before (8) because mobilizer m1 was
// added before m6.
// Notice that bodies 8 and 9 are anchored to the world. Therefore they do not
// belong to any tree and the full model has three trees, with bases at body 7,
// 5 and 4.
//
//                 ┌───┐
//                 │ 0 │(0)                              Level 0 (root, world)
//                 └─┬─┘
//                   │
//     ┌─────────────┼──────────┬────────────────┐
//     │ m2          │ m4       │ m5             │ m7
//   ┌─┴─┐         ┌─┴─┐      ┌─┴─┐            ┌─┴─┐
//   │ 7 │(1)      │ 5 │(2)   │ 9 │(4)         │ 4 │(6)         Level 1
//   └───┘         └─┬─┘      └─┬─┘            └─┬─┘
//                   │          │                │
//                   │          │          ┌─────┴──────┐
//                   │ m3       │ m8       │ m1         │ m6
//                 ┌─┴─┐      ┌─┴─┐      ┌─┴─┐        ┌─┴─┐
//                 │ 3 │(3)   │ 8 │(5)   │ 2 │(7)     │ 1 │(8)  Level 2
//                 └───┘      └───┘      └───┘        └─┬─┘
//                                                      │ m0
//                                                    ┌─┴─┐
//                                                    │ 6 │(9)  Level 3
//                                                    └───┘
//
class TreeTopologyTests : public ::testing::Test {
 public:
  // Creates MultibodyTree according to the schematic above.
  void SetUp() override {
    model_ = std::make_unique<MultibodyTree<double>>();

    const int kNumBodies = 10;
    bodies_.push_back(&model_->world_body());
    for (int i = 1; i < kNumBodies; ++i)
      AddTestBody(i);

    // Adds mobilizers to connect bodies according to the following diagram:
    ConnectBodies(*bodies_[1], *bodies_[6]);  // mob. 0
    ConnectBodies(*bodies_[4], *bodies_[2]);  // mob. 1
    ConnectBodies(*bodies_[0], *bodies_[7]);  // mob. 2
    ConnectBodies(*bodies_[5], *bodies_[3]);  // mob. 3
    ConnectBodies(*bodies_[0], *bodies_[5]);  // mob. 4
    WeldBodies(*bodies_[0], *bodies_[9]);     // mob. 5
    ConnectBodies(*bodies_[4], *bodies_[1], true);  // mob. 6
    ConnectBodies(*bodies_[0], *bodies_[4]);  // mob. 7
    WeldBodies(*bodies_[9], *bodies_[8]);     // mob. 8
  }

  const RigidBody<double>* AddTestBody(int i) {
    // NaN SpatialInertia to instantiate the RigidBody objects.
    // It is safe here since this tests only focus on topological information.
    const SpatialInertia<double> M_Bo_B;
    const RigidBody<double>* body = &model_->AddBody<RigidBody>(
       fmt::format("TestBody_{}", i), M_Bo_B);
    bodies_.push_back(body);
    return body;
  }

  void ConnectBodies(
      const RigidBody<double>& inboard, const RigidBody<double>& outboard,
      bool use_joint = false) {
    if ( use_joint ) {
      // Just for fun, here we explicitly state that the frame on body
      // "inboard" (frame P) IS the joint frame F, done by passing the empty
      // curly braces {}.
      // We DO want the model to have a frame M on body "outbaord" (frame B)
      // with a pose X_BM = Identity. We therefore pass the identity transform.
      const auto* joint = &model_->AddJoint<RevoluteJoint>(
          "FooJoint", inboard,
          {}, /* Model does not create frame F, and makes F = P.  */
          outboard,
          math::RigidTransformd::Identity(), /* Model creates frame M. */
          Vector3d::UnitZ());
      joints_.push_back(joint);
    } else {
      const Mobilizer<double> *mobilizer =
          &model_->AddMobilizer<RevoluteMobilizer>(
              inboard.body_frame(), outboard.body_frame(),
              Vector3d::UnitZ());
      mobilizers_.push_back(mobilizer);
    }
  }

  void WeldBodies(const RigidBody<double>& inboard,
                  const RigidBody<double>& outboard) {
    const Mobilizer<double>* mobilizer = &model_->AddMobilizer<WeldMobilizer>(
        inboard.body_frame(), outboard.body_frame(),
        math::RigidTransformd::Identity());
    mobilizers_.push_back(mobilizer);
  }

  void FinalizeModel() {
    model_->Finalize();

    // For testing, collect all mobilizer models introduced by Joint objects.
    // These are only available after Finalize().
    for (const RevoluteJoint<double>* joint : joints_) {
      const auto* mobilizer = JointTester::get_mobilizer(*joint);
      mobilizers_.push_back(mobilizer);
    }
  }

  // Performs a number of tests on the BodyNodeTopology corresponding to the
  // body indexed by `body`.
  static void TestBodyNode(const MultibodyTreeTopology& topology,
                           BodyIndex body) {
    const BodyNodeIndex node = topology.get_body(body).body_node;

    // Verify that the corresponding Body and BodyNode reference each other
    // correctly.
    EXPECT_EQ(topology.get_body(body).body_node,
              topology.get_body_node(node).index);
    EXPECT_EQ(topology.get_body_node(node).body, topology.get_body(body).index);

    // They should belong to the same level.
    EXPECT_EQ(topology.get_body(body).level,
              topology.get_body_node(node).level);

    const BodyNodeIndex parent_node =
        topology.get_body_node(node).parent_body_node;
    // Either (and thus the exclusive or):
    // 1. `body` is the world, and thus `parent_node` is invalid, XOR
    // 2. `body` is not the world, and thus we have a valid `parent_node`.
    EXPECT_TRUE(parent_node.is_valid() ^ (body == world_index()));

    if (body != world_index()) {
      // Verifies BodyNode has the parent node to the correct body.
      const BodyIndex parent_body = topology.get_body_node(parent_node).body;
      EXPECT_TRUE(parent_body.is_valid());
      EXPECT_EQ(parent_body, topology.get_body(body).parent_body);
      EXPECT_EQ(topology.get_body_node(parent_node).index,
                topology.get_body(parent_body).body_node);

      // Verifies that BodyNode makes reference to the proper mobilizer index.
      const MobilizerIndex mobilizer = topology.get_body_node(node).mobilizer;
      EXPECT_EQ(mobilizer, topology.get_body(body).inboard_mobilizer);

      // Verifies the mobilizer makes reference to the appropriate node.
      EXPECT_EQ(topology.get_mobilizer(mobilizer).body_node, node);

      // Helper lambda to check if this "node" effectively is a child of
      // "parent_node".
      auto is_child_of_parent = [&]() {
        const auto& children = topology.get_body_node(parent_node).child_nodes;
        return
            std::find(children.begin(), children.end(), node) != children.end();
      };
      EXPECT_TRUE(is_child_of_parent());
    }
  }

  static const BodyNodeTopology& node_topology_from_body_index(
      const MultibodyTreeTopology& topology, int body_index) {
    return topology.get_body_node(
        topology.get_body(BodyIndex(body_index)).body_node);
  }

  static void VerifyTopology(const MultibodyTreeTopology& topology) {
    const int kNumBodies = 10;

    EXPECT_EQ(topology.num_bodies(), kNumBodies);
    EXPECT_EQ(topology.num_mobilizers(), 9);
    EXPECT_EQ(topology.num_force_elements(), 1);
    EXPECT_EQ(topology.get_num_body_nodes(), kNumBodies);
    EXPECT_EQ(topology.tree_height(), 4);

    // These sets contain the indexes of the bodies in each tree level.
    // The order of these indexes in each set is not important, but only the
    // fact that they belong to the appropriate set.
    set<BodyIndex> expected_level0 = {BodyIndex(0)};
    set<BodyIndex> expected_level1 = {BodyIndex(4), BodyIndex(7), BodyIndex(5),
                                      BodyIndex(9)};
    set<BodyIndex> expected_level2 = {BodyIndex(2), BodyIndex(1), BodyIndex(3),
                                      BodyIndex(8)};
    set<BodyIndex> expected_level3 = {BodyIndex(6)};

    std::vector<std::set<BodyIndex>> levels(topology.num_bodies());
    for (BodyIndex b(0); b < topology.num_bodies(); ++b) {
      const BodyTopology& body = topology.get_body(b);
      levels[body.level].insert(b);
    }

    // Comparison of sets. The order of the elements is not important.
    EXPECT_EQ(levels[0], expected_level0);
    EXPECT_EQ(levels[1], expected_level1);
    EXPECT_EQ(levels[2], expected_level2);
    EXPECT_EQ(levels[3], expected_level3);

    // Verifies the expected number of child nodes.
    EXPECT_EQ(node_topology_from_body_index(topology, 0).get_num_children(), 4);
    EXPECT_EQ(node_topology_from_body_index(topology, 4).get_num_children(), 2);
    EXPECT_EQ(node_topology_from_body_index(topology, 7).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 5).get_num_children(), 1);
    EXPECT_EQ(node_topology_from_body_index(topology, 9).get_num_children(), 1);
    EXPECT_EQ(node_topology_from_body_index(topology, 2).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 1).get_num_children(), 1);
    EXPECT_EQ(node_topology_from_body_index(topology, 3).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 8).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 6).get_num_children(), 0);

    // Checks the correctness of each BodyNode associated with a body.
    for (BodyIndex body(0); body < kNumBodies; ++body) {
      TestBodyNode(topology, body);
    }

    // We now verify the precise expected topology. We use our internal
    // knowledge that branches are created according to the order in which
    // mobilizers are added. Refer to schematic in the documentation of this
    // test fixture.
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(0)).body, 0);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(1)).body, 7);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(2)).body, 5);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(3)).body, 3);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(4)).body, 9);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(5)).body, 8);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(6)).body, 4);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(7)).body, 2);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(8)).body, 1);
    EXPECT_EQ(topology.get_body_node(BodyNodeIndex(9)).body, 6);

    // Verify the expected "forest" of trees.
    EXPECT_EQ(topology.num_trees(), 3);
    EXPECT_EQ(topology.num_tree_velocities(TreeIndex(0)), 1);
    EXPECT_EQ(topology.num_tree_velocities(TreeIndex(1)), 2);
    EXPECT_EQ(topology.num_tree_velocities(TreeIndex(2)), 4);
    EXPECT_EQ(topology.tree_velocities_start(TreeIndex(0)), 0);
    EXPECT_EQ(topology.tree_velocities_start(TreeIndex(1)), 1);
    EXPECT_EQ(topology.tree_velocities_start(TreeIndex(2)), 3);
    // The world body does not belong to a tree. Therefore the returned index is
    // invalid.
    EXPECT_FALSE(topology.body_to_tree_index(world_index()).is_valid());
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(7)), TreeIndex(0));
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(5)), TreeIndex(1));
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(3)), TreeIndex(1));
    EXPECT_FALSE(topology.body_to_tree_index(BodyIndex(9)).is_valid());
    EXPECT_FALSE(topology.body_to_tree_index(BodyIndex(8)).is_valid());
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(4)), TreeIndex(2));
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(2)), TreeIndex(2));
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(1)), TreeIndex(2));
    EXPECT_EQ(topology.body_to_tree_index(BodyIndex(6)), TreeIndex(2));

    EXPECT_EQ(topology.num_velocities(), 7);
    EXPECT_EQ(topology.velocity_to_tree_index(0), TreeIndex(0));
    EXPECT_EQ(topology.velocity_to_tree_index(1), TreeIndex(1));
    EXPECT_EQ(topology.velocity_to_tree_index(2), TreeIndex(1));
    EXPECT_EQ(topology.velocity_to_tree_index(3), TreeIndex(2));
    EXPECT_EQ(topology.velocity_to_tree_index(4), TreeIndex(2));
    EXPECT_EQ(topology.velocity_to_tree_index(5), TreeIndex(2));
    EXPECT_EQ(topology.velocity_to_tree_index(6), TreeIndex(2));
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Bodies:
  std::vector<const RigidBody<double>*> bodies_;
  // Mobilizers:
  std::vector<const Mobilizer<double>*> mobilizers_;
  // Joints:
  std::vector<const RevoluteJoint<double>*> joints_;
};

// This unit tests verifies that the multibody topology is properly compiled.
TEST_F(TreeTopologyTests, Finalize) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 9);

  const MultibodyTreeTopology& topology = model_->get_topology();
  EXPECT_EQ(topology.get_num_body_nodes(), model_->num_bodies());
  EXPECT_EQ(topology.tree_height(), 4);

  VerifyTopology(topology);
}

// This unit tests verifies the correct number of generalized positions and
// velocities as well as the start indexes into the state vector.
TEST_F(TreeTopologyTests, SizesAndIndexing) {
  FinalizeModel();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 9);
  EXPECT_EQ(model_->num_joints(), 1);

  const MultibodyTreeTopology& topology = model_->get_topology();
  EXPECT_EQ(topology.get_num_body_nodes(), model_->num_bodies());
  EXPECT_EQ(topology.tree_height(), 4);

  // Verifies the total number of generalized positions and velocities.
  EXPECT_EQ(topology.num_positions(), 7);
  EXPECT_EQ(topology.num_velocities(), 7);
  EXPECT_EQ(topology.num_states(), 14);

  // Tip-to-Base recursion.
  // In this case all mobilizers are RevoluteMobilizer objects with one
  // generalized position and one generalized velocity per mobilizer.
  int positions_index = 0;
  int velocities_index = topology.num_positions();
  for (BodyNodeIndex node_index(1); /* Skips the world node. */
       node_index < topology.get_num_body_nodes(); ++node_index) {
    const BodyNodeTopology& node = topology.get_body_node(node_index);
    const BodyIndex body_index = node.body;
    const MobilizerIndex mobilizer_index = node.mobilizer;

    const MobilizerTopology& mobilizer_topology =
        topology.get_mobilizer(mobilizer_index);

    EXPECT_EQ(body_index, bodies_[body_index]->index());
    EXPECT_EQ(mobilizer_index, mobilizers_[mobilizer_index]->index());

    // Verify positions and velocity indexes.
    if (mobilizer_topology.num_velocities == 0) {
      // MultibodyTreeTopology sets start indexes to zero when there are no
      // mobilities associated to a node.
      EXPECT_EQ(node.mobilizer_positions_start, 0);
      EXPECT_EQ(mobilizer_topology.positions_start, 0);
      EXPECT_EQ(node.mobilizer_velocities_start, 0);
      EXPECT_EQ(mobilizer_topology.velocities_start, 0);
    } else {
      EXPECT_EQ(positions_index, node.mobilizer_positions_start);
      EXPECT_EQ(positions_index, mobilizer_topology.positions_start);
      EXPECT_EQ(velocities_index, node.mobilizer_velocities_start);
      EXPECT_EQ(velocities_index, mobilizer_topology.velocities_start);

      // Mobilizers 5 and 8 are weld joints, with no velocities. All other
      // mobilizers introduce one position and one velocity.
      positions_index += 1;
      velocities_index += 1;
    }
  }
  EXPECT_EQ(positions_index, topology.num_positions());
  EXPECT_EQ(velocities_index, topology.num_states());
}

// Verifies that the clone of a given MultibodyTree model created with
// MultibodyTree::Clone() has exactly the same topology as the original
// model.
TEST_F(TreeTopologyTests, Clone) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 9);
  EXPECT_EQ(model_->num_force_elements(), 1);
  const MultibodyTreeTopology& topology = model_->get_topology();

  auto cloned_model = model_->Clone();
  EXPECT_EQ(cloned_model->num_bodies(), 10);
  EXPECT_EQ(cloned_model->num_mobilizers(), 9);
  EXPECT_EQ(cloned_model->num_force_elements(), 1);
  const MultibodyTreeTopology& clone_topology = cloned_model->get_topology();

  // Verify the cloned topology actually is a different object.
  ASSERT_NE(&topology, &clone_topology);

  // The topology of the clone must be exactly equal to the topology of the
  // original MultibodyTree.
  EXPECT_EQ(topology, clone_topology);

  // Even though the test above confirms the two topologies are exactly equal,
  // we perform a number of additional tests.
  VerifyTopology(clone_topology);
}

// Verifies that the AutoDiffXd version of a given MultibodyTree model created
// with MultibodyTree::ToAutoDiffXd() has exactly the same topology as the
// original model.
TEST_F(TreeTopologyTests, ToAutoDiffXd) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 9);
  const MultibodyTreeTopology& topology = model_->get_topology();

  auto autodiff_model = model_->ToAutoDiffXd();
  EXPECT_EQ(autodiff_model->num_bodies(), 10);
  EXPECT_EQ(autodiff_model->num_mobilizers(), 9);
  const MultibodyTreeTopology& autodiff_topology =
      autodiff_model->get_topology();

  // The topology of the clone must be exactly equal to the topology of the
  // original MultibodyTree.
  EXPECT_EQ(topology, autodiff_topology);

  // Even though the test above confirms the two topologies are exactly equal,
  // we perform a number of additional tests.
  VerifyTopology(autodiff_topology);
}

// Verifies that the symbolic version of a given MultibodyTree model created
// with MultibodyTree::ToSymbolic() has exactly the same topology as the
// original model.
TEST_F(TreeTopologyTests, ToSymbolic) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 10);
  EXPECT_EQ(model_->num_mobilizers(), 9);
  const MultibodyTreeTopology& topology = model_->get_topology();

  auto symbolic_model = model_->CloneToScalar<symbolic::Expression>();
  EXPECT_EQ(symbolic_model->num_bodies(), 10);
  EXPECT_EQ(symbolic_model->num_mobilizers(), 9);
  const MultibodyTreeTopology& symbolic_topology =
      symbolic_model->get_topology();

  // The topology of the clone must be exactly equal to the topology of the
  // original MultibodyTree.
  EXPECT_EQ(topology, symbolic_topology);

  // Even though the test above confirms the two topologies are exactly equal,
  // we perform a number of additional tests.
  VerifyTopology(symbolic_topology);
}

// Verifies the correctness of the method
// MultibodyTreeTopology::GetKinematicPathToWorld() by computing the path from a
// given body to the world (root) of the tree on a known topology.
TEST_F(TreeTopologyTests, KinematicPathToWorld) {
  FinalizeModel();
  const MultibodyTreeTopology& topology = model_->get_topology();

  const BodyIndex body6_index(6);
  const BodyNodeIndex body6_node_index =
      topology.get_body(body6_index).body_node;
  const BodyNodeTopology& body6_node =
      topology.get_body_node(body6_node_index);

  // Compute kinematic path from body 6 to the world. See documentation for the
  // test fixture TreeTopologyTests for details on the topology under test.
  std::vector<BodyNodeIndex> path_to_world;
  topology.GetKinematicPathToWorld(body6_node_index, &path_to_world);

  const int expected_path_size = body6_node.level + 1;
  EXPECT_EQ(static_cast<int>(path_to_world.size()), expected_path_size);

  // These are the expected bodies in the path.
  const std::vector<BodyIndex> expected_bodies_path =
      {BodyIndex(0), BodyIndex(4), BodyIndex(1), BodyIndex(6)};

  for (BodyIndex body_index : expected_bodies_path) {
    // The path is computed in terms of body node indexes. Therefore obtain
    // the expected value of the node index from the expected value of the
    // body index.
    const BodyNodeIndex expected_node_index =
        topology.get_body(body_index).body_node;
    const BodyNodeTopology& node = topology.get_body_node(expected_node_index);
    // Both, expected and computed nodes must be at the same level (depth) in
    // the tree.
    const int level = node.level;
    // Verify the
    EXPECT_EQ(path_to_world[level], expected_node_index);
  }
}

// Unit test to verify the correctness of
// MultibodyTreeTopology::CreateListOfWeldedBodies().
// This test creates a tree with a topology as shown below. Single vertical
// lines indicate bodies that are connected by a non-weld mobilizer. Double
// vertical lines indicate bodies that are welded.
// A "welded body" is defined as some connected sub-graph of the tree where the
// connected edges (i.e., mobilizers) are weld mobilizers (double vertical lines
// in the schematic below). According to this definition, welded bodies for the
// tree below are (see MultibodyTreeTopology::CreateListOfWeldedBodies() for
// further details on this definition):
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
// This test verifies MultibodyTreeTopology::CreateListOfWeldedBodies() returns
// this list of welded bodies.
GTEST_TEST(WeldedBodies, CreateListOfWeldedBodies) {
  MultibodyTree<double> model;

  // Helper to add a rigid body. For these tests the value of the spatial
  // inertia is not relevant and therefore we leave it un-initialized.
  auto AddRigidBody =
      [&model](const std::string& name) -> const RigidBody<double>& {
    return model.AddRigidBody(name, SpatialInertia<double>());
  };

  // Helper to add a joint between two bodies. For this test the actual type of
  // the joint is not relevant, only the fact that "is not" a WeldJoint.
  auto AddJoint =
      [&model](const std::string& name,
               const Body<double>& parent, const Body<double>& child) {
    model.AddJoint<RevoluteJoint>(
        name, parent, {}, child, {}, Vector3<double>::UnitX());
  };

  // Helper method to add a WeldJoint between two bodies.
  auto AddWeldJoint = [&model](const std::string& name,
                               const Body<double>& parent,
                               const Body<double>& child) {
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

  const MultibodyTreeTopology& topology = model.get_topology();

  // Ask the topology to build the list of welded bodies.
  std::vector<std::set<BodyIndex>> welded_bodies =
      topology.CreateListOfWeldedBodies();
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
  std::set<std::set<BodyIndex>> welded_bodies_set(
      welded_bodies.begin(), welded_bodies.end());

  // Verify the computed list has the expected entries.
  EXPECT_EQ(welded_bodies_set, expected_welded_bodies);

  // All bodies in welded_bodies[0] are, by definition, anchored to the world.
  // We verify this with IsBodyAnchored().
  for (size_t welded_body_index = 0;
       welded_body_index < welded_bodies.size(); ++welded_body_index) {
    const std::set<BodyIndex>& welded_body = welded_bodies[welded_body_index];
    // All bodies in welded_bodies[0] are, by definition, anchored to the world.
    // We verify this with IsBodyAnchored().
    for (BodyIndex body_index : welded_body) {
        EXPECT_EQ(
            topology.IsBodyAnchored(body_index),
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
// @param[in] model the MultibodyTree to which body B is added.
// @param[in] name of the body that is being added to the model.
// @param[in] link_length the length, width, and depth of the cube-shaped body.
// @param[in] skip_validity_check is passed in as `true` to skip the validity
//  check on the new body B's spatial inertia, which ensures an exception is not
//  thrown when setting body B's spatial inertia (which would otherwise occur if
//  mass or link_length is NaN). Avoiding this early exception allows for a
//  later exception to be thrown in a subsequent function and tested below.
const RigidBody<double>& AddCubicalLink(
    MultibodyTree<double>* model,
    const std::string& name,
    const double mass,
    const double link_length = 1.0,
    const bool skip_validity_check = false) {
  DRAKE_DEMAND(model != nullptr);
  const Vector3<double> p_BoBcm_B(link_length / 2, 0, 0);
  const UnitInertia<double> G_BBo_B = MakeTestCubeUnitInertia(link_length);
  const SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B,
                                       skip_validity_check);
  return model->AddRigidBody(name, M_BBo_B);
}

// Verify Body::default_rotational_inertia() and related MultibodyTree methods.
GTEST_TEST(DefaultInertia, VerifyDefaultRotationalInertia) {
  // Create a model and add trhee rigid bodies, namely A, B, C.
  MultibodyTree<double> model;
  const double mA = 0, mB = 1, mC = 3;  // Mass of links A, B, C.
  const double length = 3;         // Length of each thin uniform-density link.
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
  std::set<BodyIndex> bodies_AA, bodies_AB, bodies_BC, bodies_ABC;
  bodies_AA.insert({body_A.index(), body_A.index()});
  bodies_AB.insert({body_A.index(), body_B.index()});
  bodies_BC.insert({body_B.index(), body_C.index()});
  bodies_ABC.insert({body_A.index(), body_B.index(), body_C.index()});

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
               const Body<double>& parent, const Body<double>& child) {
    DRAKE_DEMAND(model != nullptr);
    model->AddJoint<PrismaticJoint>(name, parent, {}, child, {},
        Vector3<double>::UnitX());
}

// Helper function to add a z-axis revolute joint between two bodies.
void AddRevoluteJointZ(MultibodyTree<double>* model, const std::string& name,
               const Body<double>& parent, const Body<double>& child) {
    DRAKE_DEMAND(model != nullptr);
    model->AddJoint<RevoluteJoint>(name, parent, {}, child, {},
        Vector3<double>::UnitZ());
}

// Helper function to add a weld joint between two bodies.
void AddWeldJoint(MultibodyTree<double>* model, const std::string& name,
               const Body<double>& parent, const Body<double>& child) {
    DRAKE_DEMAND(model != nullptr);
    model->AddJoint<WeldJoint>(name, parent, std::nullopt, child, std::nullopt,
                              math::RigidTransformd::Identity());
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() throws an exception
// if a sole composite rigid body can translate and has zero mass.
GTEST_TEST(WeldedBodies, ThrowErrorForDistalCompositeBodyWithZeroMass) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 0;  // Mass of link A or B.
  const double length = 3;  // Length of thin uniform-density link.
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a prismatic joint between the world body and bodyA.
  AddPrismaticJointX(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB.
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // We are done building the test model.
  model.Finalize();

  // The next function is usually called from MultibodyPlant::Finalize().
  const std::string expected_message = "It seems that body bodyA is massless, "
    "yet it is attached by a joint that has a translational degree of freedom.";
  DRAKE_EXPECT_THROWS_MESSAGE(model.ThrowDefaultMassInertiaError(),
      expected_message);
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() throws an exception
// if a sole composite rigid body can rotate but has zero inertia.
GTEST_TEST(WeldedBodies, ThrowErrorForDistalCompositeBodyWithZeroInertia) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 0;  // Mass of link A or B.
  const double length = 0;  // Length of thin uniform-density link.
  const RigidBody<double>& body_A = AddCubicalLink(&model, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&model, "bodyB", mB, length);

  // Add a revolute joint between the world body and bodyA.
  AddRevoluteJointZ(&model, "WA_revolute_joint", model.world_body(), body_A);

  // Add a weld joint between bodyA and bodyB.
  AddWeldJoint(&model, "AB_weld_joint", body_A, body_B);

  // We are done building the test model.
  model.Finalize();

  // The next function is usually called from MultibodyPlant::Finalize().
  const std::string expected_message = "Body bodyA has a zero rotational "
    "inertia, yet it is attached by a joint that "
    "has a rotational degree of freedom.";
  DRAKE_EXPECT_THROWS_MESSAGE(model.ThrowDefaultMassInertiaError(),
      expected_message);
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() throws an exception
// if a sole composite rigid body can rotate but has NaN inertia.
GTEST_TEST(WeldedBodies, ThrowErrorForDistalCompositeBodyWithNaNInertia) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mass = std::numeric_limits<double>::quiet_NaN();
  const double length = 3;  // Length of thin uniform-density link.
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
  const std::string expected_message = "Body bodyA has a NaN rotational "
    "inertia, yet it is attached by a joint that "
    "has a rotational degree of freedom.";
  DRAKE_EXPECT_THROWS_MESSAGE(model.ThrowDefaultMassInertiaError(),
      expected_message);
}

// Verify MultibodyTree::ThrowDefaultMassInertiaError() does not throw an
// exception if a sole composite rigid body has non-zero mass (due to a weld).
GTEST_TEST(WeldedBodies, NoErrorIfCompositeBodyHasMassDueToWeldedBody) {
  // Create a model and add two rigid bodies.
  MultibodyTree<double> model;
  const double mA = 0, mB = 1;  // Mass of link A or B.
  const double length = 3;  // Length of thin uniform-density link.
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
  const double mA = 0, mB = 1;  // Mass of link A or B.
  const double length = 3;  // Length of thin uniform-density link.
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
  const double length = 3;  // Length of thin uniform-density link.
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
  const double mA = 0, mB = 1;  // Mass of link A or B.
  const double length = 3;  // Length of thin uniform-density link.
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
