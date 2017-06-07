#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {
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
  EXPECT_EQ(model->get_num_bodies(), 1);

  // Retrieves the world body.
  const Body<double>& world_body = model->get_world_body();

  // Creates a NaN SpatialInertia to instantiate the RigidBody links of the
  // pendulum. Using a NaN spatial inertia is ok so far since we are still
  // not performing any numerical computations. This is only to test API.
  // M_Bo_B is the spatial inertia about the body frame's origin Bo and
  // expressed in the body frame B.
  SpatialInertia<double> M_Bo_B;

  // Adds a new body to the world.
  const RigidBody<double>& pendulum = model->AddBody<RigidBody>(M_Bo_B);

  // Adds a revolute mobilizer.
  EXPECT_NO_THROW((model->AddMobilizer<RevoluteMobilizer>(
      world_body.get_body_frame(), pendulum.get_body_frame(),
      Vector3d::UnitZ())));

  // We cannot add another mobilizer between the same two frames.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      world_body.get_body_frame(), pendulum.get_body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Even if connected in the opposite order.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      pendulum.get_body_frame(), world_body.get_body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Verify we cannot add a mobilizer between a frame and itself.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      pendulum.get_body_frame(), pendulum.get_body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Adds a second pendulum.
  const RigidBody<double>& pendulum2 = model->AddBody<RigidBody>(M_Bo_B);
  model->AddMobilizer<RevoluteMobilizer>(
      model->get_world_frame(), pendulum2.get_body_frame(), Vector3d::UnitZ());

  EXPECT_EQ(model->get_num_bodies(), 3);
  EXPECT_EQ(model->get_num_mobilizers(), 2);

  // Attempts to create a loop. Verify we gen an exception.
  EXPECT_THROW((model->AddMobilizer<RevoluteMobilizer>(
      pendulum.get_body_frame(), pendulum2.get_body_frame(),
      Vector3d::UnitZ())), std::runtime_error);

  // Expect the number of bodies and mobilizers not to change after the above
  // (failed) call.
  EXPECT_EQ(model->get_num_bodies(), 3);
  EXPECT_EQ(model->get_num_mobilizers(), 2);

  // Topology is invalid before MultibodyTree::Finalize().
  EXPECT_FALSE(model->topology_is_valid());
  // Verifies that the topology of this model gets validated at finalize stage.
  EXPECT_NO_THROW(model->Finalize());
  EXPECT_TRUE(model->topology_is_valid());

  // Body identifiers are unique and are assigned by MultibodyTree in increasing
  // order starting with index = 0 (world_index()) for the "world" body.
  EXPECT_EQ(world_body.get_index(), world_index());
  EXPECT_EQ(pendulum.get_index(), BodyIndex(1));
  EXPECT_EQ(pendulum2.get_index(), BodyIndex(2));

  // Tests API to access bodies.
  EXPECT_EQ(model->get_body(BodyIndex(1)).get_index(), pendulum.get_index());
  EXPECT_EQ(model->get_body(BodyIndex(2)).get_index(), pendulum2.get_index());

  // Rigid bodies have no generalized coordinates.
  EXPECT_EQ(pendulum.get_num_flexible_positions(), 0);
  EXPECT_EQ(pendulum.get_num_flexible_velocities(), 0);

  // Verifies that an exception is throw if a call to Finalize() is attempted to
  // an already finalized MultibodyTree.
  EXPECT_THROW(model->Finalize(), std::logic_error);

  // Verifies that after compilation no more bodies can be added.
  EXPECT_THROW(model->AddBody<RigidBody>(M_Bo_B), std::logic_error);
}

// Tests the correctness of MultibodyTreeElement checks to verify one or more
// elements belong to a given MultibodyTree.
GTEST_TEST(MultibodyTree, MultibodyTreeElementChecks) {
  auto model1 = std::make_unique<MultibodyTree<double>>();
  auto model2 = std::make_unique<MultibodyTree<double>>();

  // Creates a NaN SpatialInertia to instantiate the RigidBody links of the
  // pendulum. Using a NaN spatial inertia is ok so far since we are still
  // not performing any numerical computations. This is only to test API.
  // M_Bo_B is the spatial inertia about the body frame's origin Bo and
  // expressed in the body frame B.
  SpatialInertia<double> M_Bo_B;

  const RigidBody<double>& body1 = model1->AddBody<RigidBody>(M_Bo_B);
  const RigidBody<double>& body2 = model2->AddBody<RigidBody>(M_Bo_B);

  // Verifies we can add a mobilizer between body1 and the world of model1.
  const RevoluteMobilizer<double>& pin1 =
      model1->AddMobilizer<RevoluteMobilizer>(
          model1->get_world_body().get_body_frame(), /*inboard frame*/
          body1.get_body_frame() /*outboard frame*/,
          Vector3d::UnitZ() /*axis of rotation*/);

  // Verifies we cannot add a mobilizer between frames that belong to another
  // tree.
  EXPECT_THROW((model1->AddMobilizer<RevoluteMobilizer>(
      model1->get_world_body().get_body_frame(), /*inboard frame*/
      body2.get_body_frame() /*body2 belongs to model2, not model1!!!*/,
      Vector3d::UnitZ() /*axis of rotation*/)), std::logic_error);

  // model1 is complete. Expect no-throw.
  EXPECT_NO_THROW(model1->Finalize());

  // model2->Finalize() is expected to throw an exception since there is no
  // mobilizer for body2.
  try {
    model2->Finalize();
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg =
        "Body with index 1 was not assigned a mobilizer";
    EXPECT_EQ(e.what(), expected_msg);
  }

  // Tests that the created multibody elements indeed do have a parent
  // MultibodyTree.
  EXPECT_NO_THROW(body1.HasParentTreeOrThrow());
  EXPECT_NO_THROW(body2.HasParentTreeOrThrow());
  EXPECT_NO_THROW(pin1.HasParentTreeOrThrow());

  // Tests the check to verify that two bodies belong to the same MultibodyTree.
  EXPECT_THROW(body1.HasSameParentTreeOrThrow(body2), std::logic_error);
  EXPECT_NO_THROW(model1->get_world_body().HasSameParentTreeOrThrow(body1));
  EXPECT_NO_THROW(model2->get_world_body().HasSameParentTreeOrThrow(body2));

  // Verifies bodies have the correct parent tree.
  EXPECT_NO_THROW(body1.HasThisParentTreeOrThrow(model1.get()));
  EXPECT_NO_THROW(body2.HasThisParentTreeOrThrow(model2.get()));
}

// This unit test builds a MultibodyTree as shown in the schematic below, where
// the number inside the boxes corresponds to each of the bodies' indexes (
// assigned by MultibodyTree in the order bodies are created) and "m?" next to
// each connection denotes a mobilizer with the number corresponding to the
// mobilizer index (assigned by MultibodyTree in the order mobilizers are
// created).
// At Finalize(), a body node is created per (body, inboard_mobilizer) pair.
// Body node indexes are assigned according to a BFT order.
// Tests below make reference to the indexes in this schematic. The following
// points are important:
// - Body nodes are ordered by a BFT sort however,
// - There is no guarantee on which body node is assigned to which body,
// - Body nodes at a particular level are not guaranteed to be in any particular
//   order, only that they belong to the right level is important.
//
//                        +---+
//                        | 0 |                        Level 0 (root, world)
//            +-----------+-+-+-----------+
//            |             |             |
//            | m6          | m2          | m4
//            |             |             |
//          +-v-+         +-v-+         +-v-+
//          | 4 |         | 7 |         | 5 |          Level 1
//       +--+---+--+      +---+         +-+-+
//       |         |                      |
//       | m1      | m5                   | m3
//       |         |                      |
//     +-v-+     +-v-+                  +-v-+
//     | 2 |     | 1 |                  | 3 |          Level 2
//     +---+     +-+-+                  +---+
//                 |
//                 | m0
//                 |
//               +-v-+
//               | 6 |                                 Level 3
//               +---+
//
class TreeTopologyTests : public ::testing::Test {
 public:
  // Creates MultibodyTree according to the schematic above.
  void SetUp() override {
    model_ = std::make_unique<MultibodyTree<double>>();

    const int kNumBodies = 8;
    bodies_.push_back(&model_->get_world_body());
    for (int i =1; i < kNumBodies; ++i)
      AddTestBody();

    // Adds mobilizers to connect bodies according to the following diagram:
    ConnectBodies(*bodies_[1], *bodies_[6]);  // mob. 0
    ConnectBodies(*bodies_[4], *bodies_[2]);  // mob. 1
    ConnectBodies(*bodies_[0], *bodies_[7]);  // mob. 2
    ConnectBodies(*bodies_[5], *bodies_[3]);  // mob. 3
    ConnectBodies(*bodies_[0], *bodies_[5]);  // mob. 4
    ConnectBodies(*bodies_[4], *bodies_[1]);  // mob. 5
    ConnectBodies(*bodies_[0], *bodies_[4]);  // mob. 6
  }

  const RigidBody<double>* AddTestBody() {
    // NaN SpatialInertia to instantiate the RigidBody objects.
    // It is safe here since this tests only focus on topological information.
    const SpatialInertia<double> M_Bo_B;
    const RigidBody<double>* body = &model_->AddBody<RigidBody>(M_Bo_B);
    bodies_.push_back(body);
    return body;
  }

  const Mobilizer<double>* ConnectBodies(
      const Body<double>& inboard, const Body<double>& outboard) {
    const Mobilizer<double>* mobilizer =
        &model_->AddMobilizer<RevoluteMobilizer>(
            inboard.get_body_frame(), outboard.get_body_frame(),
            Vector3d::UnitZ());
    mobilizers_.push_back(mobilizer);
    return mobilizer;
  }

  // Performs a number of tests on the BodyNodeTopology corresponding to the
  // body indexed by `body`.
  void TestBodyNode(BodyIndex body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    const BodyNodeIndex node = get_body_topology(body).body_node;

    // Verify that the corresponding Body and BodyNode reference each other
    // correctly.
    EXPECT_EQ(get_body_topology(body).body_node,
              get_body_node_topology(node).index);
    EXPECT_EQ(get_body_node_topology(node).body, get_body_topology(body).index);

    // They should belong to the same level.
    EXPECT_EQ(get_body_topology(body).level,
              get_body_node_topology(node).level);

    const BodyNodeIndex parent_node =
        get_body_node_topology(node).parent_body_node;
    // Either (and thus the exclusive or):
    // 1. `body` is the world, and thus `parent_node` is invalid, XOR
    // 2. `body` is not the world, and thus we have a valid `parent_node`.
    EXPECT_TRUE(parent_node.is_valid() ^ (body == world_index()));

    if (body != world_index()) {
      // Verifies BodyNode has the parent node to the correct body.
      const BodyIndex parent_body = get_body_node_topology(parent_node).body;
      EXPECT_TRUE(parent_body.is_valid());
      EXPECT_EQ(parent_body, get_body_topology(body).parent_body);
      EXPECT_EQ(get_body_node_topology(parent_node).index,
                get_body_topology(parent_body).body_node);

      // Verifies that BodyNode makes reference to the proper mobilizer index.
      const MobilizerIndex mobilizer = get_body_node_topology(node).mobilizer;
      EXPECT_EQ(mobilizer, get_body_topology(body).inboard_mobilizer);

      // Verifies the mobilizer makes reference to the appropriate node.
      EXPECT_EQ(topology.get_mobilizer(mobilizer).body_node, node);

      // Helper lambda to check if this "node" effectively is a child of
      // "parent_node".
      auto is_child_of_parent = [&]() {
        const auto& children = get_body_node_topology(parent_node).child_nodes;
        return
            std::find(children.begin(), children.end(), node) != children.end();
      };
      EXPECT_TRUE(is_child_of_parent());
    }
  }

  const BodyTopology& get_body_topology(int body_index) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    return topology.get_body(BodyIndex(body_index));
  }

  const BodyNodeTopology& get_body_node_topology(int body_node_index) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    return topology.get_body_node(BodyNodeIndex(body_node_index));
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Bodies:
  std::vector<const Body<double>*> bodies_;
  // Mobilizers:
  std::vector<const Mobilizer<double>*> mobilizers_;
};

// This unit tests verifies that the multibody topology is properly compiled.
TEST_F(TreeTopologyTests, Finalize) {
  model_->Finalize();
  EXPECT_EQ(model_->get_num_bodies(), 8);
  EXPECT_EQ(model_->get_num_mobilizers(), 7);

  const MultibodyTreeTopology& topology = model_->get_topology();
  EXPECT_EQ(topology.get_num_body_nodes(), model_->get_num_bodies());
  EXPECT_EQ(topology.get_tree_height(), 4);

  // These sets contain the indexes of the bodies in each tree level.
  // The order of these indexes in each set is not important, but only the fact
  // that they belong to the appropriate set.
  set<BodyIndex> expected_level0 = {BodyIndex(0)};
  set<BodyIndex> expected_level1 = {BodyIndex(4), BodyIndex(7), BodyIndex(5)};
  set<BodyIndex> expected_level2 = {BodyIndex(2), BodyIndex(1), BodyIndex(3)};
  set<BodyIndex> expected_level3 = {BodyIndex(6)};

  set<BodyIndex> level0 = {get_body_node_topology(0).body};
  set<BodyIndex> level1 = {get_body_node_topology(1).body,
                           get_body_node_topology(2).body,
                           get_body_node_topology(3).body};
  set<BodyIndex> level2 = {get_body_node_topology(4).body,
                           get_body_node_topology(5).body,
                           get_body_node_topology(6).body};
  set<BodyIndex> level3 = {get_body_node_topology(7).body};

  // Comparison of sets. The order of the elements is not important.
  EXPECT_EQ(level0, expected_level0);
  EXPECT_EQ(level1, expected_level1);
  EXPECT_EQ(level2, expected_level2);
  EXPECT_EQ(level3, expected_level3);

  // Verifies the expected number of child nodes.
  EXPECT_EQ(get_body_node_topology(0).get_num_children(), 3);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(4).body_node).get_num_children(), 2);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(7).body_node).get_num_children(), 0);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(5).body_node).get_num_children(), 1);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(2).body_node).get_num_children(), 0);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(1).body_node).get_num_children(), 1);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(3).body_node).get_num_children(), 0);
  EXPECT_EQ(get_body_node_topology(
      get_body_topology(6).body_node).get_num_children(), 0);

  // Checks the correctness of each BodyNode associated with a body.
  for (BodyIndex body(0); body < model_->get_num_bodies(); ++body) {
    TestBodyNode(body);
  }
}

// This unit tests verifies the correct number of generalized positions and
// velocities as well as the start indexes into the state vector.
TEST_F(TreeTopologyTests, SizesAndIndexing) {
  model_->Finalize();
  EXPECT_EQ(model_->get_num_bodies(), 8);
  EXPECT_EQ(model_->get_num_mobilizers(), 7);

  const MultibodyTreeTopology& topology = model_->get_topology();
  EXPECT_EQ(topology.get_num_body_nodes(), model_->get_num_bodies());
  EXPECT_EQ(topology.get_tree_height(), 4);

  // Verifies the total number of generalized positions and velocities.
  EXPECT_EQ(topology.get_num_positions(), 7);
  EXPECT_EQ(topology.get_num_velocities(), 7);
  EXPECT_EQ(topology.get_num_states(), 14);

  // Tip-to-Base recursion.
  // In this case all mobilizers are RevoluteMobilizer objects with one
  // generalized position and one generalized velocity per mobilizer.
  int positions_index = 0;
  int velocities_index = topology.get_num_positions();
  for (BodyNodeIndex node_index(1); /* Skips the world node. */
       node_index < topology.get_num_body_nodes(); ++node_index) {
    const BodyNodeTopology& node = topology.get_body_node(node_index);
    const BodyIndex body_index = node.body;
    const MobilizerIndex mobilizer_index = node.mobilizer;

    const MobilizerTopology& mobilizer_topology =
        mobilizers_[mobilizer_index]->get_topology();

    EXPECT_EQ(body_index, bodies_[body_index]->get_index());
    EXPECT_EQ(mobilizer_index, mobilizers_[mobilizer_index]->get_index());

    // Verify positions index.
    EXPECT_EQ(positions_index, node.mobilizer_positions_start);
    EXPECT_EQ(positions_index, mobilizer_topology.positions_start);

    // For this case we know there is one generalized position per mobilizer.
    positions_index += 1;

    // Verify velocities index.
    EXPECT_EQ(velocities_index, node.mobilizer_velocities_start);
    EXPECT_EQ(velocities_index, mobilizer_topology.velocities_start);

    // For this case we know there is one generalized velocities per mobilizer.
    velocities_index += 1;
  }
  EXPECT_EQ(positions_index, topology.get_num_positions());
  EXPECT_EQ(velocities_index, topology.get_num_states());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
