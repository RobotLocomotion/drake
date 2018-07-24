/* clang-format off to disable clang-format-includes */
#include "drake/multibody/multibody_tree/multibody_tree.h"
/* clang-format on */

#include <algorithm>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {

// Friend class for accessing Joint<T> protected/private internals.
class JointTester {
 public:
  JointTester() = delete;
  static const RevoluteMobilizer<double>* get_mobilizer(
      const RevoluteJoint<double>& joint) {
    return joint.get_mobilizer();
  }
};

namespace {

using Eigen::Isometry3d;
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
  const RigidBody<double>& pendulum = model->AddBody<RigidBody>(M_Bo_B);

  // Adds a revolute mobilizer.
  EXPECT_NO_THROW((model->AddMobilizer<RevoluteMobilizer>(
      world_body.body_frame(), pendulum.body_frame(),
      Vector3d::UnitZ())));

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
  const RigidBody<double>& pendulum2 = model->AddBody<RigidBody>(M_Bo_B);
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
  EXPECT_NO_THROW(model->Finalize());
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
          model1->world_body().body_frame(), /*inboard frame*/
          body1.body_frame() /*outboard frame*/,
          Vector3d::UnitZ() /*axis of rotation*/);

  // Verifies we cannot add a mobilizer between frames that belong to another
  // tree.
  EXPECT_THROW((model1->AddMobilizer<RevoluteMobilizer>(
      model1->world_body().body_frame(), /*inboard frame*/
      body2.body_frame() /*body2 belongs to model2, not model1!!!*/,
      Vector3d::UnitZ() /*axis of rotation*/)), std::logic_error);

  // model1 is complete. Expect no-throw.
  EXPECT_NO_THROW(model1->Finalize());
  // model 1 has a single dof corresponding to the pin joint.
  EXPECT_EQ(model1->num_positions(), 1);
  EXPECT_EQ(model1->num_velocities(), 1);

  // model2->Finalize() is not expected to throw an exception. Since body2 has
  // no joint, MultibodyTree will default it to be free and will assign a free
  // mobilizer to it.
  EXPECT_NO_THROW(model2->Finalize());
  // We now verify the number of dofs corresponds to a quaternion free mobilizer
  // by default.
  EXPECT_EQ(model2->num_positions(), 7);
  EXPECT_EQ(model2->num_velocities(), 6);

  // Tests that the created multibody elements indeed do have a parent
  // MultibodyTree.
  EXPECT_NO_THROW(body1.HasParentTreeOrThrow());
  EXPECT_NO_THROW(body2.HasParentTreeOrThrow());
  EXPECT_NO_THROW(pin1.HasParentTreeOrThrow());

  // Tests the check to verify that two bodies belong to the same MultibodyTree.
  EXPECT_THROW(body1.HasSameParentTreeOrThrow(body2), std::logic_error);
  EXPECT_NO_THROW(model1->world_body().HasSameParentTreeOrThrow(body1));
  EXPECT_NO_THROW(model2->world_body().HasSameParentTreeOrThrow(body2));

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
    bodies_.push_back(&model_->world_body());
    for (int i =1; i < kNumBodies; ++i)
      AddTestBody();

    // Adds mobilizers to connect bodies according to the following diagram:
    ConnectBodies(*bodies_[1], *bodies_[6]);  // mob. 0
    ConnectBodies(*bodies_[4], *bodies_[2]);  // mob. 1
    ConnectBodies(*bodies_[0], *bodies_[7]);  // mob. 2
    ConnectBodies(*bodies_[5], *bodies_[3]);  // mob. 3
    ConnectBodies(*bodies_[0], *bodies_[5]);  // mob. 4
    ConnectBodies(*bodies_[4], *bodies_[1]);  // mob. 5
    ConnectBodies(*bodies_[0], *bodies_[4], true);  // mob. 6

    // Adds a force element for a uniform gravity field.
    model_->AddForceElement<UniformGravityFieldElement>(g_);
  }

  const RigidBody<double>* AddTestBody() {
    // NaN SpatialInertia to instantiate the RigidBody objects.
    // It is safe here since this tests only focus on topological information.
    const SpatialInertia<double> M_Bo_B;
    const RigidBody<double>* body = &model_->AddBody<RigidBody>(M_Bo_B);
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
          "FooJoint",
          inboard, {}, /* Model does not create frame F, and makes F = P.  */
          outboard, Isometry3d::Identity(), /* Model does creates frame M. */
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
    const int kNumBodies = 8;

    EXPECT_EQ(topology.num_bodies(), kNumBodies);
    EXPECT_EQ(topology.num_mobilizers(), 7);
    EXPECT_EQ(topology.num_force_elements(), 1);
    EXPECT_EQ(topology.get_num_body_nodes(), kNumBodies);
    EXPECT_EQ(topology.tree_height(), 4);

    // These sets contain the indexes of the bodies in each tree level.
    // The order of these indexes in each set is not important, but only the
    // fact that they belong to the appropriate set.
    set<BodyIndex> expected_level0 = {BodyIndex(0)};
    set<BodyIndex> expected_level1 = {BodyIndex(4), BodyIndex(7), BodyIndex(5)};
    set<BodyIndex> expected_level2 = {BodyIndex(2), BodyIndex(1), BodyIndex(3)};
    set<BodyIndex> expected_level3 = {BodyIndex(6)};

    set<BodyIndex> level0 = {topology.get_body_node(BodyNodeIndex(0)).body};
    set<BodyIndex> level1 = {topology.get_body_node(BodyNodeIndex(1)).body,
                             topology.get_body_node(BodyNodeIndex(2)).body,
                             topology.get_body_node(BodyNodeIndex(3)).body};
    set<BodyIndex> level2 = {topology.get_body_node(BodyNodeIndex(4)).body,
                             topology.get_body_node(BodyNodeIndex(5)).body,
                             topology.get_body_node(BodyNodeIndex(6)).body};
    set<BodyIndex> level3 = {topology.get_body_node(BodyNodeIndex(7)).body};

    // Comparison of sets. The order of the elements is not important.
    EXPECT_EQ(level0, expected_level0);
    EXPECT_EQ(level1, expected_level1);
    EXPECT_EQ(level2, expected_level2);
    EXPECT_EQ(level3, expected_level3);

    // Verifies the expected number of child nodes.
    EXPECT_EQ(node_topology_from_body_index(topology, 0).get_num_children(), 3);
    EXPECT_EQ(node_topology_from_body_index(topology, 4).get_num_children(), 2);
    EXPECT_EQ(node_topology_from_body_index(topology, 7).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 5).get_num_children(), 1);
    EXPECT_EQ(node_topology_from_body_index(topology, 2).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 1).get_num_children(), 1);
    EXPECT_EQ(node_topology_from_body_index(topology, 3).get_num_children(), 0);
    EXPECT_EQ(node_topology_from_body_index(topology, 6).get_num_children(), 0);

    // Checks the correctness of each BodyNode associated with a body.
    for (BodyIndex body(0); body < kNumBodies; ++body) {
      TestBodyNode(topology, body);
    }
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Bodies:
  std::vector<const RigidBody<double>*> bodies_;
  // Mobilizers:
  std::vector<const Mobilizer<double>*> mobilizers_;
  // Joints:
  std::vector<const RevoluteJoint<double>*> joints_;
  // The acceleration of gravity vector.
  Vector3d g_{0.0, 0.0, -9.81};
};

// This unit tests verifies that the multibody topology is properly compiled.
TEST_F(TreeTopologyTests, Finalize) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 8);
  EXPECT_EQ(model_->num_mobilizers(), 7);

  const MultibodyTreeTopology& topology = model_->get_topology();
  EXPECT_EQ(topology.get_num_body_nodes(), model_->num_bodies());
  EXPECT_EQ(topology.tree_height(), 4);

  VerifyTopology(topology);
}

// This unit tests verifies the correct number of generalized positions and
// velocities as well as the start indexes into the state vector.
TEST_F(TreeTopologyTests, SizesAndIndexing) {
  FinalizeModel();
  EXPECT_EQ(model_->num_bodies(), 8);
  EXPECT_EQ(model_->num_mobilizers(), 7);
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
        mobilizers_[mobilizer_index]->get_topology();

    EXPECT_EQ(body_index, bodies_[body_index]->index());
    EXPECT_EQ(mobilizer_index, mobilizers_[mobilizer_index]->index());

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
  EXPECT_EQ(positions_index, topology.num_positions());
  EXPECT_EQ(velocities_index, topology.num_states());
}

// Verifies that the clone of a given MultibodyTree model created with
// MultibodyTree::Clone() has has exactly the same topology as the original
// model.
TEST_F(TreeTopologyTests, Clone) {
  model_->Finalize();
  EXPECT_EQ(model_->num_bodies(), 8);
  EXPECT_EQ(model_->num_mobilizers(), 7);
  EXPECT_EQ(model_->num_force_elements(), 1);
  const MultibodyTreeTopology& topology = model_->get_topology();

  auto cloned_model = model_->Clone();
  EXPECT_EQ(cloned_model->num_bodies(), 8);
  EXPECT_EQ(cloned_model->num_mobilizers(), 7);
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
  EXPECT_EQ(model_->num_bodies(), 8);
  EXPECT_EQ(model_->num_mobilizers(), 7);
  const MultibodyTreeTopology& topology = model_->get_topology();

  auto autodiff_model = model_->ToAutoDiffXd();
  EXPECT_EQ(autodiff_model->num_bodies(), 8);
  const MultibodyTreeTopology& autodiff_topology =
      autodiff_model->get_topology();

  // The topology of the clone must be exactly equal to the topology of the
  // original MultibodyTree.
  EXPECT_EQ(topology, autodiff_topology);

  // Even though the test above confirms the two topologies are exactly equal,
  // we perform a number of additional tests.
  VerifyTopology(autodiff_topology);
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
  auto AddWeldJoint =
      [&model](const std::string& name,
               const Body<double>& parent, const Body<double>& child) {
        model.AddJoint<WeldJoint>(
            name, parent, {}, child, {}, Isometry3d::Identity());
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
}

}  // namespace
}  // namespace multibody
}  // namespace drake
