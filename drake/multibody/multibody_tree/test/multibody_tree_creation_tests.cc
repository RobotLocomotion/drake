#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
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
using std::unique_ptr;

// Tests the basic MultibodyTree API to add bodies and mobilizers.
// Tests we cannot create graph loops.
GTEST_TEST(MultibodyTree, AddMobilizers) {
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

#if 0
class TreeTopologyTests : public ::testing::Test {
 public:
  // Creates an "empty" MultibodyTree that only contains the "world" body and
  // world body frame.
  void SetUp() override {
    model_ = std::make_unique<MultibodyTree<double>>();

    const int kNumBodies = 7;
    bodies_.push_back(&model_->get_world_body());
    for (int i =1; i < kNumBodies; ++i)
      bodies_.push_back(AddTestBody());

    // Adds mobilizers to connect bodies according to the following diagram:
    mobilizers_.push_back(ConnectBodies(*bodies_[1], *bodies_[6]));  // mob. 0
    mobilizers_.push_back(ConnectBodies(*bodies_[4], *bodies_[2]));  // mob. 1
    mobilizers_.push_back(ConnectBodies(*bodies_[0], *bodies_[7]));  // mob. 2
    mobilizers_.push_back(ConnectBodies(*bodies_[5], *bodies_[3]));  // mob. 3
    mobilizers_.push_back(ConnectBodies(*bodies_[0], *bodies_[5]));  // mob. 4
    mobilizers_.push_back(ConnectBodies(*bodies_[4], *bodies_[1]));  // mob. 5
    mobilizers_.push_back(ConnectBodies(*bodies_[0], *bodies_[4]));  // mob. 6
  }

  const RigidBody<double>* AddTestBody() {
    // NaN SpatialInertia to instantiate the RigidBody objects.
    // It is safe here since this tests only focus on topological information.
    const SpatialInertia<double> M_Bo_B;
    return &model_->AddBody<RigidBody>(M_Bo_B);
  }

  const Mobilizer<double>* ConnectBodies(
      const Body<double>& inboard, const Body<double>& outboard) {
    return &model_->AddMobilizer<RevoluteMobilizer>(
        inboard.get_body_frame(), outboard.get_body_frame(),
        Vector3d::UnitZ());
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Bodies:
  std::vector<const Body<double>*> bodies_;
  // Mobilizers:
  std::vector<const Mobilizer<double>*> mobilizers_;
};

TEST_F(TreeTopologyTests, Finalize) {
  EXPECT_EQ(model_->get_num_bodies(), 8);
  EXPECT_EQ(model_->get_num_mobilizers(), 7);
}
#endif
}  // namespace
}  // namespace multibody
}  // namespace drake

//int main() {
//  drake::multibody::DoMain();
//  return 0;
//}