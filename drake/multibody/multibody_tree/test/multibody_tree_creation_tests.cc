#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace {

using std::make_unique;
using std::unique_ptr;

// Test the basic MultibodyTree API to create and add bodies.
GTEST_TEST(MultibodyTree, AddBodies) {
  auto owned_model = std::make_unique<MultibodyTree<double>>();
  MultibodyTree<double>* model = owned_model.get();

  // Initially there is only one body, the world.
  EXPECT_EQ(model->get_num_bodies(), 1);

  // Retrieves the world body.
  const Body<double>& world_body = model->get_world_body();

  // Adds a new body to the world.
  const RigidBody<double>& pendulum = RigidBody<double>::Create(model);

  // Body identifiers are unique and are assigned by MultibodyTree in increasing
  // order starting with index = 0 (world_index()) for the "world" body.
  EXPECT_EQ(world_body.get_index(), world_index());
  EXPECT_EQ(pendulum.get_index(), BodyIndex(1));

  // Tests API to access bodies.
  EXPECT_EQ(model->get_body(BodyIndex(1)).get_index(), pendulum.get_index());
  EXPECT_EQ(model->get_mutable_body(BodyIndex(1)).get_index(),
            pendulum.get_index());

  // Rigid bodies have no generalized coordinates.
  EXPECT_EQ(pendulum.get_num_flexible_positions(), 0);
  EXPECT_EQ(pendulum.get_num_flexible_velocities(), 0);

  EXPECT_FALSE(model->topology_is_valid());
  // Verifies that the topology of this model gets validated at compile stage.
  model->Compile();
  EXPECT_TRUE(model->topology_is_valid());

  // Verifies that an exception is throw if a call to Compile() is attempted to
  // an already compiled MultibodyTree.
  EXPECT_THROW(model->Compile(), std::logic_error);

  // Verifies that more bodies can still be added to an already compiled model.
  // However, the topology of the multibody tree gets invalidated.
  EXPECT_NO_THROW(RigidBody<double>::Create(model));
  EXPECT_FALSE(model->topology_is_valid());

  // Verifies we can re-compile.
  EXPECT_NO_THROW(model->Compile());
  EXPECT_TRUE(model->topology_is_valid());
}

// Tests the correctness of MultibodyTreeElement checks to verify one or more
// elements belong to a given MultibodyTree.
GTEST_TEST(MultibodyTree, MultibodyTreeElementChecks) {
  auto model1 = std::make_unique<MultibodyTree<double>>();
  auto model2 = std::make_unique<MultibodyTree<double>>();

  const RigidBody<double>& body1 = RigidBody<double>::Create(model1.get());
  const RigidBody<double>& body2 = RigidBody<double>::Create(model2.get());

  // Tests that the created bodies indeed do have a parent MultibodyTree.
  EXPECT_NO_THROW(body1.HasParentTreeOrThrow());
  EXPECT_NO_THROW(body2.HasParentTreeOrThrow());

  // Tests the check to verify that two bodies belong to the same MultibodyTree.
  EXPECT_THROW(body1.HasSameParentTreeOrThrow(body2), std::logic_error);
  EXPECT_NO_THROW(model1->get_world_body().HasSameParentTreeOrThrow(body1));
  EXPECT_NO_THROW(model2->get_world_body().HasSameParentTreeOrThrow(body2));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
