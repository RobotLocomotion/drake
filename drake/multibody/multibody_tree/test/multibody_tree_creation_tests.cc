#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

#include <memory>

namespace drake {
namespace multibody {
namespace {

using std::make_unique;
using std::unique_ptr;

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
  // order starting with id = 0 (MultibodyTree::kWorldBodyId) for the "world"
  // body.
  EXPECT_EQ(world_body.get_id(), kWorldBodyId);
  EXPECT_EQ(pendulum.get_id(), BodyIndex(1));

  // Tests API to access bodies.
  EXPECT_EQ(model->get_body(BodyIndex(1)).get_id(), pendulum.get_id());
  EXPECT_EQ(model->get_mutable_body(BodyIndex(1))->get_id(), pendulum.get_id());

  // Rigid bodies have no generalized coordinates.
  EXPECT_EQ(pendulum.get_num_positions(), 0);
  EXPECT_EQ(pendulum.get_num_velocities(), 0);

  // Verify that no more bodies can be added to a MultibodyTree if it was
  // compiled already.
  model->Compile();
  EXPECT_THROW(RigidBody<double>::Create(model), std::runtime_error);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
