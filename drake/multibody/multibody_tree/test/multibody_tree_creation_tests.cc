#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

#include <memory>

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {
namespace {

using std::make_unique;
using std::unique_ptr;

GTEST_TEST(MultibodyTree, AddBodies) {
  auto owned_model = std::make_unique<MultibodyTree<double>>();
  MultibodyTree<double>* model = owned_model.get();
  const Body<double>& world_body = model->get_world_body();

  const RigidBody<double>& pendulum = RigidBody<double>::Create(model);

  EXPECT_EQ(world_body.get_id(), kWorldBodyId);
  EXPECT_EQ(pendulum.get_id(), BodyIndex(1));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
