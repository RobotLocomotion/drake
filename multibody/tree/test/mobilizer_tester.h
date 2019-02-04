#pragma once

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper fixture to setup a simple MBT model containing a single body
// connected to the world by a mobilizer.
class MobilizerTester : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a
  // mobilizer connecting it to the world, with the sole purpose of verifying
  // the mobilizer methods.
  MobilizerTester() {
    // Spatial inertia for adding a body. The actual value is not important for
    // these tests since they are all kinematic.
    const SpatialInertia<double> M_B;

    // Create an empty model.
    owned_tree_ = std::make_unique<MultibodyTree<double>>();
    tree_ = owned_tree_.get();

    // Add a body so we can add a mobilizer to it.
    body_ = &owned_tree_->AddBody<RigidBody>(M_B);
  }

  template <template <typename> class MobilizerType>
  const MobilizerType<double>& AddMobilizerAndFinalize(
      std::unique_ptr<MobilizerType<double>> mobilizer) {
    // Add a mobilizer between the world and the body:
    const MobilizerType<double>& mobilizer_ref =
        owned_tree_->AddMobilizer(std::move(mobilizer));

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(
        std::move(owned_tree_));
    context_ = system_->CreateDefaultContext();

    return mobilizer_ref;
  }

  const MultibodyTree<double>& tree() const { return *tree_; }
  MultibodyTree<double>& mutable_tree() { return *tree_; }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* body_{nullptr};

 private:
  // This unique_ptr is only valid between construction and the call to
  // AddMobilizerAndFinalize(). After AddMobilizerAndFinalize() it is an
  // invalid nullptr.
  std::unique_ptr<MultibodyTree<double>> owned_tree_;
  MultibodyTree<double>* tree_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
