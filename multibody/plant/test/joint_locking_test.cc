#include <memory>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::Vector3d;
using systems::Context;

namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static const std::vector<int>& EvalJointLockingIndices(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalJointLockingIndices(context);
  }
};

namespace {
// Set up a plant with 2 trees, one tree having a single floating body, the
// second tree a serial chain of two bodies attached to each other and world by
// revolute joints.
class JointLockingTest : public ::testing::TestWithParam<int> {
 public:
  void SetUp() {
    plant_ = std::make_unique<MultibodyPlant<double>>(0.1);

    // Each permutation of adding trees (children of world body) to the plant.
    switch (GetParam()) {
      case 0:
        AddFloatingPendulum();
        AddDoublePendulum();
        break;
      case 1:
        AddDoublePendulum();
        AddFloatingPendulum();
        break;
    }

    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;

 private:
  void AddFloatingPendulum() {
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    plant_->AddRigidBody("body1", SpatialInertia<double>::MakeTestCube());
    plant_->AddRigidBody("body2", SpatialInertia<double>::MakeTestCube());

    std::unique_ptr<RevoluteJoint<double>> body1_body2 =
        std::make_unique<RevoluteJoint<double>>(
            "body1_body2", plant_->GetRigidBodyByName("body1").body_frame(),
            plant_->GetRigidBodyByName("body2").body_frame(),
            Vector3d::UnitZ());
    plant_->AddJoint(std::move(body1_body2));
  }

  void AddDoublePendulum() {
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    plant_->AddRigidBody("body3", SpatialInertia<double>::MakeTestCube());
    plant_->AddRigidBody("body4", SpatialInertia<double>::MakeTestCube());

    std::unique_ptr<RevoluteJoint<double>> world_body3 =
        std::make_unique<RevoluteJoint<double>>(
            "world_body3", plant_->world_frame(),
            plant_->GetRigidBodyByName("body3").body_frame(),
            Vector3d::UnitZ());
    std::unique_ptr<RevoluteJoint<double>> body3_body4 =
        std::make_unique<RevoluteJoint<double>>(
            "body3_body4", plant_->GetRigidBodyByName("body3").body_frame(),
            plant_->GetRigidBodyByName("body4").body_frame(),
            Vector3d::UnitZ());

    plant_->AddJoint(std::move(world_body3));
    plant_->AddJoint(std::move(body3_body4));
  }
};

// Test that regardless of the order the joints were added to the plant (i.e.
// different joint indexing orders), the established joint locking velocity
// indexing remains correct.
TEST_P(JointLockingTest, JointLockingIndicesTest) {
  const RigidBody<double>& body1 = plant_->GetRigidBodyByName("body1");

  const RevoluteJoint<double>& body1_body2 =
      plant_->GetJointByName<RevoluteJoint>("body1_body2");
  const RevoluteJoint<double>& world_body3 =
      plant_->GetJointByName<RevoluteJoint>("world_body3");
  RevoluteJoint<double>& body3_body4 =
      plant_->GetMutableJointByName<RevoluteJoint>("body3_body4");

  const int body1_velocity_start =
      body1.floating_velocities_start() - plant_->num_positions();

  // No joints/bodies are locked, all joint/body velocity indices should exist.
  {
    const std::vector<int>& unlocked_indices =
        MultibodyPlantTester::EvalJointLockingIndices(*plant_, *context_);

    EXPECT_EQ(unlocked_indices.size(), 9);

    const std::vector<int>& expected_unlocked_indices = {
        body1_velocity_start,         body1_velocity_start + 1,
        body1_velocity_start + 2,     body1_velocity_start + 3,
        body1_velocity_start + 4,     body1_velocity_start + 5,
        body1_body2.velocity_start(), world_body3.velocity_start(),
        body3_body4.velocity_start()};

    EXPECT_THAT(unlocked_indices,
                testing::UnorderedElementsAreArray(expected_unlocked_indices));
  }

  // Lock body3_body4 and re-evaluate joint locking indices
  {
    body3_body4.Lock(context_.get());
    const std::vector<int>& unlocked_indices =
        MultibodyPlantTester::EvalJointLockingIndices(*plant_, *context_);

    EXPECT_EQ(unlocked_indices.size(), 8);

    const std::vector<int>& expected_unlocked_indices = {
        body1_velocity_start,         body1_velocity_start + 1,
        body1_velocity_start + 2,     body1_velocity_start + 3,
        body1_velocity_start + 4,     body1_velocity_start + 5,
        body1_body2.velocity_start(), world_body3.velocity_start()};

    EXPECT_THAT(unlocked_indices,
                testing::UnorderedElementsAreArray(expected_unlocked_indices));
  }

  // Unlock body3_body4 and lock body1 and re-evaluate joint locking indices.
  {
    body3_body4.Unlock(context_.get());
    body1.Lock(context_.get());

    const std::vector<int>& unlocked_indices =
        MultibodyPlantTester::EvalJointLockingIndices(*plant_, *context_);

    EXPECT_EQ(unlocked_indices.size(), 3);

    const std::vector<int>& expected_unlocked_indices = {
        body1_body2.velocity_start(), world_body3.velocity_start(),
        body3_body4.velocity_start()};

    EXPECT_THAT(unlocked_indices,
                testing::UnorderedElementsAreArray(expected_unlocked_indices));
  }
}

INSTANTIATE_TEST_SUITE_P(IndexPermutations, JointLockingTest,
                         ::testing::Values(0, 1));
}  // namespace
}  // namespace multibody
}  // namespace drake
