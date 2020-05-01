#pragma once

#include <limits>
#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {

using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using systems::Context;
using std::unique_ptr;

namespace multibody {
namespace test {

// Fixture to perform a number of computational tests on a KUKA Iiwa model.
class KukaIiwaModelTests : public ::testing::Test {
 public:
  // Creates MultibodyTree for a KUKA Iiwa robot arm.
  void SetUp() override {
    const std::string kArmSdfPath = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf");

    // Create a model of a Kuka arm. Notice we do not weld the robot's base to
    // the world and therefore the model is free floating in space. This makes
    // for a more interesting setup to test the computation of Jacobians with
    // respect to q̇ (time-derivative of generalized positions).
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    Parser parser(plant_.get());
    parser.AddModelFromFile(kArmSdfPath);
    // Add a frame H with a fixed pose X_EH in the end effector frame E.
    end_effector_link_ = &plant_->GetBodyByName("iiwa_link_7");
    frame_H_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "H", *end_effector_link_, X_EH_));
    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();

    // Fix input ports.
    const VectorX<double> tau =
        VectorX<double>::Zero(plant_->num_actuated_dofs());
    plant_->get_actuation_input_port().FixValue(context_.get(), tau);

    // Scalar-convert the model and create a default context for it.
    plant_autodiff_ = systems::System<double>::ToAutoDiffXd(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

  // If unit_quaternion = false then the quaternion for the free floating base
  // is not normalized. This configuration is useful to verify the computation
  // of Jacobians with respect to q̇ (time-derivative of generalized positions),
  // even if the state stores a non-unit quaternion.
  void SetArbitraryConfiguration(bool unit_quaternion = true) {
    // Get an arbitrary set of angles and velocities for each joint.
    const VectorX<double> x0 = GetArbitraryJointConfiguration();
    SetState(x0);
     if (!unit_quaternion) {
        VectorX<double> q = plant_->GetPositions(*context_);
        // TODO(amcastro-tri): This assumes the first 4 entries in the
        // generalized positions correspond to the quaternion for the free
        // floating robot base. Provide API to access these values.
        q.head<4>() *= 2;  // multiply quaternion by a factor.
        plant_->SetPositions(context_.get(), q);
    }
  }

  // Sets the state of the joints according to x_joints. The pose and spatial
  // velocity of the base is set arbitrarily to a non-identity pose and non-zero
  // spatial velocity.
  void SetState(const VectorX<double>& x_joints) {
    EXPECT_EQ(plant_->num_joints(), 7);
    for (JointIndex joint_index(0); joint_index < plant_->num_joints();
         ++joint_index) {
      const RevoluteJoint<double>& joint =
          dynamic_cast<const RevoluteJoint<double>&>(
              plant_->get_joint(joint_index));
      joint.set_angle(context_.get(), x_joints[joint_index]);
      joint.set_angular_rate(context_.get(),
                             x_joints[kNumJoints + joint_index]);
    }

    // Set an arbitrary (though non-identity) pose of the floating base link.
    const auto& base_body = plant_->GetBodyByName("iiwa_link_0");
    const RigidTransform<double> X_WB(
        RollPitchYaw<double>(M_PI / 3, -M_PI / 2, M_PI / 8),
        Vector3<double>(0.05, -0.2, 0.05));
    plant_->SetFreeBodyPoseInAnchoredFrame(
        context_.get(), plant_->world_frame(), base_body, X_WB);
    // Set an arbitrary non-zero spatial velocity of the floating base link.
    const Vector3<double> w_WB{-1, 1, -1};
    const Vector3<double> v_WB{1, -1, 1};
    plant_->SetFreeBodySpatialVelocity(context_.get(), base_body, {w_WB, v_WB});
  }

  // Gets an arm state to an arbitrary configuration in which joint angles and
  // rates are non-zero.
  VectorX<double> GetArbitraryJointConfiguration() {
    VectorX<double> x(2 * kNumJoints);

    // A set of values for the joint's angles chosen mainly to avoid in-plane
    // motions.
    const double q30 = M_PI / 6, q60 = M_PI / 3;
    const double qA = q60;
    const double qB = q30;
    const double qC = q60;
    const double qD = q30;
    const double qE = q60;
    const double qF = q30;
    const double qG = q60;
    // Arbitrary non-zero velocities.
    const double v_positive = 0.1;   // rad/s
    const double v_negative = -0.1;  // rad/s
    const double vA = v_positive;
    const double vB = v_negative;
    const double vC = v_positive;
    const double vD = v_negative;
    const double vE = v_positive;
    const double vF = v_negative;
    const double vG = v_positive;
    x << qA, qB, qC, qD, qE, qF, qG, vA, vB, vC, vD, vE, vF, vG;

    return x;
  }

 protected:
  // Problem sizes.
  const int kNumJoints = 7;
  const int kNumPositions = 14;
  const int kNumVelocities = 13;

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
  const RigidTransform<double> X_EH_{
      RollPitchYaw<double>{M_PI_2, 0, M_PI_2}.ToRotationMatrix(),
      Vector3<double>{0, 0, 0.081}};
  const Body<double>* end_effector_link_{nullptr};
  const Frame<double>* frame_H_{nullptr};

  // AutoDiffXd model to compute automatic derivatives:
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
};

}  // namespace test
}  // namespace multibody
}  // namespace drake
