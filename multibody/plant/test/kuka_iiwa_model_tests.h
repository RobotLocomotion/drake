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

    // Create a model of a Kuka arm. Notice we do not weld the robot's base
    // to the world and therefore the model is free floating in space. This
    // makes for a more interesting setup to test the computation of
    // analytical Jacobians.
    plant_ = std::make_unique<MultibodyPlant<double>>();
    Parser parser(plant_.get());
    parser.AddModelFromFile(kArmSdfPath);
    // Add a frame H with a fixed pose X_EH in the end effector frame E.
    end_effector_link_ = &plant_->GetBodyByName("iiwa_link_7");
    frame_H_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "H", *end_effector_link_, X_EH_.GetAsIsometry3()));
    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();

    // Scalar-convert the model and create a default context for it.
    plant_autodiff_ = std::make_unique<MultibodyPlant<AutoDiffXd>>(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

  void SetArbitraryConfiguration() {
    // Get an arbitrary set of angles and velocities for each joint.
    const VectorX<double> x0 = GetArbitraryJointConfiguration();

    EXPECT_EQ(plant_->num_joints(), 7);
    for (JointIndex joint_index(0); joint_index < plant_->num_joints();
         ++joint_index) {
      const RevoluteJoint<double>& joint =
          dynamic_cast<const RevoluteJoint<double>&>(
              plant_->get_joint(joint_index));
      joint.set_angle(context_.get(), x0[joint_index]);
      joint.set_angular_rate(context_.get(), x0[kNumJoints + joint_index]);
    }

    // Set an arbitrary (though non-identity) pose of the floating base link.
    const auto& base_body = plant_->GetBodyByName("iiwa_link_0");
    const RigidTransform<double> X_WB(
        RollPitchYaw<double>(M_PI / 3, -M_PI / 2, M_PI / 8),
        Vector3<double>(0.05, -0.2, 0.05));
    plant_->SetFreeBodyPoseInAnchoredFrame(
        context_.get(), plant_->world_frame(), base_body,
        X_WB.GetAsIsometry3());
    // Set an arbitrary non-zero spatial velocity of the floating base link.
    const Vector3<double> w_WB{-1, 1, -1};
    const Vector3<double> v_WB{1, -1, 1};
    plant_->SetFreeBodySpatialVelocity(
        context_.get(), base_body, {w_WB, v_WB});
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

  // Computes the analytical Jacobian Jq_WPi for a set of points Pi moving with
  // the end effector frame E, given their (fixed) position p_EPi in the end
  // effector frame.
  // This templated helper method allows us to use automatic differentiation.
  // See MultibodyTree::CalcPointsAnalyticalJacobianExpressedInWorld() for
  // details.
  // TODO(amcastro-tri): Rename this method as per issue #10155.
  template <typename T>
  void CalcPointsOnEndEffectorAnalyticJacobian(
      const MultibodyPlant<T>& plant_on_T,
      const Context<T>& context_on_T,
      const MatrixX<T>& p_EPi,
      MatrixX<T>* p_WPi, MatrixX<T>* Jq_WPi) const {
    const Body<T>& linkG_on_T =
        plant_on_T.get_body(end_effector_link_->index());
    plant_on_T.CalcPointsAnalyticalJacobianExpressedInWorld(
        context_on_T, linkG_on_T.body_frame(), p_EPi, p_WPi, Jq_WPi);
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
