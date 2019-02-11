#include "drake/multibody/tree/revolute_spring.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;

namespace multibody {
namespace internal {
namespace {

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

class SpringTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    bodyA_ = &model->AddRigidBody("BodyA", SpatialInertia<double>());
    bodyB_ = &model->AddRigidBody("BodyB", SpatialInertia<double>());
    bodyC_ = &model->AddRigidBody("BodyC", SpatialInertia<double>());

    model->AddJoint<WeldJoint>(
        "WeldBodyAToWorld", model->world_body(), {}, *bodyA_, {},
        Isometry3<double>::Identity());

    // Allow body B to rotate about the z axis.
    joint_ = &model->AddJoint<RevoluteJoint>("joint_AB", *bodyA_, {},
        *bodyB_, {}, Vector3<double>::UnitZ());

    model->AddJoint<RevoluteJoint>("joint_BC", *bodyB_, {},
        *bodyC_, {}, Vector3<double>::UnitX());

    // Add spring
    spring_ = &model->AddForceElement<RevoluteSpring>(
        *joint_, free_length_, stiffness_);

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = system_->CreateDefaultContext();

    context_->EnableCaching();

    forces_ = std::make_unique<MultibodyForces<double>>(tree());
  }

  void SetJointState(double position, double position_rate) {
    joint_->set_angle(context_.get(), position);
    joint_->set_angular_rate(context_.get(), position_rate);
  }

  void CalcSpringForces() const {
    forces_->SetZero();
    spring_->CalcAndAddForceContribution(
        *context_, tree().EvalPositionKinematics(*context_),
        tree().EvalVelocityKinematics(*context_), forces_.get());
  }

  const MultibodyTree<double>& tree() const {
    return GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RigidBody<double>* bodyC_{nullptr};
  const RevoluteJoint<double>* joint_{nullptr};
  const RevoluteSpring<double>* spring_{nullptr};
  std::unique_ptr<MultibodyForces<double>> forces_;

  // Parameters of the case.
  const double free_length_ = 1.0;  // [m]
  const double stiffness_ = 2.0;    // [N/m]
};

TEST_F(SpringTester, ConstructionAndAccessors) {
  EXPECT_EQ(spring_->joint().index(), joint_->index());
  EXPECT_EQ(spring_->stiffness(), stiffness_);
  EXPECT_EQ(spring_->free_length(), free_length_);
}

// Verify the spring applies no forces when the separation equals the
// rest length.
TEST_F(SpringTester, RestLength) {
  SetJointState(1.0, 0.0);
  CalcSpringForces();
  const VectorX<double>& generalized_forces = forces_->generalized_forces();
  EXPECT_EQ(generalized_forces, VectorX<double>::Zero(2));

  // Verify the potential energy is zero.
  const double potential_energy = spring_->CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, 0.0, kTolerance);
}

// Verify forces computation when the spring length is larger than its rest
// length.
TEST_F(SpringTester, DeltaLength) {
  const double length = 2.0;
  SetJointState(length, 0.0);
  CalcSpringForces();
  const VectorX<double>& generalized_forces = forces_->generalized_forces();

  const double expected_torque_magnitude = stiffness_ * (free_length_ - length);
  VectorX<double> expected_generalized_forces(2);
  expected_generalized_forces << expected_torque_magnitude, 0;
  const SpatialForce<double> F_A_W_expected(
      Vector3<double>(0, 0, -expected_torque_magnitude),
      Vector3<double>::Zero());
  EXPECT_TRUE(CompareMatrices(generalized_forces, expected_generalized_forces,
      kTolerance, MatrixCompareType::relative));
  
  // Verify the value of the potential energy.
  const double potential_energy_expected =
      0.5 * stiffness_ * (length - free_length_) * (length - free_length_);
  const double potential_energy = spring_->CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, potential_energy_expected, kTolerance);

  // Since the spring configuration is static, that is velocities are zero, we
  // expect zero conservative and non-conservative power.
  const double conservative_power = spring_->CalcConservativePower(
      *context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  EXPECT_NEAR(conservative_power, 0.0, kTolerance);
}

// This test verifies the computation of both conservative and non-conservative
// powers for a configuration when they are non-zero.
TEST_F(SpringTester, Power) {
  const double length = 2.0;
  const double length_dot = 1.0;
  SetJointState(length, length_dot);

  const double conservative_power = spring_->CalcConservativePower(
      *context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  const double conservative_power_expected =
      -stiffness_ * (length - free_length_) * length_dot;
  EXPECT_NEAR(conservative_power, conservative_power_expected, kTolerance);

  const double non_conservative_power =
      spring_->CalcNonConservativePower(
          *context_, tree().EvalPositionKinematics(*context_),
          tree().EvalVelocityKinematics(*context_));
  const double non_conservative_power_expected = 0;
  // It should always be non-positive.
  EXPECT_NEAR(non_conservative_power, non_conservative_power_expected,
              kTolerance);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
