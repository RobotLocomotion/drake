#include "drake/multibody/tree/linear_spring_damper.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;

namespace multibody {
namespace {

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

class SpringDamperTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Turn off gravity so there won't be any gravitational potential
    // energy to confuse things; we just want to see the spring.
    plant_.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d::Zero());

    const SpatialInertia<double> point_mass(1., Eigen::Vector3d::Zero(),
                                            UnitInertia<double>(0., 0., 0.));
    bodyA_ = &plant_.AddRigidBody("BodyA", point_mass);
    bodyB_ = &plant_.AddRigidBody("BodyB", point_mass);

    plant_.AddJoint<WeldJoint>("WeldBodyAToWorld", plant_.world_body(),
                              std::nullopt, *bodyA_, std::nullopt,
                              math::RigidTransform<double>::Identity());

    // Allow body B to slide along the x axis.
    slider_ = &plant_.AddJoint<PrismaticJoint>(
        "Slider", plant_.world_body(), std::nullopt, *bodyB_, std::nullopt,
        Vector3<double>::UnitX());

    spring_damper_ = &plant_.AddForceElement<LinearSpringDamper>(
        *bodyA_, p_AP_, *bodyB_, p_BQ_, free_length_, stiffness_, damping_);

    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
    forces_ = std::make_unique<MultibodyForces<double>>(plant_);
  }

  void SetSliderState(double position, double position_rate) {
    slider_->set_translation(context_.get(), position);
    slider_->set_translation_rate(context_.get(), position_rate);
  }

  void CalcSpringDamperForces() const {
    forces_->SetZero();
    spring_damper_->CalcAndAddForceContribution(
        *context_, plant_.EvalPositionKinematics(*context_),
        plant_.EvalVelocityKinematics(*context_), forces_.get());
  }

  const SpatialForce<double>& GetSpatialForceOnBodyA() const {
    return forces_->body_forces().at(bodyA_->node_index());
  }

  const SpatialForce<double>& GetSpatialForceOnBodyB() const {
    return forces_->body_forces().at(bodyB_->node_index());
  }

 protected:
  MultibodyPlant<double> plant_{0.};
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const PrismaticJoint<double>* slider_{nullptr};
  const LinearSpringDamper<double>* spring_damper_{nullptr};
  std::unique_ptr<MultibodyForces<double>> forces_;

  // Parameters of the case.
  const double free_length_ = 1.0;  // [m]
  const double stiffness_ = 2.0;    // [N/m]
  const double damping_ = 0.5;      // [Ns/m]
  const double torque_arm_length_{1.0};
  const Vector3<double> p_AP_{0, torque_arm_length_, 0};
  const Vector3<double> p_BQ_{0, torque_arm_length_, 0};
};

GTEST_TEST(LinearSpringDamper, BadParameters) {
  MultibodyPlant<double> plant{0.};
  const auto& bodyA = plant.AddRigidBody("BodyA", SpatialInertia<double>());
  const auto& bodyB = plant.AddRigidBody("BodyB", SpatialInertia<double>());

  // These are reasonable parameters.
  const double free_length{1.5};  // [m]
  const double stiffness{100.};   // [N/m]
  const double damping{0.1};      // [Ns/m]
  const Vector3<double> p_AP{1., 2., 3.};
  const Vector3<double> p_BQ{4., 5., 6.};

  EXPECT_NO_THROW(plant.AddForceElement<LinearSpringDamper>(
      bodyA, p_AP, bodyB, p_BQ, free_length, stiffness, damping));

  // Verify the constructor for the spring-damper throws if either the rest
  // length, stiffness or damping are negative numbers.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddForceElement<LinearSpringDamper>(bodyA, p_AP, bodyB, p_BQ,
                                                -1.0 /* negative rest length */,
                                                stiffness, damping),
      ".*condition 'free_length > 0' failed.*");

  DRAKE_EXPECT_THROWS_MESSAGE(plant.AddForceElement<LinearSpringDamper>(
                                  bodyA, p_AP, bodyB, p_BQ, free_length,
                                  -1.0 /* negative stiffness */, damping),
                              ".*condition 'stiffness >= 0' failed.*");

  DRAKE_EXPECT_THROWS_MESSAGE(plant.AddForceElement<LinearSpringDamper>(
                                  bodyA, p_AP, bodyB, p_BQ, free_length,
                                  stiffness, -1.0 /* negative damping */),
                              ".*condition 'damping >= 0' failed.*");
}

TEST_F(SpringDamperTester, ConstructionAndAccessors) {
  EXPECT_EQ(spring_damper_->bodyA().index(), bodyA_->index());
  EXPECT_EQ(spring_damper_->bodyB().index(), bodyB_->index());
  EXPECT_EQ(spring_damper_->stiffness(), stiffness_);
  EXPECT_EQ(spring_damper_->damping(), damping_);
  EXPECT_EQ(spring_damper_->free_length(), free_length_);
  EXPECT_EQ(spring_damper_->p_AP(), p_AP_);
  EXPECT_EQ(spring_damper_->p_BQ(), p_BQ_);
}

// Verify the spring applies no forces when the separation length equals the
// rest length.
TEST_F(SpringDamperTester, RestLength) {
  SetSliderState(1.0, 0.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  EXPECT_EQ(F_A_W.get_coeffs(), SpatialForce<double>::Zero().get_coeffs());
  EXPECT_EQ(F_B_W.get_coeffs(), SpatialForce<double>::Zero().get_coeffs());

  // Verify the potential energy is zero.
  const double potential_energy = spring_damper_->CalcPotentialEnergy(
      *context_, plant_.EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, 0.0, kTolerance);
}

TEST_F(SpringDamperTester, LengthApproachesZero) {
  SetSliderState(0.0, 0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(CalcSpringDamperForces(), std::runtime_error,
                              "The length of the spring became nearly zero. "
                              "Revisit your model to avoid this situation.");
}

// Verify forces computation when the spring length is larger than its rest
// length.
TEST_F(SpringDamperTester, LengthLargerThanRestLength) {
  const double length = 2.0;
  SetSliderState(length, 0.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  const double expected_force_magnitude = stiffness_ * (length - free_length_);
  const double expected_torque_magnitude =
      expected_force_magnitude * torque_arm_length_;
  const SpatialForce<double> F_A_W_expected(
      Vector3<double>(0, 0, -expected_torque_magnitude),
      Vector3<double>(expected_force_magnitude, 0, 0));
  EXPECT_TRUE(CompareMatrices(F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // Verify the value of the potential energy.
  const double potential_energy_expected =
      0.5 * stiffness_ * (length - free_length_) * (length - free_length_);
  const double potential_energy = spring_damper_->CalcPotentialEnergy(
      *context_, plant_.EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, potential_energy_expected, kTolerance);

  // The System potential energy should consist only of the spring energy.
  EXPECT_EQ(plant_.EvalPotentialEnergy(*context_), potential_energy);

  // Since the spring configuration is static, that is velocities are zero, we
  // expect zero conservative and non-conservative power.
  const double conservative_power = spring_damper_->CalcConservativePower(
      *context_, plant_.EvalPositionKinematics(*context_),
      plant_.EvalVelocityKinematics(*context_));
  EXPECT_EQ(conservative_power, 0.0);

  const double non_conservative_power =
      spring_damper_->CalcNonConservativePower(
          *context_, plant_.EvalPositionKinematics(*context_),
          plant_.EvalVelocityKinematics(*context_));
  EXPECT_EQ(non_conservative_power, 0.0);
}

// Verify forces computation when the spring length is smaller than its rest
// length.
TEST_F(SpringDamperTester, LengthSmallerThanRestLength) {
  const double length = 0.5;
  SetSliderState(length, 0.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  const double expected_force_magnitude = stiffness_ * (length - free_length_);
  const double expected_torque_magnitude =
      expected_force_magnitude * torque_arm_length_;
  const SpatialForce<double> F_A_W_expected(
      Vector3<double>(0, 0, -expected_torque_magnitude),
      Vector3<double>(expected_force_magnitude, 0, 0));
  EXPECT_TRUE(CompareMatrices(F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
}

// Verify forces computation when the spring is at its rest length (zero spring
// force) but it is expanding/compressing and therefore damping is non-zero.
TEST_F(SpringDamperTester, NonZeroVelocity) {
  // The spring is stretching.
  const double length_dot = 1.0;
  // We use the rest length for this test so that the spring contribution is
  // zero.
  SetSliderState(free_length_, length_dot);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  // The spring force is zero in this case and only the damping force is
  // non-zero.
  const double expected_force_magnitude = damping_ * length_dot;
  const double expected_torque_magnitude =
      expected_force_magnitude * torque_arm_length_;
  const SpatialForce<double> F_A_W_expected(
      Vector3<double>(0, 0, -expected_torque_magnitude),
      Vector3<double>(expected_force_magnitude, 0, 0));
  EXPECT_TRUE(CompareMatrices(F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // Spring is compressing.
  SetSliderState(free_length_, -length_dot);
  CalcSpringDamperForces();
  EXPECT_TRUE(CompareMatrices(F_A_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(F_B_W.get_coeffs(), F_A_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
}

// This test verifies the computation of both conservative and non-conservative
// powers for a configuration when they are non-zero.
TEST_F(SpringDamperTester, Power) {
  const double length = 2.0;
  const double length_dot = 1.0;
  SetSliderState(length, length_dot);

  const double conservative_power = spring_damper_->CalcConservativePower(
      *context_, plant_.EvalPositionKinematics(*context_),
      plant_.EvalVelocityKinematics(*context_));
  const double conservative_power_expected =
      -stiffness_ * (length - free_length_) * length_dot;
  EXPECT_NEAR(conservative_power, conservative_power_expected, kTolerance);

  const double non_conservative_power =
      spring_damper_->CalcNonConservativePower(
          *context_, plant_.EvalPositionKinematics(*context_),
          plant_.EvalVelocityKinematics(*context_));
  const double non_conservative_power_expected =
      -damping_ * length_dot * length_dot;
  // It should always be non-positive.
  EXPECT_LT(non_conservative_power, 0.0);
  EXPECT_NEAR(non_conservative_power, non_conservative_power_expected,
              kTolerance);

  // System power should reflect only the spring.
  EXPECT_EQ(plant_.EvalConservativePower(*context_), conservative_power);
  EXPECT_EQ(plant_.EvalNonConservativePower(*context_),
            non_conservative_power);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
