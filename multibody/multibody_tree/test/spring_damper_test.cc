#include "drake/multibody/multibody_tree/spring_damper.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;

namespace multibody {
namespace multibody_tree {
namespace {

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

class SpringDamperTester : public ::testing::Test {
 public:
  void SetUp() override {
    bodyA_ = &model_.AddRigidBody(
        "BodyA", SpatialInertia<double>());
    bodyB_ = &model_.AddRigidBody(
        "BodyB", SpatialInertia<double>());

    model_.AddJoint<WeldJoint>(
        "WeldBodyAToWorld", model_.world_body(), {}, *bodyA_, {},
        Isometry3<double>::Identity());

    // Allow body B to slide along the x axis.
    slider_ = &model_.AddJoint<PrismaticJoint>(
        "Slider", model_.world_body(), {}, *bodyB_, {},
        Vector3<double>::UnitX());

    spring_damper_ = &model_.AddForceElement<SpringDamper>(
        *bodyA_, p_AP_, *bodyB_, p_BQ_, rest_length_, stiffness_, damping_);

    model_.Finalize();

    context_ = model_.CreateDefaultContext();
    mbt_context_ = dynamic_cast<MultibodyTreeContext<double>*>(context_.get());
    ASSERT_TRUE(mbt_context_ != nullptr);
    pc_ = std::make_unique<PositionKinematicsCache<double>>(
        model_.get_topology());
    vc_ = std::make_unique<VelocityKinematicsCache<double>>(
        model_.get_topology());
    forces_ = std::make_unique<MultibodyForces<double>>(model_);
  }

  void SetSliderState(double position, double position_rate) {
    slider_->set_translation(context_.get(), position);
    slider_->set_translation_rate(context_.get(), position_rate);
    // Update the kinematics cache.
    model_.CalcPositionKinematicsCache(*context_, pc_.get());
    model_.CalcVelocityKinematicsCache(*context_, *pc_, vc_.get());
  }

  void CalcSpringDamperForces() const {
    forces_->SetZero();
    spring_damper_->CalcAndAddForceContribution(
        *mbt_context_, *pc_, *vc_, forces_.get());
  }

  const SpatialForce<double>& GetSpatialForceOnBodyA() const {
    return forces_->body_forces().at(bodyA_->node_index());
  }

  const SpatialForce<double>& GetSpatialForceOnBodyB() const {
    return forces_->body_forces().at(bodyB_->node_index());
  }

 protected:
  MultibodyTree<double> model_;
  std::unique_ptr<Context<double>> context_;
  MultibodyTreeContext<double>* mbt_context_{nullptr};
  std::unique_ptr<PositionKinematicsCache<double>> pc_;
  std::unique_ptr<VelocityKinematicsCache<double>> vc_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const PrismaticJoint<double>* slider_{nullptr};
  const SpringDamper<double>* spring_damper_{nullptr};
  std::unique_ptr<MultibodyForces<double>> forces_;

  // Parameters of the case.
  const double rest_length_ = 1.0;  // [m]
  const double stiffness_ = 2.0;    // [N/m]
  const double damping_ = 0.5;      // [Ns/m]
  const double torque_arm_length_{1.0};
  const Vector3<double> p_AP_{0, torque_arm_length_, 0};
  const Vector3<double> p_BQ_{0, torque_arm_length_, 0};
};

TEST_F(SpringDamperTester, ConstructionAndAccessors) {
  EXPECT_EQ(spring_damper_->bodyA().index(), bodyA_->index());
  EXPECT_EQ(spring_damper_->bodyB().index(), bodyB_->index());
  EXPECT_EQ(spring_damper_->stiffness(), stiffness_);
  EXPECT_EQ(spring_damper_->damping(), damping_);
  EXPECT_EQ(spring_damper_->rest_length(), rest_length_);
  EXPECT_EQ(spring_damper_->point_on_bodyA(), p_AP_);
  EXPECT_EQ(spring_damper_->point_on_bodyB(), p_BQ_);
}

// Verify the spring applies no forces when the spearation length equals the
// rest length.
TEST_F(SpringDamperTester, RestLength) {
  SetSliderState(1.0, 0.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  EXPECT_EQ(F_A_W.get_coeffs(), SpatialForce<double>::Zero().get_coeffs());
  EXPECT_EQ(F_B_W.get_coeffs(), SpatialForce<double>::Zero().get_coeffs());

  // Verify the potential energy is zero.
  const double potential_energy =
      spring_damper_->CalcPotentialEnergy(*mbt_context_, *pc_);
  EXPECT_NEAR(potential_energy, 0.0, kTolerance);
}

// Verify forces computation when the spring length is larger than its rest
// length.
TEST_F(SpringDamperTester, LengthLargerThanRestLength) {
  const double length = 2.0;
  SetSliderState(length, 0.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  const double expected_force_magnitude = stiffness_ * (length - rest_length_);
  const double expected_torque_magnitude =
      expected_force_magnitude * torque_arm_length_;
  const SpatialForce<double> F_A_W_expected(
      Vector3<double>(0, 0, -expected_torque_magnitude),
      Vector3<double>(expected_force_magnitude, 0, 0));
  EXPECT_TRUE(CompareMatrices(
      F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));

  // Verify the value of the potential energy.
  const double potential_energy_expected =
      0.5 * stiffness_ * (length - rest_length_) * (length - rest_length_);
  const double potential_energy =
      spring_damper_->CalcPotentialEnergy(*mbt_context_, *pc_);
  EXPECT_NEAR(potential_energy, potential_energy_expected, kTolerance);

  // Since the spring configuration is static, that is velocities are zero, we
  // expect zero conservative and non-conservative power.
  const double conservative_power =
      spring_damper_->CalcConservativePower(*mbt_context_, *pc_, *vc_);
  EXPECT_NEAR(conservative_power, 0.0, kTolerance);
}

// Verify forces computation when the spring length is smaller than its rest
// length.
TEST_F(SpringDamperTester, LengthSmallerThanRestLength) {
  const double length = 0.5;
  SetSliderState(length, 0.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  const double expected_force_magnitude = stiffness_ * (length - rest_length_);
  const double expected_torque_magnitude =
      expected_force_magnitude * torque_arm_length_;
  const SpatialForce<double> F_A_W_expected(
      Vector3<double>(0, 0, -expected_torque_magnitude),
      Vector3<double>(expected_force_magnitude, 0, 0));
  EXPECT_TRUE(CompareMatrices(
      F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
}

// Verify forces computation when the spring is at its rest length (zero spring
// force) but it is expanding/compressing and therefore damping is non-zero.
TEST_F(SpringDamperTester, NonZeroVelocity) {
  // The spring is stretching.
  const double length_dot = 1.0;
  // We use the rest length for this test so that the spring contribution is
  // zero.
  SetSliderState(rest_length_, length_dot);
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
  EXPECT_TRUE(CompareMatrices(
      F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));

  // Spring is compressing.
  SetSliderState(rest_length_, -length_dot);
  CalcSpringDamperForces();
  EXPECT_TRUE(CompareMatrices(
      F_A_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      F_B_W.get_coeffs(), F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
}

// This test verifies the computation of both conservative and non-conservative
// powers for a configuration when they are non-zero.
TEST_F(SpringDamperTester, Power) {
  const double length = 2.0;
  const double length_dot = 1.0;
  SetSliderState(length, length_dot);

  const double conservative_power =
      spring_damper_->CalcConservativePower(*mbt_context_, *pc_, *vc_);
  const double conservative_power_expected =
      -stiffness_ * (length - rest_length_) * length_dot;
  EXPECT_NEAR(conservative_power, conservative_power_expected, kTolerance);

  const double non_conservative_power =
      spring_damper_->CalcNonConservativePower(*mbt_context_, *pc_, *vc_);
  const double non_conservative_power_expected =
      -damping_ * length_dot * length_dot;
  // It should always be non-positive.
  EXPECT_LT(non_conservative_power, 0.0);
  EXPECT_NEAR(non_conservative_power,
              non_conservative_power_expected, kTolerance);
}

}  // namespace
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
