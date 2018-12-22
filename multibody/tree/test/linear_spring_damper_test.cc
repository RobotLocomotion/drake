#include "drake/multibody/tree/linear_spring_damper.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
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
namespace internal {
namespace {

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

class SpringDamperTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    bodyA_ = &model->AddRigidBody("BodyA", SpatialInertia<double>());
    bodyB_ = &model->AddRigidBody("BodyB", SpatialInertia<double>());

    model->AddJoint<WeldJoint>(
        "WeldBodyAToWorld", model->world_body(), {}, *bodyA_, {},
        Isometry3<double>::Identity());

    // Allow body B to slide along the x axis.
    slider_ = &model->AddJoint<PrismaticJoint>(
        "Slider", model->world_body(), {}, *bodyB_, {},
        Vector3<double>::UnitX());

    spring_damper_ = &model->AddForceElement<LinearSpringDamper>(
        *bodyA_, p_AP_, *bodyB_, p_BQ_, free_length_, stiffness_, damping_);

    // Verify the the constructor for the spring-damper throws if either the
    // rest length, stiffness or damping are negative numbers.
    DRAKE_EXPECT_THROWS_MESSAGE(
        model->AddForceElement<LinearSpringDamper>(
            *bodyA_, p_AP_, *bodyB_, p_BQ_,
            -1.0 /* negative rest length */, stiffness_, damping_),
        std::exception,
        ".*condition 'free_length > 0' failed.*");

    DRAKE_EXPECT_THROWS_MESSAGE(
        model->AddForceElement<LinearSpringDamper>(
            *bodyA_, p_AP_, *bodyB_, p_BQ_,
            free_length_, -1.0  /* negative stiffness */, damping_),
        std::exception,
        ".*condition 'stiffness >= 0' failed.*");

    DRAKE_EXPECT_THROWS_MESSAGE(
        model->AddForceElement<LinearSpringDamper>(
            *bodyA_, p_AP_, *bodyB_, p_BQ_,
            free_length_, stiffness_, -1.0 /* negative damping */),
        std::exception,
        ".*condition 'damping >= 0' failed.*");

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = system_->CreateDefaultContext();

    mbt_context_ = dynamic_cast<MultibodyTreeContext<double>*>(context_.get());
    ASSERT_TRUE(mbt_context_ != nullptr);

    context_->EnableCaching();

    forces_ = std::make_unique<MultibodyForces<double>>(tree());
  }

  void SetSliderState(double position, double position_rate) {
    slider_->set_translation(context_.get(), position);
    slider_->set_translation_rate(context_.get(), position_rate);
  }

  void CalcSpringDamperForces() const {
    forces_->SetZero();
    spring_damper_->CalcAndAddForceContribution(
        *mbt_context_, tree().EvalPositionKinematics(*context_),
        tree().EvalVelocityKinematics(*context_), forces_.get());
  }

  const SpatialForce<double>& GetSpatialForceOnBodyA() const {
    return forces_->body_forces().at(bodyA_->node_index());
  }

  const SpatialForce<double>& GetSpatialForceOnBodyB() const {
    return forces_->body_forces().at(bodyB_->node_index());
  }

  const MultibodyTree<double>& tree() const {
    return GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;
  MultibodyTreeContext<double>* mbt_context_{nullptr};

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
      *mbt_context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, 0.0, kTolerance);
}

TEST_F(SpringDamperTester, LengthApproachesZero) {
  SetSliderState(0.0, 0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      CalcSpringDamperForces(), std::runtime_error,
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
  EXPECT_TRUE(CompareMatrices(
      F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));

  // Verify the value of the potential energy.
  const double potential_energy_expected =
      0.5 * stiffness_ * (length - free_length_) * (length - free_length_);
  const double potential_energy = spring_damper_->CalcPotentialEnergy(
      *mbt_context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, potential_energy_expected, kTolerance);

  // Since the spring configuration is static, that is velocities are zero, we
  // expect zero conservative and non-conservative power.
  const double conservative_power = spring_damper_->CalcConservativePower(
      *mbt_context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
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
  const double expected_force_magnitude = stiffness_ * (length - free_length_);
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
  EXPECT_TRUE(CompareMatrices(
      F_A_W.get_coeffs(), F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      F_B_W.get_coeffs(), -F_A_W_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));

  // Spring is compressing.
  SetSliderState(free_length_, -length_dot);
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

  const double conservative_power = spring_damper_->CalcConservativePower(
      *mbt_context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  const double conservative_power_expected =
      -stiffness_ * (length - free_length_) * length_dot;
  EXPECT_NEAR(conservative_power, conservative_power_expected, kTolerance);

  const double non_conservative_power =
      spring_damper_->CalcNonConservativePower(
          *mbt_context_, tree().EvalPositionKinematics(*context_),
          tree().EvalVelocityKinematics(*context_));
  const double non_conservative_power_expected =
      -damping_ * length_dot * length_dot;
  // It should always be non-positive.
  EXPECT_LT(non_conservative_power, 0.0);
  EXPECT_NEAR(non_conservative_power, non_conservative_power_expected,
              kTolerance);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
