#include "drake/multibody/multibody_tree/spring_damper.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

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
        "Slider", model_.world_body(), {}, *bodyB_, {}, Vector3<double>::UnitX());

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

  void SetSliderPosition(double position) {
    slider_->set_translation(context_.get(), position);
    // Update the kinematics cache.
    model_.CalcPositionKinematicsCache(*context_, pc_.get());
    model_.CalcVelocityKinematicsCache(*context_, *pc_, vc_.get());
  }

  void CalcSpringDamperForces() const {
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
  SetSliderPosition(1.0);
  CalcSpringDamperForces();
  const SpatialForce<double>& F_A_W = GetSpatialForceOnBodyA();
  const SpatialForce<double>& F_B_W = GetSpatialForceOnBodyB();
  EXPECT_EQ(F_A_W.get_coeffs(), SpatialForce<double>::Zero().get_coeffs());
  EXPECT_EQ(F_B_W.get_coeffs(), SpatialForce<double>::Zero().get_coeffs());
}

// Verify forces computation when the spring length is larger than its rest
// length.
TEST_F(SpringDamperTester, LengthLargerThanRestLength) {
  const double length = 2.0;
  SetSliderPosition(length);
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

// Verify forces computation when the spring length is smaller than its rest
// length.
TEST_F(SpringDamperTester, LengthSmallerThanRestLength) {
  const double length = 0.5;
  SetSliderPosition(length);
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


  PRINT_VAR(F_A_W);
  PRINT_VAR(F_B_W);
}

}  // namespace
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
