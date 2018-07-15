#include "drake/multibody/multibody_tree/spring_damper.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
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
  }

  void SetSliderPosition(double position) {
    slider_->set_translation(context_.get(), position);
    // Update the kinematics cache.
    model_.CalcPositionKinematicsCache(*context_, pc_.get());
    model_.CalcVelocityKinematicsCache(*context_, *pc_, vc_.get());
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
  
  // Parameters of the case.
  const Vector3<double> p_AP_{1, 0, 0};
  const Vector3<double> p_BQ_{0, 0, 0};

  const double rest_length_ = 1.0;  // [m]
  const double stiffness_ = 2.5;    // [N/m]
  const double damping_ = 0.5;      // [Ns/m]
};

//constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

TEST_F(SpringDamperTester, ConstructionAndAccessors) {
  EXPECT_EQ(spring_damper_->bodyA().index(), bodyA_->index());
  EXPECT_EQ(spring_damper_->bodyB().index(), bodyB_->index());
  EXPECT_EQ(spring_damper_->stiffness(), stiffness_);
  EXPECT_EQ(spring_damper_->damping(), damping_);
  EXPECT_EQ(spring_damper_->rest_length(), rest_length_);
  EXPECT_EQ(spring_damper_->point_on_bodyA(), p_AP_);
  EXPECT_EQ(spring_damper_->point_on_bodyB(), p_BQ_);
}

// Test rigid body constructor.
TEST_F(SpringDamperTester, LengthLongerThanResingLength) {
  SetSliderPosition(3.0);

  MultibodyForces<double> forces(model_);
  spring_damper_->CalcAndAddForceContribution(
      *mbt_context_, *pc_, *vc_, &forces);

  const SpatialForce<double>& F_A_W =
      forces.body_forces().at(bodyA_->node_index());
  const SpatialForce<double>& F_B_W =
      forces.body_forces().at(bodyB_->node_index());

  PRINT_VAR(F_A_W);
  PRINT_VAR(F_B_W);
}

}  // namespace
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
