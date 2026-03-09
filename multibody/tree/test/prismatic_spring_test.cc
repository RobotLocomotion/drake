#include "drake/multibody/tree/prismatic_spring.h"

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;

namespace multibody {
namespace internal {
namespace {

constexpr double kTolerance = std::numeric_limits<double>::epsilon();

class SpringTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    bodyA_ = &model->AddRigidBody("BodyA", SpatialInertia<double>::NaN());
    bodyB_ = &model->AddRigidBody("BodyB", SpatialInertia<double>::NaN());

    model->AddJoint<WeldJoint>("WeldBodyAToWorld", model->world_body(), {},
                               *bodyA_, {},
                               math::RigidTransform<double>::Identity());

    // Allow body B to translate along the z axis.
    joint_ = &model->AddJoint<PrismaticJoint>("joint_AB", *bodyA_, std::nullopt,
                                              *bodyB_, std::nullopt,
                                              Vector3<double>::UnitZ());

    // Add spring
    spring_ = &model->AddForceElement<PrismaticSpring>(
        *joint_, nominal_position_, stiffness_);

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = system_->CreateDefaultContext();

    forces_ = std::make_unique<MultibodyForces<double>>(tree());
  }

  void SetJointState(double position, double position_rate) {
    joint_->set_translation(context_.get(), position);
    joint_->set_translation_rate(context_.get(), position_rate);
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
  const PrismaticJoint<double>* joint_{nullptr};
  const PrismaticSpring<double>* spring_{nullptr};
  std::unique_ptr<MultibodyForces<double>> forces_;

  // Parameters of the case.
  const double nominal_position_ = 1.0;  // [m]
  const double stiffness_ = 2.0;         // [N/m]
};

TEST_F(SpringTester, ConstructionAndAccessors) {
  EXPECT_EQ(spring_->joint().index(), joint_->index());
  EXPECT_EQ(spring_->stiffness(), stiffness_);
  EXPECT_EQ(spring_->nominal_position(), nominal_position_);
}

// Verify the spring applies no forces when the separation equals the
// nominal position.
TEST_F(SpringTester, NominalPosition) {
  SetJointState(1.0, 0.0);
  CalcSpringForces();
  const VectorX<double>& generalized_forces = forces_->generalized_forces();
  EXPECT_TRUE(CompareMatrices(generalized_forces, VectorX<double>::Zero(1),
                              kTolerance, MatrixCompareType::relative));

  // Verify the potential energy is zero.
  const double potential_energy = spring_->CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, 0.0, kTolerance);
}

// Verify forces computation when the spring position differs from the nominal.
TEST_F(SpringTester, DeltaPosition) {
  const double position = 2.0;
  SetJointState(position, 0.0);
  CalcSpringForces();
  const VectorX<double>& generalized_forces = forces_->generalized_forces();

  const double expected_force_magnitude =
      stiffness_ * (nominal_position_ - position);
  VectorX<double> expected_generalized_forces(1);
  expected_generalized_forces << expected_force_magnitude;
  EXPECT_TRUE(CompareMatrices(generalized_forces, expected_generalized_forces,
                              kTolerance, MatrixCompareType::relative));

  // Verify the value of the potential energy.
  const double potential_energy_expected = 0.5 * stiffness_ *
                                           (position - nominal_position_) *
                                           (position - nominal_position_);
  const double potential_energy = spring_->CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, potential_energy_expected, kTolerance);

  // Since the spring configuration is static, that is velocities are zero, we
  // expect zero conservative and non-conservative power.
  const double conservative_power = spring_->CalcConservativePower(
      *context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  EXPECT_NEAR(conservative_power, 0.0, kTolerance);
  const double non_conservative_power = spring_->CalcNonConservativePower(
      *context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  EXPECT_NEAR(non_conservative_power, 0.0, kTolerance);
}

// This test verifies the computation of both conservative and non-conservative
// powers for a configuration when they are non-zero.
TEST_F(SpringTester, Power) {
  const double position = 2.0;
  const double position_dot = 1.7;
  SetJointState(position, position_dot);

  const double conservative_power = spring_->CalcConservativePower(
      *context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  const double conservative_power_expected =
      -stiffness_ * (position - nominal_position_) * position_dot;
  EXPECT_NEAR(conservative_power, conservative_power_expected, kTolerance);

  const double non_conservative_power = spring_->CalcNonConservativePower(
      *context_, tree().EvalPositionKinematics(*context_),
      tree().EvalVelocityKinematics(*context_));
  const double non_conservative_power_expected = 0;
  // It should always be zero.
  EXPECT_EQ(non_conservative_power, non_conservative_power_expected);
}

TEST_F(SpringTester, Clone) {
  /* Clone the entire model, which should also clone the prismatic spring in the
   model. */
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  /* ForceElementIndex 0 corresponds to gravity and 1 corresponds to the only
   force element we have, which is the PrismaticSpring. */
  const auto& force_element_clone =
      model_clone->get_force_element(ForceElementIndex(1));
  const PrismaticSpring<AutoDiffXd>* spring_clone1 =
      dynamic_cast<const PrismaticSpring<AutoDiffXd>*>(&force_element_clone);
  ASSERT_NE(spring_clone1, nullptr);

  /* Clone just the element (shallow), once more. */
  std::unique_ptr<ForceElement<AutoDiffXd>> shallow =
      spring_clone1->ShallowClone();
  const PrismaticSpring<AutoDiffXd>* spring_clone2 =
      dynamic_cast<const PrismaticSpring<AutoDiffXd>*>(shallow.get());
  ASSERT_NE(spring_clone2, nullptr);

  /* Verify all quantities are truthfully copied. */
  for (const auto* clone : {spring_clone1, spring_clone2}) {
    if (clone != spring_clone2) {
      EXPECT_EQ(spring_->joint().index(), clone->joint().index());
    }
    EXPECT_EQ(spring_->stiffness(), clone->stiffness());
    EXPECT_EQ(spring_->nominal_position(), clone->nominal_position());
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
