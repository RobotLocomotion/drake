#pragma once

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace systems {
namespace analysis_test {

// GenericIntegratorBaseTest contains much of the formal setup for generic
// integrator tests, but it does not put any model elements into the
// MultibodyPlant. It does provide virtual methods for derived fixtures to put
// interesting model elements into the plant.
//
// T is the integrator test factory type (e.g.,
// IntegratorTestFactory<RungeKutta3Integrator<double>>).
template <class T>
class GenericIntegratorBaseTest : public ::testing::Test {
 protected:
  void SetUp() {
    auto plant = std::make_unique<multibody::MultibodyPlant<double>>(0.0);

    SetupPlant(plant.get());

    plant->Finalize();

    auto maybe_dut = T::MakeIntegratorTestArticles(std::move(plant));
    if (!maybe_dut) {
      GTEST_SKIP() << maybe_dut.error();
    }
    articles_ = std::move(maybe_dut.value());

    plant_ = articles_.sub_system;
    plant_context_ = articles_.sub_context;
    integrator_ = articles_.integrator.get();
    context_ = articles_.root_context.get();

    SetPlantContext();
  }

  // Derived fixtures can modify the plant model and context.
  virtual void SetupPlant(multibody::MultibodyPlant<double>*) {}
  virtual void SetPlantContext() {}

  // Check a fixed step of integration, with the expectation that no continuous
  // state is expected to change. The caller is expected to provide a
  // no-changes scenario.
  void CheckTrivialFixedStep() {
    integrator_->set_maximum_step_size(0.1);
    integrator_->set_target_accuracy(1e-5);
    integrator_->Initialize();
    integrator_->set_fixed_step_mode(true);
    VectorX<double> state_before =
        context_->get_continuous_state().CopyToVector();
    EXPECT_TRUE(integrator_->IntegrateWithSingleFixedStepToTime(0.1));
    EXPECT_EQ(context_->get_time(), 0.1);
    VectorX<double> state_after =
        context_->get_continuous_state().CopyToVector();
    EXPECT_TRUE(CompareMatrices(state_before, state_after));
  }

  // Check an error-controlled step of integration, with the expectation that
  // no continuous state is expected to change. The caller is expected to
  // provide a no-changes scenario.
  void CheckTrivialErrorControlledStep() {
    integrator_->set_maximum_step_size(0.1);
    integrator_->set_target_accuracy(1e-5);
    integrator_->Initialize();
    VectorX<double> state_before =
        context_->get_continuous_state().CopyToVector();
    integrator_->IntegrateWithMultipleStepsToTime(0.5);
    EXPECT_EQ(context_->get_time(), 0.5);
    VectorX<double> state_after =
        context_->get_continuous_state().CopyToVector();
    EXPECT_TRUE(CompareMatrices(state_before, state_after));
  }

  // `articles_` holds the objects received from the factory and retains
  // ownership.
  IntegratorTestArticles<multibody::MultibodyPlant<double>> articles_;
  // Create aliases into `articles_` for convenience.
  const multibody::MultibodyPlant<double>* plant_{};
  Context<double>* plant_context_{};
  IntegratorBase<double>* integrator_{};
  Context<double>* context_{};
};

TYPED_TEST_SUITE_P(GenericIntegratorBaseTest);

// Confirms that integration supports fixed steps on systems with no continuous
// state. Uses an empty plant.
TYPED_TEST_P(GenericIntegratorBaseTest, TrivialFixedStepEmptyPlant) {
  ASSERT_EQ(this->context_->get_continuous_state().size(), 0);
  this->CheckTrivialFixedStep();
}

// Confirms that integration supports error controlled steps on systems with no
// continuous state. Uses an empty plant.
TYPED_TEST_P(GenericIntegratorBaseTest, TrivialErrorControlledStepEmptyPlant) {
  ASSERT_EQ(this->context_->get_continuous_state().size(), 0);
  this->CheckTrivialErrorControlledStep();
}

// GenericIntegratorTest extends GenericIntegratorBaseTest by adding a single
// free body into the MultibodyPlant. It adds a sphere with mass, and sets some
// non-trivial initial state.
//
// T is the integrator test factory type (e.g.,
// IntegratorTestFactory<RungeKutta3Integrator<double>>).
template <class T>
class GenericIntegratorTest : public GenericIntegratorBaseTest<T> {
 protected:
  void SetupPlant(multibody::MultibodyPlant<double>* plant) override {
    // Add a single free body to the world.
    const double radius = 0.05;  // m
    const double mass = 0.1;     // kg
    multibody::SpatialInertia<double> M_BBcm =
        multibody::SpatialInertia<double>::SolidSphereWithMass(mass, radius);

    plant->AddRigidBody("Ball", M_BBcm);
  }

  void SetPlantContext() override {
    // Set body linear and angular velocity.
    Vector3<double> v0(1., 2., 3.);    // Linear velocity in body's frame.
    Vector3<double> w0(-4., 5., -6.);  // Angular velocity in body's frame.
    VectorX<double> generalized_velocities(6);
    generalized_velocities << w0, v0;
    this->plant_->SetVelocities(this->plant_context_, generalized_velocities);

    // Set body position and orientation.
    Vector3<double> p0(1., 2., 3.);  // Body's frame position in the world.
    // Set body's frame orientation to 90 degree rotation about y-axis.
    Vector4<double> q0(std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2., 0.);
    VectorX<double> generalized_positions(7);
    generalized_positions << q0, p0;
    this->plant_->SetPositions(this->plant_context_, generalized_positions);
  }
};

TYPED_TEST_SUITE_P(GenericIntegratorTest);

// Verifies that the dense output is working for an integrator.
TYPED_TEST_P(GenericIntegratorTest, DenseOutput) {
  this->integrator_->set_maximum_step_size(0.1);

  // An accuracy that should be achievable with all integrators.
  this->integrator_->set_target_accuracy(1e-5);
  this->integrator_->Initialize();

  // Start a dense integration i.e. one that generates a dense
  // output for the state function.
  this->integrator_->StartDenseIntegration();

  const double t_final = 1.0;
  // Arbitrary step, valid as long as it doesn't match the same
  // steps taken by the integrator. Otherwise, dense output accuracy
  // would not be checked.
  const double h = 0.01;
  const int n_steps = (t_final / h);
  for (int i = 1; i <= n_steps; ++i) {
    // Integrate the whole step.
    this->integrator_->IntegrateWithMultipleStepsToTime(i * h);

    // Check solution.
    EXPECT_TRUE(CompareMatrices(
        this->integrator_->get_dense_output()->value(
            this->context_->get_time()),
        this->plant_->GetPositionsAndVelocities(*this->plant_context_),
        this->integrator_->get_accuracy_in_use(), MatrixCompareType::relative));
  }

  // Stop undergoing dense integration.
  std::unique_ptr<trajectories::PiecewisePolynomial<double>> dense_output =
      this->integrator_->StopDenseIntegration();
  EXPECT_FALSE(this->integrator_->get_dense_output());

  // Integrate one more step.
  this->integrator_->IntegrateWithMultipleStepsToTime(t_final + h);

  // Verify that the dense output was not updated.
  EXPECT_LT(dense_output->end_time(), this->context_->get_time());
}

// Confirm that integration supports times < 0.
TYPED_TEST_P(GenericIntegratorTest, NegativeTime) {
  this->integrator_->set_maximum_step_size(0.1);
  this->integrator_->set_target_accuracy(1e-5);
  this->integrator_->Initialize();
  this->context_->SetTime(-1.0);
  this->integrator_->IntegrateWithMultipleStepsToTime(-0.5);
  EXPECT_EQ(this->context_->get_time(), -0.5);
}

// Confirms that integration supports fixed steps on systems with no active
// dofs. Uses joint locking to lock all dofs.
TYPED_TEST_P(GenericIntegratorTest, TrivialFixedStepJointsLocked) {
  for (const multibody::JointIndex& j : this->plant_->GetJointIndices()) {
    this->plant_->get_joint(j).Lock(this->plant_context_);
  }
  ASSERT_NE(this->context_->get_continuous_state().size(), 0);
  this->CheckTrivialFixedStep();
}

// Confirms that integration supports error controlled steps on systems with no
// active dofs. Uses joint locking to lock all dofs.
TYPED_TEST_P(GenericIntegratorTest, TrivialErrorControlledStepJointsLocked) {
  for (const multibody::JointIndex& j : this->plant_->GetJointIndices()) {
    this->plant_->get_joint(j).Lock(this->plant_context_);
  }
  ASSERT_NE(this->context_->get_continuous_state().size(), 0);
  this->CheckTrivialErrorControlledStep();
}

REGISTER_TYPED_TEST_SUITE_P(GenericIntegratorTest, DenseOutput, NegativeTime,
                            TrivialFixedStepJointsLocked,
                            TrivialErrorControlledStepJointsLocked);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
