#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace systems {
namespace analysis_test {

// T is the integrator type (e.g., RungeKutta3Integrator<double>).
template <class T>
struct GenericIntegratorTest : public ::testing::Test {
 public:
  void SetUp() {
    plant_ = std::make_unique<multibody::MultibodyPlant<double>>(0.0);

    // Add a single free body to the world.
    const double radius = 0.05;  // m
    const double mass = 0.1;     // kg
    auto G_Bcm = multibody::UnitInertia<double>::SolidSphere(radius);
    multibody::SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(),
                                            G_Bcm);
    plant_->AddRigidBody("Ball", M_Bcm);
    plant_->Finalize();

    context_ = MakePlantContext();
    integrator_ = std::make_unique<T>(*plant_, context_.get());
  }

  std::unique_ptr<Context<double>> MakePlantContext() const {
    std::unique_ptr<Context<double>> context = plant_->CreateDefaultContext();

    // Set body linear and angular velocity.
    Vector3<double> v0(1., 2., 3.);    // Linear velocity in body's frame.
    Vector3<double> w0(-4., 5., -6.);  // Angular velocity in body's frame.
    VectorX<double> generalized_velocities(6);
    generalized_velocities << w0, v0;
    plant_->SetVelocities(context.get(), generalized_velocities);

    // Set body position and orientation.
    Vector3<double> p0(1., 2., 3.);  // Body's frame position in the world.
    // Set body's frame orientation to 90 degree rotation about y-axis.
    Vector4<double> q0(std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2., 0.);
    VectorX<double> generalized_positions(7);
    generalized_positions << q0, p0;
    plant_->SetPositions(context.get(), generalized_positions);

    return context;
  }

  std::unique_ptr<multibody::MultibodyPlant<double>> plant_{};
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<T> integrator_;
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
        this->plant_->GetPositionsAndVelocities(*this->context_),
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

REGISTER_TYPED_TEST_SUITE_P(GenericIntegratorTest, DenseOutput, NegativeTime);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
