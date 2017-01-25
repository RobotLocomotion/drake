#include <cmath>
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/half_explicit_DAE1_solver.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

class HalfExplicitDAE1SolverTest : public ::testing::Test {
 public:
  HalfExplicitDAE1SolverTest() {
    // Create a mass-spring-system with update rate=0.
    spring_mass_ = std::make_unique<analysis_test::MySpringMassSystem<double>>(
        spring_k, mass, 0.);
    context_ = spring_mass_->CreateDefaultContext();

    // Create and initialize the integrator.
    integrator_ = std::make_unique<RungeKutta3Integrator<double>>(
        *spring_mass_, context_.get());  // Use default Context.
  }

  std::unique_ptr<analysis_test::MySpringMassSystem<double>> spring_mass_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<HalfExplicitDAE1Solver<double>> integrator_;
  const double dt = 1e-3;        // Integration step size.
  const double big_dt = 1e-1;    // Big integration step size.
  const double spring_k = 300.0;  // N/m
  const double mass = 2.0;      // kg
};

/*
TEST_F(HalfExplicitDAE1SolverTest, ContextAccess) {
  integrator_->get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator_->get_context().get_time(), 3.);
  EXPECT_EQ(context_->get_time(), 3.);

  // Reset the context time.
  integrator_->get_mutable_context()->set_time(0.);
}

// Tests the ability to setup the integrator robustly (i.e., with minimal
// user knowledge); in other words, if the user fails to set some aspect of the
// integrator properly, will NaN values make it run forever?
TEST_F(HalfExplicitDAE1SolverTest, BulletProofSetup) {
  // Setup the initial position and initial velocity
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(spring_k / mass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             initial_position);
  spring_mass_->set_velocity(integrator_->get_mutable_context(),
                             initial_velocity);

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Attempt to initialize the integrator: should throw logic error because
  // neither maximum step size nor target accuracy has been set.
  EXPECT_THROW(integrator_->Initialize(), std::logic_error);

  // Attempt to initialize the integrator: should throw logic error because
  // maximum step size smaller than minimum step size.
  integrator_->set_maximum_step_size(dt);
  integrator_->set_minimum_step_size(big_dt);
  EXPECT_THROW(integrator_->Initialize(), std::logic_error);

  // Set step sizes to cogent values and try to initialize again but now using
  // bad requested initial step sizes.
  integrator_->set_minimum_step_size(1e-8);
  integrator_->set_maximum_step_size(big_dt);
  integrator_->request_initial_step_size_target(1e-10);
  EXPECT_THROW(integrator_->Initialize(), std::logic_error);
  integrator_->request_initial_step_size_target(big_dt*2.0);

  // Set the accuracy to something too loose, set the maximum step size and
  // try again. Integrator should now silently adjust the target accuracy to
  // the in-use accuracy.
  integrator_->request_initial_step_size_target(dt);
  integrator_->set_target_accuracy(10.0);
  integrator_->Initialize();
  EXPECT_LE(integrator_->get_accuracy_in_use(),
            integrator_->get_target_accuracy());

  // Integrate for 1 second using variable stepping.
  const double t_final = 1.0;
  double t_remaining = t_final - context_->get_time();
  do {
    integrator_->StepOnceAtMost(t_remaining, t_remaining, t_remaining);
    t_remaining = t_final - context_->get_time();
  } while (t_remaining > 0.0);

  // Get the final position.
  const double x_final =
      context_->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution. We're not really looking for accuracy here, just
  // want to make sure that the value is finite.
  EXPECT_NEAR(
      c1 * std::cos(omega * t_final) + c2 * std::sin(omega * t_final),
      x_final, 1e0);
}
*/

}  // namespace
}  // namespace systems
}  // namespace drake
