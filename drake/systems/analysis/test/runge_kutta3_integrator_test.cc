#include "drake/systems/analysis/runge_kutta3_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

class RK3IntegratorTest : public ::testing::Test {
 public:
  RK3IntegratorTest() {
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
  std::unique_ptr<RungeKutta3Integrator<double>> integrator_;
  const double dt = 1e-3;        // Integration step size.
  const double big_dt = 1e-1;    // Big integration step size.
  const double spring_k = 300.0;  // N/m
  const double mass = 2.0;      // kg
};

TEST_F(RK3IntegratorTest, ReqAccuracy) {
  // Set the accuracy.
  integrator_->request_initial_step_size_target(dt);

  EXPECT_EQ(integrator_->get_initial_step_size_target(), dt);
}

TEST_F(RK3IntegratorTest, ContextAccess) {
  integrator_->get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator_->get_context().get_time(), 3.);
  EXPECT_EQ(context_->get_time(), 3.);

  // Reset the context time.
  integrator_->get_mutable_context()->set_time(0.);
}

/// Verifies error estimation is supported.
TEST_F(RK3IntegratorTest, ErrorEstSupport) {
  EXPECT_GE(integrator_->get_error_estimate_order(), 1);
  EXPECT_EQ(integrator_->supports_error_estimation(), true);
  EXPECT_NO_THROW(integrator_->set_target_accuracy(1e-1));
  EXPECT_NO_THROW(integrator_->request_initial_step_size_target(dt));
}

// Verifies that the stepping works with large magnitude times and small
// magnitude step sizes.
TEST_F(RK3IntegratorTest, MagDisparity) {
  // Set a large magnitude time.
  context_->set_time(1.0);

  // Set integrator parameters.
  integrator_->set_maximum_step_size(0.1);
  integrator_->set_minimum_step_size(1e-40);
  integrator_->set_target_accuracy(1e-3);

  // Take all the defaults.
  integrator_->Initialize();

  // Take a variable step.
  integrator_->StepExactlyVariable(1e-40);
}

// Test scaling vectors
TEST_F(RK3IntegratorTest, Scaling) {
  // Setting maximum integrator step size is necessary to prevent integrator
  // from throwing an exception.
  integrator_->set_maximum_step_size(big_dt);

  // Initialize the integrator to set weight vector sizes.
  integrator_->Initialize();

  // Test scaling
  EXPECT_EQ(integrator_->get_mutable_generalized_state_weight_vector().size(),
            1);
  EXPECT_EQ(integrator_->get_mutable_generalized_state_weight_vector()
                .lpNorm<Eigen::Infinity>(),
            1);
  EXPECT_EQ(integrator_->get_misc_state_weight_vector().size(), 1);
  EXPECT_EQ(integrator_->get_mutable_misc_state_weight_vector()
                .lpNorm<Eigen::Infinity>(),
            1);
}

// Tests the ability to setup the integrator robustly (i.e., with minimal
// user knowledge); in other words, if the user fails to set some aspect of the
// integrator properly, will NaN values make it run forever?
TEST_F(RK3IntegratorTest, BulletProofSetup) {
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

// Tests the error estimation capabilities.
TEST_F(RK3IntegratorTest, ErrEst) {
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

  // Set integrator parameters: do no error control.
  integrator_->set_maximum_step_size(big_dt);
  integrator_->set_fixed_step_mode(true);

  // Initialize the integrator.
  integrator_->Initialize();

  // Take a single step of size big_dt.
  integrator_->StepOnceAtMost(big_dt, big_dt, big_dt);

  // Verify that a step of big_dt was taken.
  EXPECT_NEAR(context_->get_time(), big_dt,
              std::numeric_limits<double>::epsilon());

  // Get the true solution
  const double x_true =
      c1 * std::cos(omega * big_dt) + c2 * std::sin(omega * big_dt);

  // Get the integrator's solution
  const double kXApprox = context_->get_continuous_state_vector().GetAtIndex(0);

  // Get the error estimate
  const double err_est =
      integrator_->get_error_estimate()->get_vector().GetAtIndex(0);

  // Verify that difference between integration result and true result is
  // captured by the error estimate. The 0.2 below indicates that the error
  // estimate is quite conservative. (That's because we estimate the error in
  // the 2nd order integral but propagate the 3rd order integral which is
  // generally more accurate.)
  EXPECT_NEAR(kXApprox, x_true, err_est * 0.2);
}

// Integrate a purely continuous system with no sampling using error control.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(RK3IntegratorTest, SpringMassStepEC) {
  // Set integrator parameters: do no error control.
  integrator_->set_maximum_step_size(dt);
  integrator_->set_fixed_step_mode(true);

  // Initialize the integrator.
  integrator_->Initialize();

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

  // StepOnceAtFixedSize for 1 second.
  const double t_final = 1.0;
  for (double t = 0.0; t < t_final; t += dt)
    integrator_->StepOnceAtMost(dt, dt, dt);

  // Get the final position.
  const double x_final =
      context_->get_continuous_state()->get_vector().GetAtIndex(0);

  // Store the number of integration steps
  int fixed_steps = integrator_->get_num_steps_taken();

  // Check the solution.
  EXPECT_NEAR(
      c1 * std::cos(omega * t_final) + c2 * std::sin(omega * t_final),
      x_final, 1e-5);

  // Reset the integrator and set reasonable parameters for integration with
  // error control.
  integrator_->Reset();
  integrator_->set_maximum_step_size(0.1);
  integrator_->set_minimum_step_size(1e-6);
  integrator_->set_target_accuracy(1e-3);

  // Re-initialize the integrator.
  integrator_->Initialize();

  // Set initial condition using the Simulator's internal Context.
  integrator_->get_mutable_context()->set_time(0.);
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             initial_position);
  spring_mass_->set_velocity(integrator_->get_mutable_context(),
                             initial_velocity);

  // StepOnceAtFixedSize for 1 second.
  double t_remaining = t_final - context_->get_time();
  do {
    integrator_->StepOnceAtMost(t_remaining, t_remaining, t_remaining);
    t_remaining = t_final - context_->get_time();
  } while (t_remaining > 0.0);

  // Check the solution.
  EXPECT_NEAR(
      c1 * std::cos(omega * t_final) + c2 * std::sin(omega * t_final),
      x_final, 1e-5);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator_->get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator_->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator_->get_smallest_adapted_step_size_taken(), 0.0);
  EXPECT_GE(integrator_->get_num_steps_taken(), 0);
  EXPECT_NE(integrator_->get_error_estimate(), nullptr);

  // Verify that less computation was performed compared to the fixed step
  // integrator.
  EXPECT_LT(integrator_->get_num_steps_taken(), fixed_steps);
}

// Verify that attempting to take a single fixed step throws an exception.
TEST_F(RK3IntegratorTest, IllegalFixedStep) {
  // Set integrator parameters: do error control.
  integrator_->set_maximum_step_size(dt);
  integrator_->set_fixed_step_mode(false);

  // Set accuracy to a really small value so that the step is guaranteed to be
  // small.
  integrator_->set_target_accuracy(1e-8);

  // Initialize the integrator.
  integrator_->Initialize();

  EXPECT_THROW(integrator_->StepExactlyFixed(1e-8), std::logic_error);
}

// Verifies statistics validity for error controlled integrator.
TEST_F(RK3IntegratorTest, CheckStat) {
  // Set integrator parameters: do error control.
  integrator_->set_maximum_step_size(dt);
  integrator_->set_fixed_step_mode(false);

  // Set accuracy to a really small value so that the step is guaranteed to be
  // small.
  integrator_->set_target_accuracy(1e-8);

  // Initialize the integrator.
  integrator_->Initialize();

  // Setup the initial position and initial velocity
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             initial_position);
  spring_mass_->set_velocity(integrator_->get_mutable_context(),
                             initial_velocity);

  // Integrate just one step.
  integrator_->StepOnceAtMost(dt, dt, dt);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator_->get_previous_integration_step_size(), 0.0);
  EXPECT_LE(integrator_->get_previous_integration_step_size(), dt);
  EXPECT_LE(integrator_->get_smallest_adapted_step_size_taken(), dt);
}

// Tests accuracy when generalized velocity is not the time derivative of
// generalized configuration (using a rigid body).
GTEST_TEST(RK3RK2IntegratorTest, RigidBody) {
  // Instantiates a Multibody Dynamics (MBD) model of the world.
  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Add a single free body with a quaternion base.
  RigidBody<double>* body;
  tree->add_rigid_body(
      std::unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
  body->set_name("free_body");

  // Sets body to have a non-zero spatial inertia. Otherwise the body gets
  // welded by a fixed joint to the world by RigidBodyTree::compile().
  body->set_mass(1.0);
  body->set_spatial_inertia(Matrix6<double>::Identity());
  body->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
      "base", Eigen::Isometry3d::Identity()));

  tree->compile();

  // Instantiates a RigidBodyPlant from the  MBD model.
  RigidBodyPlant<double> plant(move(tree));
  auto context = plant.CreateDefaultContext();

  Eigen::Vector3d v0(1, 2, 3);    // Linear velocity in body's frame.
  Eigen::Vector3d w0(-4, 5, -6);  // Angular velocity in body's frame.
  BasicVector<double> generalized_velocities(plant.get_num_velocities());
  generalized_velocities.get_mutable_value() << w0, v0;

  // Set the linear and angular velocity.
  for (int i=0; i< plant.get_num_velocities(); ++i)
    plant.set_velocity(context.get(), i, generalized_velocities[i]);

  // Set a non-identity position and orientation.
  plant.set_position(context.get(), 0, 1.0);  // Set position to (1,2,3).
  plant.set_position(context.get(), 1, 2.0);
  plant.set_position(context.get(), 2, 3.0);
  plant.set_position(context.get(), 3, std::sqrt(2)/2);  // Set orientation to
  plant.set_position(context.get(), 4, 0.0);             // 90 degree rotation
  plant.set_position(context.get(), 5, std::sqrt(2)/2);  // about y-axis.
  plant.set_position(context.get(), 6, 0.0);

  // Integrate for ten thousand steps using a RK2 integrator with
  // small step size.
  const double dt = 5e-5;
  RungeKutta2Integrator<double> rk2(plant, dt, context.get());
  rk2.Initialize();
  const double t_final = 1.0;
  const int n_steps = t_final / dt;
  for (int i = 0; i< n_steps; ++i)
    rk2.StepExactlyFixed(dt);

  // Get the final state.
  VectorX<double> x_final_rk2 = context->get_continuous_state_vector().
      CopyToVector();

  // Re-integrate with RK3
  context->set_time(0.);
  plant.SetDefaultState(*context, context->get_mutable_state());
  for (int i=0; i< plant.get_num_velocities(); ++i)
    plant.set_velocity(context.get(), i, generalized_velocities[i]);
  // Reset the non-identity position and orientation.
  plant.set_position(context.get(), 0, 1.0);  // Set position to (1,2,3).
  plant.set_position(context.get(), 1, 2.0);
  plant.set_position(context.get(), 2, 3.0);
  plant.set_position(context.get(), 3, std::sqrt(2)/2);  // Set orientation to
  plant.set_position(context.get(), 4, 0.0);             // 90 degree rotation
  plant.set_position(context.get(), 5, std::sqrt(2)/2);  // about y-axis.
  plant.set_position(context.get(), 6, 0.0);
  RungeKutta3Integrator<double> rk3(plant, context.get());
  rk3.set_maximum_step_size(0.1);
  rk3.set_target_accuracy(1e-6);
  rk3.Initialize();

  // Verify that StepExactlyVariable works.
  const double tol = std::numeric_limits<double>::epsilon();
  rk3.StepExactlyVariable(t_final - context->get_time());
  EXPECT_NEAR(context->get_time(), t_final, tol);

  // Verify that the final states are "close".
  VectorX<double> x_final_rk3 = context->get_continuous_state_vector().
      CopyToVector();
  const double close_tol = 2e-6;
  for (int i=0; i< x_final_rk2.size(); ++i)
    EXPECT_NEAR(x_final_rk2[i], x_final_rk3[i], close_tol);
}

}  // namespace
}  // namespace systems
}  // namespace drake
