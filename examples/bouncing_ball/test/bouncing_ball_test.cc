#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace examples {
namespace bouncing_ball {
namespace {

// Computes the drop time from the initial height and the gravitational
// acceleration.
double CalcDropTime(const double g, const double q0) {
  DRAKE_DEMAND(g < 0.0);
  DRAKE_DEMAND(q0 > 0.0);

  // The time that the ball will impact the ground is:
  // gt^2/2 + q0 = 0
  // Solve the quadratic equation at^2 + bt + c = 0 for t.
  const double a = g/2;
  const double c = q0;
  return std::sqrt(-c/a);
}

// Computes the closed form height and velocity at tf seconds for a bouncing
// ball starting from height q0, subject to gravitational acceleration g,
// and with coefficient of restitution e, assuming that the initial velocity
// is zero. Restitution coefficients of 0 and 1 are the only ones supported.
// Returns a pair of values, the first corresponding to the height at tf,
// the second corresponding to the velocity at tf.
std::pair<double, double> CalcClosedFormHeightAndVelocity(double g,
                                                          double e,
                                                          double q0,
                                                          double tf) {
  const double drop_time = CalcDropTime(g, q0);

  // Handle the cases appropriately.
  if (e == 0.0) {
    // TODO(edrumwri): Test these cases when we can handle the Zeno's Paradox
    // problem.
    if (tf < drop_time) {
      // In a ballistic phase.
      return std::make_pair(g*tf*tf/2 + q0, g*tf);
    } else {
      // Ball has hit the ground.
      return std::make_pair(0.0, 0.0);
    }
  }

  if (e == 1.0) {
    // Get the number of phases that have passed.
    int num_phases = static_cast<int>(std::floor(tf / drop_time));

    // Get the time within the phase.
    const double t = tf - num_phases*drop_time;

    // Even phases mean that the ball is falling, odd phases mean that it is
    // rising.
    if ((num_phases & 1) == 0) {
      return std::make_pair(g*t*t/2 + q0, g*t);
    } else {
      // Get the ball velocity at the time of impact.
      const double vf = g*drop_time;
      return std::make_pair(g*t*t/2 - vf*t, g*t - vf);
    }
  }

  throw std::logic_error("Invalid restitution coefficient!");
}

class BouncingBallTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<BouncingBall<double>>();
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  systems::VectorBase<double>& continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  const systems::VectorBase<double>& generalized_position() {
    return context_->get_continuous_state().get_generalized_position();
  }

  const systems::VectorBase<double>& generalized_velocity() {
    return context_->get_continuous_state().get_generalized_velocity();
  }

  std::unique_ptr<BouncingBall<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

GTEST_TEST(BouncingBall, AutoDiff) {
  BouncingBall<AutoDiffXd> ad_plant;
}

TEST_F(BouncingBallTest, Transmogrification) {
  ASSERT_TRUE(systems::is_autodiffxd_convertible(*dut_));
  ASSERT_TRUE(systems::is_symbolic_convertible(*dut_));
}

TEST_F(BouncingBallTest, Topology) {
  ASSERT_EQ(0, dut_->num_input_ports());

  ASSERT_EQ(1, dut_->num_output_ports());
  const auto& output_port = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_port.get_data_type());
}

TEST_F(BouncingBallTest, Output) {
  // Grab a pointer to where the CalcOutput results will be saved.
  const auto result = output_->get_vector_data(0);

  // Initial state and output.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(10.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // New state just propagates through.
  continuous_state().SetAtIndex(0, 1.0);
  continuous_state().SetAtIndex(1, 2.0);
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->GetAtIndex(0));
  EXPECT_EQ(2.0, result->GetAtIndex(1));
}

TEST_F(BouncingBallTest, Derivatives) {
  // Grab a pointer to where the EvalTimeDerivatives results will be saved.
  const auto& result = derivatives_->get_mutable_vector();

  // Evaluate time derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result.GetAtIndex(0));
  EXPECT_EQ(-9.81, result.GetAtIndex(1));

  // Test at non-zero velocity.
  continuous_state().SetAtIndex(1, 5.3);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(5.3, result.GetAtIndex(0));
  EXPECT_EQ(-9.81, result.GetAtIndex(1));
}

TEST_F(BouncingBallTest, Accessors) {
  // Evaluate accessors specific to the second-order system.
  EXPECT_EQ(10.0, generalized_position().GetAtIndex(0));
  EXPECT_EQ(0, generalized_velocity().GetAtIndex(0));
}

TEST_F(BouncingBallTest, Simulate) {
  // Small errors from time-of-impact isolation tolerances propagate for this
  // particular instance of the problem (with restitution coefficient of 1).
  // Drake's integrators control local (truncation) rather than global
  // (i.e., solution to the initial value problem) error. This means that the
  // number of digits of precision obtained will not be equal to the digits of
  // precision requested (via the accuracy setting) for longer running times
  // than t_final = 10.0.
  const double t_final = 10.0;
  const double q0 = 1.0;
  const double v0 = 0.0;
  const double accuracy = 1e-4;

  // Prepare to integrate.
  // TODO(edrumwri): Update the code below when accuracy is settable purely
  // from the context.
  drake::systems::Simulator<double> simulator(*dut_, std::move(context_));
  simulator.reset_integrator<systems::RungeKutta3Integrator<double>>();
  simulator.get_mutable_context().SetAccuracy(accuracy);
  simulator.get_mutable_integrator().request_initial_step_size_target(1e-3);
  simulator.get_mutable_integrator().set_target_accuracy(accuracy);

  // Note: The bouncing ball's witness function is triggered when the ball's
  // height is positive then non-positive. As shown below, the maximum step size
  // is limited to avoid missing a subsequent bounce in the following situation:
  // 1. Bounce detected (previous height positive, current height negative).
  // 2. Impact reverses ball's downward velocity to an upward velocity.
  // 3. Integrator advances time too far so that the next time the witness
  // function is called, the ball's height is again negative.
  // 4. Since the witness function sees two subsequent negative heights, the
  // witness function is not triggered (so subsequent bounce is missed).
  // This situation arises when:
  // a. an integrator can simulate parabolic trajectory without error (e.g.,
  // 2nd-order integrator) which means the time step can be large, or
  // b. a semi-explicit 1st-order integrator has large error tolerances that
  // allow for a large time step and two successive negative heights.
  ASSERT_EQ(dut_->get_restitution_coef(), 1.0);
  const double g = dut_->get_gravitational_acceleration();
  const double drop_time = CalcDropTime(g, q0);
  simulator.get_mutable_integrator().set_maximum_step_size(drop_time);

  simulator.Initialize();

  // Set the initial state for the bouncing ball.
  systems::VectorBase<double>& xc = simulator.get_mutable_context().
      get_mutable_continuous_state_vector();
  xc.SetAtIndex(0, q0);
  xc.SetAtIndex(1, v0);

  // Integrate.
  simulator.AdvanceTo(t_final);
  EXPECT_EQ(simulator.get_mutable_context().get_time(), t_final);

  // Check against closed form solution for the bouncing ball. We anticipate
  // some small integration error.
  const double tol = accuracy;
  double height, velocity;
  std::tie(height, velocity) = CalcClosedFormHeightAndVelocity(
      dut_->get_gravitational_acceleration(),
      dut_->get_restitution_coef(), q0, t_final);
  EXPECT_NEAR(xc.GetAtIndex(0), height, tol);
  EXPECT_NEAR(xc.GetAtIndex(1), velocity, tol);
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
