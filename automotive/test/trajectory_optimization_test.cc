#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/simple_car.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

// Sets up a simple trajectory optimization problem that finds a series
// of DrivingCommand's that takes the SimpleCar from an initial condition
// off the x-axis back to the x-axis.
GTEST_TEST(TrajectoryOptimizationTest, SimpleCarDircolTest) {
  SimpleCar<double> plant;
  auto context = plant.CreateDefaultContext();

  SimpleCarState<double> x0, xf;

  x0.set_x(0.0);
  x0.set_y(1.0);
  x0.set_heading(-.02);
  x0.set_velocity(15.0);  // m/s = ~ 33mph

  const double initial_duration = 30.0;  // seconds
  xf.set_x(x0.x() + initial_duration * x0.velocity());
  xf.set_y(0.0);
  xf.set_heading(0.0);
  xf.set_velocity(x0.velocity());

  const int kNumTimeSamples = 21;

  // The solved trajectory may deviate from the initial guess at a reasonable
  // duration.
  const double kMinimumTimeStep =
                   0.8 * initial_duration / (kNumTimeSamples - 1),
               kMaximumTimeStep =
                   1.2 * initial_duration / (kNumTimeSamples - 1);

  systems::trajectory_optimization::DirectCollocation prog(
      &plant, *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);

  prog.AddEqualTimeIntervalsConstraints();

  const SimpleCarParams<double>* params =
      dynamic_cast<const SimpleCarParams<double>*>(
          &context->get_numeric_parameter(0));
  DRAKE_DEMAND(params != nullptr);

  // Impose limits that are inside the true command limits.
  const double kConstraintSafetyFactor{1.2};

  // TODO(russt): Provide a more elegant and efficient way to write the bounding
  // box constraints below
  // (this generates MANY small bounding box constraints for the
  // MathematicalProgram).

  // Impose input limits from the SimpleCarParams.
  DrivingCommand<symbolic::Expression> input;
  input.SetFromVector(prog.input().cast<symbolic::Expression>());
  prog.AddConstraintToAllKnotPoints(kConstraintSafetyFactor *
                                        input.steering_angle() <=
                                    params->max_abs_steering_angle());
  prog.AddConstraintToAllKnotPoints(-params->max_abs_steering_angle() <=
                                    kConstraintSafetyFactor *
                                        input.steering_angle());
  prog.AddConstraintToAllKnotPoints(kConstraintSafetyFactor *
                                        input.acceleration() <=
                                    params->max_acceleration());
  prog.AddConstraintToAllKnotPoints(-params->max_acceleration() <=
                                    kConstraintSafetyFactor *
                                        input.acceleration());

  // Impose velocity limit from the SimpleCarParams.
  SimpleCarState<symbolic::Expression> state;
  state.SetFromVector(prog.state().cast<symbolic::Expression>());
  prog.AddConstraintToAllKnotPoints(
      kConstraintSafetyFactor * state.velocity() <= params->max_velocity());
  prog.AddConstraintToAllKnotPoints(0 <= state.velocity());

  // Help out IPOPT by putting bounds on everything.
  prog.AddBoundingBoxConstraint(-100, 1000, prog.decision_variables());

  // Fix initial conditions.
  prog.AddLinearConstraint(prog.initial_state() == x0.get_value());

  // Fix final conditions.
  prog.AddLinearConstraint(prog.final_state() == xf.get_value());

  // Cost function: int_0^T [ u'u ] dt.
  prog.AddRunningCost(prog.input().transpose() * prog.input());

  // Initial guess is a straight line from the initial state to the final state.
  auto initial_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_duration}, {x0.get_value(), xf.get_value()});

  prog.SetInitialTrajectory(PiecewisePolynomial<double>(),
                            initial_state_trajectory);
  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Plot the solution.
  // Note: see call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs = prog.GetInputSamples();
  Eigen::MatrixXd states = prog.GetStateSamples();
  common::CallMatlab("plot", states.row(SimpleCarStateIndices::kX),
                     states.row(SimpleCarStateIndices::kY));
  common::CallMatlab("xlabel", "x (m)");
  common::CallMatlab("ylabel", "y (m)");

  // Checks that the input commands found are not too large.
  EXPECT_LE(inputs.row(0).lpNorm<1>(), 0.1);
  EXPECT_LE(inputs.row(1).lpNorm<1>(), 1);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
