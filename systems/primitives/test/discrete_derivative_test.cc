#include "drake/systems/primitives/discrete_derivative.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Matrix2d;
using Eigen::Vector2d;

GTEST_TEST(DiscreteDerivativeTest, FirstOrderHold) {
  DiagramBuilder<double> builder;

  const double kDuration = 1.0;
  // Create an input system u(t) = [ 2*t, 3*t ].
  Matrix2d knots;
  // clang-format off
  knots << 4, 4+2,
           5, 5+3;
  // clang-format on
  const auto pp = trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      Vector2d(0., kDuration), knots);
  auto ppsource = builder.AddSystem<TrajectorySource<double>>(pp);

  const int kNumInputs = 2;
  const double time_step = 0.1;
  auto deriv =
      builder.AddSystem<DiscreteDerivative<double>>(kNumInputs, time_step);

  builder.Connect(ppsource->get_output_port(), deriv->get_input_port());
  auto log = LogOutput(deriv->get_output_port(), &builder);
  // Evaluate the outputs at a distinct sampling interval.
  log->set_publish_period(time_step / 3.267);

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(kDuration);

  for (int i = 0; i < log->sample_times().size(); i++) {
    if (log->sample_times()(i) == 0.0) {
      // The initial outputs should be zero (due to the zero initial
      // conditions).
      EXPECT_TRUE(CompareMatrices(log->data().col(i), Vector2d(0., 0.)));
    } else if (log->sample_times()(i) <= time_step) {
      // The outputs should jump for one timestep because u(0) is non-zero.
      EXPECT_TRUE(CompareMatrices(
          log->data().col(i), Vector2d(4. / time_step, 5. / time_step), 1e-12));
    } else {
      // Once time has advanced, outputs should have the steady-state
      // derivatives.
      EXPECT_TRUE(CompareMatrices(log->data().col(i), Vector2d(2, 3), 1e-12));
    }
  }
}

GTEST_TEST(DiscreteDerivativeTest, SetState) {
  const int kNumInputs = 2;
  const double time_step = 0.1;
  DiscreteDerivative<double> deriv(kNumInputs, time_step);

  // Setting the initial state to an arbitrary value results in the
  // derivative output being set to zero.
  auto context = deriv.CreateDefaultContext();
  deriv.set_input_history(context.get(), Eigen::Vector2d(3.4, 4.4));
  EXPECT_TRUE(CompareMatrices(
      deriv.get_output_port().Eval<BasicVector<double>>(*context).get_value(),
      Vector2d::Zero()));
}

GTEST_TEST(StateInterpolatorWithDiscreteDerivativeTest, BasicTest) {
  const int kNumPositions = 2;
  const double time_step = 0.1;
  StateInterpolatorWithDiscreteDerivative<double> position_to_state(
      kNumPositions, time_step);

  EXPECT_EQ(&position_to_state.GetInputPort("position"),
            &position_to_state.get_input_port());
  EXPECT_EQ(&position_to_state.GetOutputPort("state"),
            &position_to_state.get_output_port());

  auto context = position_to_state.CreateDefaultContext();

  const Eigen::Vector2d current_position(0.643, 0.821);
  context->FixInputPort(position_to_state.get_input_port().get_index(),
                        current_position);

  // Use setter that zeros the initial velocity output:
  const Eigen::Vector2d last_position(0.123, 0.456);
  position_to_state.set_initial_position(context.get(), last_position);
  Eigen::Vector4d expected_state;
  expected_state << current_position, Eigen::Vector2d::Zero();
  EXPECT_TRUE(CompareMatrices(position_to_state.get_output_port()
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              expected_state));

  // Use setter that specifies the initial velocity output:
  const Eigen::Vector2d velocity(2.53, -6.2);
  position_to_state.set_initial_state(context.get(), last_position, velocity);
  expected_state << current_position, velocity;
  EXPECT_TRUE(CompareMatrices(position_to_state.get_output_port()
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              expected_state, 1e-14));
}

GTEST_TEST(StateInterpolatorWithDiscreteDerivativeTest, ScalarTypesTest) {
  const int kNumPositions = 2;
  const double time_step = 0.1;
  StateInterpolatorWithDiscreteDerivative<double> position_to_state(
      kNumPositions, time_step);

  auto autodiff = position_to_state.ToAutoDiffXd();
  EXPECT_EQ(autodiff->get_output_port(0).size(), 2*kNumPositions);

  auto symbolic = position_to_state.ToSymbolic();
  EXPECT_EQ(symbolic->get_output_port(0).size(), 2*kNumPositions);
}

}  // namespace
}  // namespace systems
}  // namespace drake
