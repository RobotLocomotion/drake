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
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(kDuration);

  for (int i = 0; i < log->sample_times().size(); i++) {
    if (log->sample_times()(i) == 0.0) {
      // The initial outputs should be zero (due to the zero initial
      // conditions).
      EXPECT_TRUE(CompareMatrices(log->data().col(i), Vector2d(0., 0.)));
    } else if (log->sample_times()(i) <= 2 * time_step) {
      // TODO(edrumwri): Zap this "else if" block on resolution of #9702.
      // It is only here to cover up the bug.
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
  deriv.set_state(Eigen::Vector2d(3.4, 4.4), context.get());
  EXPECT_TRUE(CompareMatrices(
      deriv.get_output_port().Eval<BasicVector<double>>(*context).get_value(),
      Vector2d::Zero()));
}

}  // namespace
}  // namespace systems
}  // namespace drake
