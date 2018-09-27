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
  knots << 0., 2.,
           0., 3.;
  // clang-format on
  const auto pp = trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      Vector2d(0., kDuration), knots);
  auto ppsource = builder.AddSystem<TrajectorySource<double>>(pp);

  const int kNumInputs = 2;
  const double time_period = 0.1;
  auto deriv =
      builder.AddSystem<DiscreteDerivative<double>>(kNumInputs, time_period);

  builder.Connect(ppsource->get_output_port(), deriv->get_input_port());
  auto log = LogOutput(deriv->get_output_port(), &builder);

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  simulator.StepTo(kDuration);

  for (int i = 0; i < log->sample_times().size(); i++) {
    if (log->sample_times()(i) == 0.0) {
      // The initial outputs should be zero (due to the zero initial
      // conditions).
      EXPECT_TRUE(CompareMatrices(log->data().col(0), Vector2d(0., 0.)));
    } else {
      // Once time has advanced, outputs should have the intended derivatives.
      EXPECT_TRUE(CompareMatrices(log->data().col(i), Vector2d(2., 3.), 1e-14));
    }
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
