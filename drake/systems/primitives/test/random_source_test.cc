#include "drake/systems/primitives/random_source.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace {

// Simulate the random source and check its statistics.
GTEST_TEST(TestSignalLogger, GaussianWhiteNoise) {
  systems::DiagramBuilder<double> builder;

  auto source = builder.AddSystem<systems::GaussianRandomSource>(2, 0.0025);
  source->set_name("source");
  auto logger = builder.AddSystem<systems::SignalLogger<double>>(2);
  logger->set_name("logger");
  builder.Cascade(*source, *logger);

  source->set_random_seed(42);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.get_mutable_context()->get_mutable_discrete_state(0)->SetAtIndex(
      0, 0.0);

  simulator.Initialize();
  simulator.StepTo(20);

  const auto& x = logger->data();

  // Cumulative distribution function of the standard normal distribution.
  auto Phi = [](double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); };

  const double h = 0.1;
  const int N = x.size();
  // Evaluate all subintervals (a,a+h) in (-2,2).
  for (double a = -2.0; a <= 2.0 - h; a += h) {
    // Counts the number of samples in (a,a+h).
    const double count =
        (x.array() >= a && x.array() <= a + h).cast<double>().matrix().sum();

    // Basic confidence interval statistics.  See, for instance,
    //    Simulation and the Monte Carlo Method (Third Edition)
    //      by Rubinstein and Kroese, 2017
    //    page 108,
    // where I've used Y as the indicator function of a ≤ X ≤ a+h,
    // E[Y] = Phi(a+h)-Phi(a), Var(Y) = E[Y]-E[Y]², since E[Y²]=E[Y],
    // and Phi(x) is the cdf of the standard normal distribution.
    // We expect a perfect sampler to obtain a value
    //    count/N = E[Y] ± 1.645 √(Var(Y)/N)
    // with 95% confidence.  Checks that our pseudo-random sampler
    // accomplishes this bound within a factor of tol.
    const double EY = Phi(a + h) - Phi(a);
    const double VarY = EY - EY * EY;
    const double tol = 2.0;
    EXPECT_NEAR(count / N, EY, tol * 1.645 * std::sqrt(VarY / N));
  }
}

// Simulate the random source and check its statistics.
GTEST_TEST(TestSignalLogger, UniformWhiteNoise) {
  systems::DiagramBuilder<double> builder;

  auto source = builder.AddSystem<systems::UniformRandomSource>(2, 0.0025);
  source->set_name("source");
  auto logger = builder.AddSystem<systems::SignalLogger<double>>(2);
  logger->set_name("logger");
  builder.Cascade(*source, *logger);

  source->set_random_seed(42);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.get_mutable_context()->get_mutable_discrete_state(0)->SetAtIndex(
      0, 0.0);

  simulator.Initialize();
  simulator.StepTo(20);

  const auto& x = logger->data();

  const double h = 0.1;
  const int N = x.size();
  // Evaluate all subintervals (a,a+h) in (0,1).
  for (double a = 0.0; a <= 1.0 - h; a += h) {
    // Counts the number of samples in (a,a+h).
    const double count =
        (x.array() >= a && x.array() <= a + h).cast<double>().matrix().sum();

    // Basic confidence interval statistics.  See, for instance,
    //    Simulation and the Monte Carlo Method (Third Edition)
    //      by Rubinstein and Kroese, 2017
    //    page 108,
    // where I've used Y as the indicator function of a ≤ X ≤ a+h,
    // E[Y] = h, Var(Y) = h-h². We expect a perfect sampler to obtain
    // a value
    //   count/N = E[Y] ± 1.645 √(Var(Y)/N)
    // with 95% confidence.
    EXPECT_NEAR(count / N, h, 1.645 * std::sqrt((h - h * h) / N));
  }
}

}  // namespace
}  // namespace drake
