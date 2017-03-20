#include "drake/systems/primitives/random_source.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace {

// Simulate the random source and check its statistics.
GTEST_TEST(TestSignalLogger, GaussianWhiteNoise) {
  systems::DiagramBuilder<double> builder;

  auto source = builder.AddSystem<systems::GaussianRandomSource>(1, 0.0025);
  auto logger = builder.AddSystem<systems::SignalLogger<double>>(1);
  builder.Cascade(*source, *logger);

  source->set_random_seed(42);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.get_mutable_context()->get_mutable_discrete_state(0)->SetAtIndex(
      0, 0.0);

  simulator.Initialize();
  simulator.StepTo(5);

  const auto& x = logger->data();

  for (double threshold = -2.0; threshold < 2.0; threshold += 0.5) {
    double count = (x.array() < threshold).cast<double>().matrix().sum();
    // Probability of x < threshold for a Gaussian can be computed with erf:
    EXPECT_NEAR(count / x.size(),
                .5 + std::erf(threshold / std::sqrt(2.0)) / 2.0,
                0.02);  // Note intentionally very large tolerance.
    // TODO(russt): Tighten tolerance once #4325 is resolved.
  }
}

// Simulate the random source and check its statistics.
GTEST_TEST(TestSignalLogger, UniformWhiteNoise) {
  systems::DiagramBuilder<double> builder;

  auto source = builder.AddSystem<systems::UniformRandomSource>(1, 0.0025);
  auto logger = builder.AddSystem<systems::SignalLogger<double>>(1);
  builder.Cascade(*source, *logger);

  source->set_random_seed(42);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.get_mutable_context()->get_mutable_discrete_state(0)->SetAtIndex(
      0, 0.0);

  simulator.Initialize();
  simulator.StepTo(5);

  const auto& x = logger->data();

  for (double threshold = -1.0; threshold < 1.0; threshold += 0.1) {
    double count = (x.array() < threshold).cast<double>().matrix().sum();
    // Probability of x < threshold for this uniform distribution is
    // (threshold+1)/2.
    EXPECT_NEAR(count / x.size(), (threshold + 1.0) / 2.0,
                0.02);  // Note intentionally very large tolerance.
    // TODO(russt): Tighten tolerance once #4325 is resolved.
  }
}

}  // namespace
}  // namespace drake
