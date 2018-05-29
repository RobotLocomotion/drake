#include "drake/systems/primitives/random_source.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace systems {
namespace {

// Runs the system for 20 seconds, and checks the resulting histogram against
// the expected statistics.  The histogram is taken over the domain [min_value,
// max_value] with interval size h.  Statistics are compared against a rigorous
// bound multiplied by a fudge_factor (>=1, smaller is tighter).
template <typename Distribution, typename Generator>
void CheckStatistics(
    const std::function<double(double)>& cumulative_distribution,
    double min_value, double max_value, double h, double fudge_factor,
    std::unique_ptr<internal::RandomSource<Distribution, Generator>>
        random_source_system) {
  DiagramBuilder<double> builder;

  auto source = builder.AddSystem(std::move(random_source_system));
  source->set_name("source");
  auto logger = LogOutput(source->get_output_port(0), &builder);
  logger->set_name("logger");

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  BasicVector<double>& state =
      simulator.get_mutable_context().get_mutable_discrete_state(0);
  for (int i = 0; i < state.size(); i++) {
    state.SetAtIndex(i, 0.0);
  }

  simulator.Initialize();
  simulator.StepTo(20);

  const auto& x = logger->data();

  const int N = x.size();

  // Evaluate all subintervals [a,a+h] in [min_value,max_value].
  for (double a = min_value; a <= max_value - h; a += h) {
    // Counts the number of samples in (a,a+h).
    const double count = (x.array() >= a && x.array() <= a + h)
                             .template cast<double>()
                             .matrix()
                             .sum();

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
    // accomplishes this bound within a factor of fudge_factor.
    const double EY =
        cumulative_distribution(a + h) - cumulative_distribution(a);
    const double VarY = EY - EY * EY;
    EXPECT_NEAR(count / N, EY, fudge_factor * 1.645 * std::sqrt(VarY / N));
  }
}

GTEST_TEST(RandomSourceTest, UniformWhiteNoise) {
  auto random_source = std::make_unique<UniformRandomSource>(2, 0.0025);

  // Cumulative distribution function of the uniform distribution.
  auto Phi = [](double z) { return z; };

  const double min_value = 0.0;
  const double max_value = 1.0;
  const double h = 0.1;
  const double fudge_factor = 1.5;
  CheckStatistics(Phi, min_value, max_value, h, fudge_factor,
                  std::move(random_source));
}

GTEST_TEST(RandomSourceTest, GaussianWhiteNoise) {
  auto random_source = std::make_unique<GaussianRandomSource>(2, 0.0025);

  // Cumulative distribution function of the standard normal distribution.
  auto Phi = [](double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); };

  const double min_value = -2.0;
  const double max_value = 2.0;
  const double h = 0.1;
  const double fudge_factor = 2.0;
  CheckStatistics(Phi, min_value, max_value, h, fudge_factor,
                  std::move(random_source));
}

GTEST_TEST(RandomSourceTest, ExponentialWhiteNoise) {
  auto random_source = std::make_unique<ExponentialRandomSource>(2, 0.0025);

  // Cumulative distribution function of the exponential distribution with λ=1,
  // (note: only valid for z>=0).
  auto Phi = [](double z) { return 1 - std::exp(-z); };

  const double min_value = 0.0;
  const double max_value = 2.0;
  const double h = 0.1;
  const double fudge_factor = 2.0;
  CheckStatistics(Phi, min_value, max_value, h, fudge_factor,
                  std::move(random_source));
}

class TestSystem : public LeafSystem<double> {
 public:
  // Make methods available.
  using LeafSystem::DeclareInputPort;
  using LeafSystem::EvalVectorInput;
};

//      +-------------------------+
//      |                         |
//      | +--------+              |
//      | |uniform |              |
//      | +--------+       +----+ |
//      |          +------>|sys1| |
// +---------------------->|    | |
//      |                  +----+ |
//      | +--------+              |
//      | |gaussian|---+          |
//      | +--------+   |   +----+ |
//      | +--------+   +-->|    | |
//      | |exponent|------>|sys2| |
//      | +--------+   +-->|    | |
//      | +--------+   |   +----+ |
//      | |constant|---+          |
//      | +--------+              |
//      |                         |
//      +-------------------------+
GTEST_TEST(RandomSourceTest, AddToDiagramBuilderTest) {
  DiagramBuilder<double> builder;

  auto* sys1 = builder.AddSystem<TestSystem>();
  sys1->DeclareInputPort(kVectorValued, 3, RandomDistribution::kUniform);
  sys1->DeclareInputPort(kVectorValued, 2, RandomDistribution::kExponential);

  auto* sys2 = builder.AddSystem<TestSystem>();
  sys2->DeclareInputPort(kVectorValued, 5, RandomDistribution::kGaussian);
  sys2->DeclareInputPort(kVectorValued, 2, RandomDistribution::kExponential);
  sys2->DeclareInputPort(kVectorValued, 1, RandomDistribution::kGaussian);

  // Export input 1.
  builder.ExportInput(sys2->get_input_port(1));

  // Connect input 2 to a different block.
  const auto* constant_input =
      builder.AddSystem<ConstantVectorSource<double>>(14.0);
  builder.Connect(constant_input->get_output_port(), sys2->get_input_port(2));

  EXPECT_EQ(AddRandomInputs(1e-3, &builder), 3);

  const auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Check that the uniform input port is connected.
  EXPECT_NE(
      sys1->EvalVectorInput(diagram->GetSubsystemContext(*sys1, *context), 0),
      nullptr);

  // Check that the exponential input port is connected.
  EXPECT_NE(
      sys1->EvalVectorInput(diagram->GetSubsystemContext(*sys1, *context), 1),
      nullptr);

  // Check that the Gaussian input port is connected.
  EXPECT_NE(
      sys2->EvalVectorInput(diagram->GetSubsystemContext(*sys2, *context), 0),
      nullptr);

  // Check that the exported input remained exported.
  EXPECT_EQ(diagram->get_num_input_ports(), 1);
  EXPECT_EQ(diagram->get_input_port(0).size(), 2);

  // Check that the previously connected input remained connected.
  EXPECT_EQ(sys2->EvalEigenVectorInput(
                diagram->GetSubsystemContext(*sys2, *context), 2)[0],
            14.0);
}

GTEST_TEST(RandomSourceTest, CorrelationTest) {
  // Tests that two separate input ports, with the default seeds, are
  // uncorrelated.

  DiagramBuilder<double> builder;
  const int kSize = 1;
  const double kSampleTime = 0.0025;
  const auto* random1 =
      builder.AddSystem<GaussianRandomSource>(kSize, kSampleTime);
  const auto* log1 = LogOutput(random1->get_output_port(0), &builder);

  const auto* random2 =
      builder.AddSystem<GaussianRandomSource>(kSize, kSampleTime);
  const auto* log2 = LogOutput(random2->get_output_port(0), &builder);

  const auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.StepTo(20);

  const auto& x1 = log1->data();
  const auto& x2 = log2->data();

  EXPECT_EQ(x1.size(), x2.size());
  const int N = static_cast<int>(x1.size()) / 2;
  for (int i = 0; i < N; i++) {
    // Compute cross-correlation
    const double xcorr =
        (x1.middleCols(0, N).array() * x2.middleCols(i, N).array()).sum();
    // Note: The threshold doesn't need to be small.  Any correlations due to
    // using the same seed will lead to numbers ≊ 1.
    EXPECT_LE(xcorr / N, 0.1);
  }
}

// Make sure that calling SetRandomContext changes the output, and
// SetDefaultContext returns it to the original (default) output.
GTEST_TEST(RandomSourceTest, SetRandomContextTest) {
  UniformRandomSource random_source(2, 0.0025);
  auto output = random_source.get_output_port(0).Allocate();
  const BasicVector<double>& output_values =
      output->GetValueOrThrow<systems::BasicVector<double>>();

  auto context = random_source.CreateDefaultContext();

  random_source.get_output_port(0).Calc(*context, output.get());
  Eigen::Vector2d default_values = output_values.CopyToVector();

  RandomGenerator generator;
  random_source.SetRandomContext(context.get(), &generator);
  random_source.get_output_port(0).Calc(*context, output.get());
  EXPECT_NE(default_values[0], output_values.GetAtIndex(0));
  EXPECT_NE(default_values[1], output_values.GetAtIndex(1));

  random_source.SetDefaultContext(context.get());
  random_source.get_output_port(0).Calc(*context, output.get());
  EXPECT_EQ(default_values[0], output_values.GetAtIndex(0));
  EXPECT_EQ(default_values[1], output_values.GetAtIndex(1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
