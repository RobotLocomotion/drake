#include "drake/systems/primitives/random_source.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/vector_log_sink.h"

namespace drake {
namespace systems {
namespace {

// Runs the system for 20 seconds, and checks the resulting histogram against
// the expected statistics.  The histogram is taken over the domain [min_value,
// max_value] with interval size h.  Statistics are compared against a rigorous
// bound multiplied by a fudge_factor (>=1, smaller is tighter).
void CheckStatistics(
    const std::function<double(double)>& cumulative_distribution,
    double min_value, double max_value, double h, double fudge_factor,
    std::unique_ptr<RandomSource<double>> random_source_system) {
  DiagramBuilder<double> builder;

  auto source = builder.AddSystem(std::move(random_source_system));
  source->set_name("source");
  auto logger = LogVectorOutput(source->get_output_port(0), &builder);
  logger->set_name("logger");

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  BasicVector<double>& state =
      simulator.get_mutable_context().get_mutable_discrete_state(0);
  for (int i = 0; i < state.size(); i++) {
    state.SetAtIndex(i, 0.0);
  }

  simulator.Initialize();
  simulator.AdvanceTo(20);

  const auto& x = logger->FindLog(simulator.get_context()).data();

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
  auto random_source = std::make_unique<RandomSource<double>>(
      RandomDistribution::kUniform, 2, 0.0025);
  EXPECT_EQ(random_source->get_distribution(), RandomDistribution::kUniform);
  EXPECT_EQ(random_source->get_fixed_seed(), std::nullopt);

  // Cumulative distribution function of the uniform distribution.
  auto Phi = [](double z) { return z; };

  const double min_value = 0.0;
  const double max_value = 1.0;
  const double h = 0.1;
  const double fudge_factor = 1.5;
  CheckStatistics(Phi, min_value, max_value, h, fudge_factor,
                  std::move(random_source));
}

GTEST_TEST(RandomSourceTest, ToAutoDiff) {
  auto random_source = std::make_unique<RandomSource<double>>(
      RandomDistribution::kUniform, 2, 0.0025);
  EXPECT_TRUE(is_autodiffxd_convertible(*random_source));
}

GTEST_TEST(RandomSourceTest, ToSymbolic) {
  auto random_source = std::make_unique<RandomSource<double>>(
      RandomDistribution::kUniform, 2, 0.0025);
  EXPECT_FALSE(is_symbolic_convertible(*random_source));
}

GTEST_TEST(RandomSourceTest, UniformWhiteNoiseAutoDiff) {
  auto random_source = std::make_unique<RandomSource<AutoDiffXd>>(
      RandomDistribution::kUniform, 2, 0.0025);
  EXPECT_EQ(random_source->get_distribution(), RandomDistribution::kUniform);
  EXPECT_EQ(random_source->get_fixed_seed(), std::nullopt);

  // Now take a single sample from this source. Make sure the scalar type of the
  // sample is AutoDiffXd, and the gradient is empty.
  auto context = random_source->CreateDefaultContext();
  auto sample = random_source->get_output_port().Eval(*context);
  static_assert(std::is_same_v<decltype(sample)::Scalar, AutoDiffXd>,
                "The sample scalar type should be AutoDiffXd");
  EXPECT_EQ(sample.rows(), 2);
  // The derivatives are empty.
  EXPECT_EQ(sample(0).derivatives().rows(), 0);
  EXPECT_EQ(sample(1).derivatives().rows(), 0);
}

GTEST_TEST(RandomSourceTest, GaussianWhiteNoise) {
  auto random_source = std::make_unique<RandomSource<double>>(
      RandomDistribution::kGaussian, 2, 0.0025);
  EXPECT_EQ(random_source->get_distribution(), RandomDistribution::kGaussian);
  EXPECT_EQ(random_source->get_fixed_seed(), std::nullopt);

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
  auto random_source = std::make_unique<RandomSource<double>>(
      RandomDistribution::kExponential, 2, 0.0025);
  EXPECT_EQ(random_source->get_distribution(),
            RandomDistribution::kExponential);
  EXPECT_EQ(random_source->get_fixed_seed(), std::nullopt);

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

template <typename T>
class TestSystem : public LeafSystem<T> {
 public:
  // Make methods available.
  using LeafSystem<T>::DeclareInputPort;
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

template <typename T>
void AddToDiagramBuilderTest() {
  DiagramBuilder<T> builder;

  auto* sys1 = builder.template AddSystem<TestSystem<T>>();
  sys1->DeclareInputPort("uniform", kVectorValued, 3,
                         RandomDistribution::kUniform);
  sys1->DeclareInputPort("exponential", kVectorValued, 2,
                         RandomDistribution::kExponential);

  auto* sys2 = builder.template AddSystem<TestSystem<T>>();
  sys2->DeclareInputPort("gaussian", kVectorValued, 5,
                         RandomDistribution::kGaussian);
  sys2->DeclareInputPort("exponential", kVectorValued, 2,
                         RandomDistribution::kExponential);
  sys2->DeclareInputPort("scalar_gaussian", kVectorValued, 1,
                         RandomDistribution::kGaussian);

  // Export input 1.
  builder.ExportInput(sys2->get_input_port(1));

  // Connect input 2 to a different block.
  const auto* constant_input =
      builder.template AddSystem<ConstantVectorSource<T>>(T(14.0));
  builder.Connect(constant_input->get_output_port(), sys2->get_input_port(2));

  EXPECT_EQ(AddRandomInputs(1e-3, &builder), 3);

  const auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Check that the uniform input port is connected.
  EXPECT_TRUE(sys1->get_input_port(0).HasValue(
      diagram->GetSubsystemContext(*sys1, *context)));

  // Check that the exponential input port is connected.
  EXPECT_TRUE(sys1->get_input_port(1).HasValue(
      diagram->GetSubsystemContext(*sys1, *context)));

  // Check that the Gaussian input port is connected.
  EXPECT_TRUE(sys2->get_input_port(0).HasValue(
      diagram->GetSubsystemContext(*sys2, *context)));

  // Check that the exported input remained exported.
  EXPECT_EQ(diagram->num_input_ports(), 1);
  EXPECT_EQ(diagram->get_input_port(0).size(), 2);

  // Check that the previously connected input remained connected.
  EXPECT_EQ(sys2->get_input_port(2).Eval(
                diagram->GetSubsystemContext(*sys2, *context))[0],
            T(14.0));
}

GTEST_TEST(RandomSourceTest, AddToDiagramBuilderTest) {
  AddToDiagramBuilderTest<double>();
  AddToDiagramBuilderTest<AutoDiffXd>();
}

GTEST_TEST(RandomSourceTest, CorrelationTest) {
  // Tests that two separate input ports, with the default seeds, are
  // uncorrelated.

  DiagramBuilder<double> builder;
  const int kSize = 1;
  const double kSampleTime = 0.0025;
  const auto* random1 = builder.AddSystem<RandomSource<double>>(
      RandomDistribution::kGaussian, kSize, kSampleTime);
  const auto* log1 = LogVectorOutput(random1->get_output_port(0), &builder);

  const auto* random2 = builder.AddSystem<RandomSource<double>>(
      RandomDistribution::kGaussian, kSize, kSampleTime);
  const auto* log2 = LogVectorOutput(random2->get_output_port(0), &builder);

  const auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  const auto& context = simulator.get_context();
  simulator.Initialize();
  simulator.AdvanceTo(20);

  const auto& x1 = log1->FindLog(context).data();
  const auto& x2 = log2->FindLog(context).data();

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

// Check the invariants of the seed lifecycle.
GTEST_TEST(RandomSourceTest, SeedTest) {
  RandomSource<double> dut(RandomDistribution::kUniform, 1, 0.0025);
  const auto& port = dut.get_output_port(0);

  // The source does not have a fixed seed by default.
  EXPECT_EQ(dut.get_fixed_seed(), std::nullopt);

  // A given instance of a RandomSource always defaults to the same seed.
  using Seed = RandomSource<double>::Seed;
  const Seed default_seed = dut.get_seed(*dut.CreateDefaultContext());
  EXPECT_EQ(dut.get_seed(*dut.CreateDefaultContext()), default_seed);
  EXPECT_EQ(dut.get_seed(*dut.CreateDefaultContext()), default_seed);
  EXPECT_EQ(dut.get_seed(*dut.CreateDefaultContext()), default_seed);

  // A given instance of a RandomSource always defaults to the same output.
  const double default_output = port.Eval(*dut.CreateDefaultContext())[0];
  EXPECT_EQ(port.Eval(*dut.CreateDefaultContext())[0], default_output);
  EXPECT_EQ(port.Eval(*dut.CreateDefaultContext())[0], default_output);
  EXPECT_EQ(port.Eval(*dut.CreateDefaultContext())[0], default_output);

  // When asked to create a random seed, a fresh seed and output appears.  (The
  // checks below have a small chance of ending up spuriously false.)
  struct SeedAndOutput { Seed seed{}; double output{}; };
  RandomGenerator new_seed_generator;
  auto make_new_seed_and_output = [&dut, &new_seed_generator]() {
    auto random_context = dut.CreateDefaultContext();
    dut.SetRandomContext(random_context.get(), &new_seed_generator);
    return SeedAndOutput{
        dut.get_seed(*random_context),
        dut.get_output_port(0).Eval(*random_context)[0]};
  };
  const auto new1 = make_new_seed_and_output();
  const auto new2 = make_new_seed_and_output();
  EXPECT_NE(new1.seed, default_seed);
  EXPECT_NE(new2.seed, default_seed);
  EXPECT_NE(new1.seed, new2.seed);
  EXPECT_NE(new1.output, default_output);
  EXPECT_NE(new2.output, default_output);
  EXPECT_NE(new1.output, new2.output);

  // We can re-create those same identical random seeds and values again,
  // starting from the same (original) generator.
  new_seed_generator = {};
  const auto new1b = make_new_seed_and_output();
  const auto new2b = make_new_seed_and_output();
  EXPECT_EQ(new1b.seed, new1.seed);
  EXPECT_EQ(new2b.seed, new2.seed);
  EXPECT_EQ(new1b.output, new1.output);
  EXPECT_EQ(new2b.output, new2.output);

  // Now let's spin up a different RandomSource.
  RandomSource<double> other(dut.get_distribution(), 1, 0.0025);
  const auto& other_port = other.get_output_port(0);

  // The output defaults to something fresh.
  const double other_default_output =
      other_port.Eval(*other.CreateDefaultContext())[0];
  EXPECT_NE(other_default_output, default_output);

  // Even on a different RandomSource, we can re-create the identical values of
  // the original dut if we start from identical constructor arguments the same
  // fixed seed.
  other.set_fixed_seed(new1.seed);
  EXPECT_EQ(other.get_fixed_seed().value_or(0), new1.seed);
  auto other_context = other.CreateDefaultContext();
  EXPECT_EQ(other.get_seed(*other_context), new1.seed);
  EXPECT_EQ(other_port.Eval(*other_context)[0], new1.output);

  // Both the default context and the random context use the fixed seed.
  RandomGenerator ignored_seed_generator;
  other.SetRandomContext(other_context.get(), &ignored_seed_generator);
  EXPECT_EQ(other.get_seed(*other_context), new1.seed);
  EXPECT_EQ(other_port.Eval(*other_context)[0], new1.output);
  other.SetRandomContext(other_context.get(), &ignored_seed_generator);
  EXPECT_EQ(other.get_seed(*other_context), new1.seed);
  EXPECT_EQ(other_port.Eval(*other_context)[0], new1.output);

  // If we remove the fixed seed, it returns to normal.
  other.set_fixed_seed(std::nullopt);
  EXPECT_EQ(other.get_fixed_seed(), std::nullopt);
  EXPECT_EQ(other_port.Eval(*other.CreateDefaultContext())[0],
            other_default_output);
}

// Make sure that calling SetRandomContext changes the output, and
// SetDefaultContext returns it to the original (default) output.
GTEST_TEST(RandomSourceTest, SetRandomContextTest) {
  RandomSource<double> random_source(RandomDistribution::kUniform, 2, 0.0025);

  auto context = random_source.CreateDefaultContext();
  const Eigen::Vector2d default_values =
      random_source.get_output_port(0).Eval(*context);

  RandomGenerator generator;
  random_source.SetRandomContext(context.get(), &generator);
  const Eigen::Vector2d novel_values =
      random_source.get_output_port(0).Eval(*context);
  EXPECT_NE(default_values[0], novel_values[0]);
  EXPECT_NE(default_values[1], novel_values[1]);

  random_source.SetDefaultContext(context.get());
  const Eigen::Vector2d default_values_again =
      random_source.get_output_port(0).Eval(*context);
  EXPECT_EQ(default_values[0], default_values_again[0]);
  EXPECT_EQ(default_values[1], default_values_again[1]);
}

}  // namespace
}  // namespace systems
}  // namespace drake
