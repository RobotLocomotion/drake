#include "drake/systems/analysis/monte_carlo.h"

#include <cmath>
#include <thread>

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/random_source.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {
// Only use two threads during testing. This value should match the "cpu" tag
// in BUILD.bazel defining this test.
constexpr int kTestConcurrency = 2;

GTEST_TEST(SelectNumberOfThreadsToUseTest, BasicTest) {
  const int hardware_concurrency =
      static_cast<int>(std::thread::hardware_concurrency());

  // When kNoConcurrency is selected, only one thread should be used.
  EXPECT_EQ(internal::SelectNumberOfThreadsToUse(kNoConcurrency), 1);

  // If kUseHardwareConcurrency is specified, the number of threads should
  // match std::thread::hardware_concurrency().
  EXPECT_EQ(internal::SelectNumberOfThreadsToUse(kUseHardwareConcurrency),
            hardware_concurrency);

  // If a value greater than zero is specified, it selects the number of threads
  // to use.
  EXPECT_EQ(internal::SelectNumberOfThreadsToUse(1), 1);
  EXPECT_EQ(internal::SelectNumberOfThreadsToUse(10), 10);
  EXPECT_EQ(internal::SelectNumberOfThreadsToUse(100), 100);

  // Zero and negative values (that are not kUseHardwareConcurrency) throw.
  EXPECT_THROW(internal::SelectNumberOfThreadsToUse(0), std::exception);
  EXPECT_THROW(internal::SelectNumberOfThreadsToUse(-10), std::exception);
}

// Checks that RandomSimulation repeatedly produces the same output sample
// when given the same RandomGenerator, but produces different output samples
// when given different generators.
void CheckConsistentReplay(const SimulatorFactory& make_simulator,
                           const ScalarSystemFunction& output,
                           double final_time) {
  RandomGenerator generator;

  // Run a simulation with a snapshot of the generator saved.
  const RandomGenerator generator_snapshot(generator);
  const double random_output =
      RandomSimulation(make_simulator, output, final_time, &generator);

  // Run a few more random simulators to mix-up the state.  Make sure they
  // give different outputs.
  for (int i = 0; i < 5; i++) {
    EXPECT_NE(RandomSimulation(make_simulator, output, final_time, &generator),
              random_output);
  }

  // Reset the generator to our snapshot and confirm that we got our
  // original result:
  generator = generator_snapshot;
  EXPECT_EQ(RandomSimulation(make_simulator, output, final_time, &generator),
            random_output);
}

double GetScalarOutput(const System<double>& system,
                       const Context<double>& context) {
  return system.get_output_port(0)
      .Eval<BasicVector<double>>(context)
      .GetAtIndex(0);
}

// Check that we get the expected deterministic result if our
// SimulatorFactory is deterministic.
GTEST_TEST(RandomSimulationTest, DeterministicSimulator) {
  RandomGenerator generator;
  const double final_time = 0.1;
  const double value = 1.432;
  const SimulatorFactory make_simulator = [value](RandomGenerator*) {
    auto system = std::make_unique<ConstantVectorSource<double>>(value);
    return std::make_unique<Simulator<double>>(std::move(system));
  };
  EXPECT_EQ(RandomSimulation(make_simulator, &GetScalarOutput, final_time,
                             &generator),
            value);
}

// Ensure that RandomSimulation provides deterministic results when
// the "randomness" is in the SimulatorFactory.
GTEST_TEST(RandomSimulationTest, WithRandomSimulator) {
  // Factory for a simple system output value is different in each simulator
  // (but constant over the duration of each simulation).
  const SimulatorFactory make_simulator = [](RandomGenerator* generator) {
    std::normal_distribution<> distribution;
    auto system = std::make_unique<ConstantVectorSource<double>>(
        distribution(*generator));
    return std::make_unique<Simulator<double>>(std::move(system));
  };
  const double final_time = 0.1;
  CheckConsistentReplay(make_simulator, &GetScalarOutput, final_time);
}

// Simple system that outputs constant scalar, where this scalar is stored in
// the discrete state of the system.  The scalar value is randomized in
// SetRandomState().
class RandomContextSystem : public VectorSystem<double> {
 public:
  RandomContextSystem() : VectorSystem(0, 1) { this->DeclareDiscreteState(1); }

 private:
  void SetRandomState(const Context<double>& context, State<double>* state,
                      RandomGenerator* generator) const override {
    std::normal_distribution<> distribution;
    state->get_mutable_discrete_state(0).SetAtIndex(0,
                                                    distribution(*generator));
  }

  void DoCalcVectorOutput(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorX<double>>& input,
      const Eigen::VectorBlock<const VectorX<double>>& state,
      Eigen::VectorBlock<VectorX<double>>* output) const override {
    *output = state;
  }
};

// Ensure that RandomSimulation provides correct/deterministic results when
// the "randomness" is in the SetRandomContext.
GTEST_TEST(RandomSimulationTest, WithRandomContext) {
  // Factory for a simple system output value is different in each simulator
  // (but constant over the duration of each simulation).
  const SimulatorFactory make_simulator = [](RandomGenerator*) {
    auto system = std::make_unique<RandomContextSystem>();
    return std::make_unique<Simulator<double>>(std::move(system));
  };

  const double final_time = 0.1;
  CheckConsistentReplay(make_simulator, &GetScalarOutput, final_time);
}

// Ensure that RandomSimulation provides correct/deterministic results when
// the "randomness" comes from random input ports.  (Also provides coverage
// for using a DiagramBuilder in a SimulatorFactory).
GTEST_TEST(RandomSimulationTest, WithRandomInputs) {
  const SimulatorFactory make_simulator = [](RandomGenerator*) {
    DiagramBuilder<double> builder;
    const int kNumOutputs = 1;
    const double sampling_interval = 0.1;
    auto random_source = builder.AddSystem<RandomSource<double>>(
        RandomDistribution::kUniform, kNumOutputs, sampling_interval);
    auto pass_through = builder.template AddSystem<PassThrough>(kNumOutputs);
    builder.Connect(random_source->get_output_port(0),
                    pass_through->get_input_port());
    builder.ExportOutput(pass_through->get_output_port(), "random");
    auto diagram = builder.Build();
    return std::make_unique<Simulator<double>>(std::move(diagram));
  };

  const double final_time = 0.1;
  CheckConsistentReplay(make_simulator, &GetScalarOutput, final_time);
}

GTEST_TEST(MonteCarloSimulationTest, BasicTest) {
  const SimulatorFactory make_simulator = [](RandomGenerator* generator) {
    auto system = std::make_unique<RandomContextSystem>();
    return std::make_unique<Simulator<double>>(std::move(system));
  };
  const double final_time = 0.1;
  const int num_samples = 100;

  const RandomGenerator prototype_generator;
  RandomGenerator serial_generator(prototype_generator);
  RandomGenerator parallel_generator(prototype_generator);

  const auto serial_results = MonteCarloSimulation(
      make_simulator, &GetScalarOutput, final_time, num_samples,
      &serial_generator, kNoConcurrency);
  const auto parallel_results = MonteCarloSimulation(
      make_simulator, &GetScalarOutput, final_time, num_samples,
      &parallel_generator, kTestConcurrency);

  EXPECT_EQ(serial_results.size(), num_samples);
  EXPECT_EQ(parallel_results.size(), num_samples);

  // Check that the results were all different. We only check the serial results
  // since we check later that serial and parallel results are identical.
  std::unordered_set<double> serial_outputs;
  for (const auto& serial_result : serial_results) {
    serial_outputs.emplace(serial_result.output);
  }
  EXPECT_EQ(serial_outputs.size(), serial_results.size());

  // Confirm that serial and parallel MonteCarloSimulation produce the same
  // results, and that they are both reproducible.
  for (int sample = 0; sample < num_samples; ++sample) {
    const auto& serial_result = serial_results.at(sample);
    const auto& parallel_result = parallel_results.at(sample);

    EXPECT_EQ(serial_result.output, parallel_result.output);

    RandomGenerator serial_reproduction_generator(
        serial_result.generator_snapshot);
    RandomGenerator parallel_reproduction_generator(
        parallel_result.generator_snapshot);

    EXPECT_EQ(RandomSimulation(make_simulator, &GetScalarOutput, final_time,
                               &serial_reproduction_generator),
              serial_result.output);
    EXPECT_EQ(RandomSimulation(make_simulator, &GetScalarOutput, final_time,
                               &parallel_reproduction_generator),
              parallel_result.output);
  }
}

// Simple system that outputs constant scalar, where this scalar is stored in
// the discrete state of the system.  The scalar value is randomized in
// SetRandomState(). If the state value (cast to int) is odd, DoCalcVectorOutput
// throws.
class ThrowingRandomContextSystem : public VectorSystem<double> {
 public:
  ThrowingRandomContextSystem() : VectorSystem(0, 1) {
    this->DeclareDiscreteState(1);
  }

 private:
  void SetRandomState(const Context<double>& context, State<double>* state,
                      RandomGenerator* generator) const override {
    std::normal_distribution<> distribution;
    state->get_mutable_discrete_state(0).SetAtIndex(0,
                                                    distribution(*generator));
  }

  void DoCalcVectorOutput(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorX<double>>& input,
      const Eigen::VectorBlock<const VectorX<double>>& state,
      Eigen::VectorBlock<VectorX<double>>* output) const override {
    if ((static_cast<int>(state(0)) % 2) != 0) {
      throw std::runtime_error("State value is odd");
    } else {
      *output = state;
    }
  }
};

GTEST_TEST(MonteCarloSimulationExceptionTest, BasicTest) {
  const SimulatorFactory make_simulator = [](RandomGenerator* generator) {
    auto system = std::make_unique<ThrowingRandomContextSystem>();
    return std::make_unique<Simulator<double>>(std::move(system));
  };
  const double final_time = 0.1;
  const int num_samples = 10;

  const RandomGenerator prototype_generator;
  RandomGenerator serial_generator(prototype_generator);
  RandomGenerator parallel_generator(prototype_generator);

  EXPECT_THROW(MonteCarloSimulation(
      make_simulator, &GetScalarOutput, final_time, num_samples,
      &serial_generator, kNoConcurrency),
      std::exception);
  EXPECT_THROW(MonteCarloSimulation(
      make_simulator, &GetScalarOutput, final_time, num_samples,
      &parallel_generator, kTestConcurrency),
      std::exception);
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
