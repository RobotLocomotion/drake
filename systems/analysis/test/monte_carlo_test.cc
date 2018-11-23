#include "drake/systems/analysis/monte_carlo.h"

#include <cmath>

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
    auto random_source =
        builder.AddSystem<UniformRandomSource>(kNumOutputs, sampling_interval);
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
  const int num_samples = 10;
  const auto results = MonteCarloSimulation(make_simulator, &GetScalarOutput,
                                            final_time, num_samples);

  EXPECT_EQ(results.size(), num_samples);

  // Check that the results were all different.
  std::unordered_set<double> outputs;
  for (const auto& result : results) {
    outputs.emplace(result.output);
  }
  EXPECT_EQ(outputs.size(), results.size());

  // Confirm that we can reproduce all of the results using RandomSimulation.
  // Walk through the results in reverse, just for good measure.
  for (auto it = results.rbegin(); it != results.rend(); ++it) {
    RandomGenerator generator(it->generator_snapshot);
    EXPECT_EQ(RandomSimulation(make_simulator, &GetScalarOutput, final_time,
                               &generator),
              it->output);
  }
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
