#include "drake/systems/primitives/random_source.h"

#include <atomic>
#include <random>

#include "drake/common/never_destroyed.h"

namespace drake {
namespace systems {
namespace {

using Seed = RandomSource::Seed;

// Stores exactly one of the three supported distribution objects.  Note that
// the distribution objects hold computational state; they are not just pure
// mathematical functions.
using DistributionVariant = variant<
    std::uniform_real_distribution<double>,
    std::normal_distribution<double>,
    std::exponential_distribution<double>>;

// Creates a distribution object from the distribution enumeration.
DistributionVariant MakeDistributionVariant(RandomDistribution which) {
  switch (which) {
    case RandomDistribution::kUniform:
      return std::uniform_real_distribution<double>();
    case RandomDistribution::kGaussian:
      return std::normal_distribution<double>();
    case RandomDistribution::kExponential:
      return std::exponential_distribution<double>();
  }
  DRAKE_UNREACHABLE();
}

// Generates real-valued (i.e., `double`) samples from some distribution.  This
// serves as the abstract state of a RandomSource, which encompasses all of the
// source's state *except* for the currently-sampled output values which are
// stored as discrete state.
class SampleGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SampleGenerator)

  SampleGenerator() = default;
  SampleGenerator(Seed seed, RandomDistribution which)
      : seed_(seed), generator_(seed),
        distribution_(MakeDistributionVariant(which)) {}

  Seed seed() const { return seed_; }

  double GenerateNext() {
    switch (distribution_.index()) {
      case 0: return get<0>(distribution_)(generator_);
      case 1: return get<1>(distribution_)(generator_);
      case 2: return get<2>(distribution_)(generator_);
    }
    DRAKE_UNREACHABLE();
  }

 private:
  Seed seed_{RandomGenerator::default_seed};
  RandomGenerator generator_;
  DistributionVariant distribution_;
};

// Returns a monotonically increasing integer on each call.
Seed get_next_seed() {
  static never_destroyed<std::atomic<Seed>> seed(
      RandomGenerator::default_seed);
  return seed.access()++;
}

}  // namespace

RandomSource::RandomSource(
    RandomDistribution distribution, int num_outputs,
    double sampling_interval_sec)
    : distribution_(distribution), instance_seed_(get_next_seed()) {
  this->DeclareDiscreteState(num_outputs);
  this->DeclareAbstractState(Value<SampleGenerator>().Clone());
  this->DeclarePeriodicUnrestrictedUpdateEvent(
      sampling_interval_sec, 0., &RandomSource::UpdateSamples);
  this->DeclareVectorOutputPort(
      "output", BasicVector<double>(num_outputs),
      [](const Context<double>& context, BasicVector<double>* output) {
        const auto& values = context.get_discrete_state(0);
        output->SetFrom(values);
      });
}

RandomSource::~RandomSource() {}

Seed RandomSource::get_seed(const Context<double>& context) const {
  const auto& source = context.template get_abstract_state<SampleGenerator>(0);
  return source.seed();
}

void RandomSource::SetDefaultState(
    const Context<double>& context, State<double>* state) const {
  const Seed seed = fixed_seed_.value_or(instance_seed_);
  SetSeed(seed, context, state);
}

void RandomSource::SetRandomState(
    const Context<double>& context, State<double>* state,
    RandomGenerator* seed_generator) const {
  const Seed fresh_seed = (*seed_generator)();
  const Seed seed = fixed_seed_.value_or(fresh_seed);
  SetSeed(seed, context, state);
}

// Writes the given seed into abstract state (replacing the existing
// SampleGenerator) and then does `UpdateSamples`.
void RandomSource::SetSeed(
    Seed seed, const Context<double>& context, State<double>* state) const {
  state->template get_mutable_abstract_state<SampleGenerator>(0) =
      SampleGenerator(seed, distribution_);
  UpdateSamples(context, state);
}

// Samples random values into the discrete state, using the SampleGenerator
// from the abstract state.  (Note that the generator's abstract state is also
// mutated as a side effect of this method.)
void RandomSource::UpdateSamples(
    const Context<double>&, State<double>* state) const {
  auto& source = state->template get_mutable_abstract_state<SampleGenerator>(0);
  auto& samples = state->get_mutable_discrete_state(0);
  for (int i = 0; i < samples.size(); ++i) {
    samples[i] = source.GenerateNext();
  }
}

int AddRandomInputs(double sampling_interval_sec,
                    DiagramBuilder<double>* builder) {
  int count = 0;
  // Note: the mutable assignment to const below looks odd, but
  // there is (currently) no builder->GetSystems() method.
  for (const auto* system : builder->GetMutableSystems()) {
    for (int i = 0; i < system->num_input_ports(); i++) {
      const systems::InputPort<double>& port = system->get_input_port(i);
      // Check for the random label.
      if (!port.is_random()) {
        continue;
      }

      using InputPortLocator = Diagram<double>::InputPortLocator;
      // Check if the input is already wired up.
      InputPortLocator id{&port.get_system(), port.get_index()};
      if (builder->connection_map_.count(id) > 0 ||
          builder->diagram_input_set_.count(id) > 0) {
        continue;
      }

      const auto* const source = builder->AddSystem<RandomSource>(
          port.get_random_type().value(), port.size(), sampling_interval_sec);
      builder->Connect(source->get_output_port(0), port);
      ++count;
    }
  }
  return count;
}

}  // namespace systems
}  // namespace drake
