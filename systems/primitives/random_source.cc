#include "drake/systems/primitives/random_source.h"

#include <atomic>
#include <random>
#include <variant>

#include "drake/common/default_scalars.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace systems {
namespace {

using Seed = RandomSourced::Seed;

// Stores exactly one of the three supported distribution objects.  Note that
// the distribution objects hold computational state; they are not just pure
// mathematical functions.
using DistributionVariant = std::variant<
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
      case 0: return std::get<0>(distribution_)(generator_);
      case 1: return std::get<1>(distribution_)(generator_);
      case 2: return std::get<2>(distribution_)(generator_);
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

namespace internal {
template <typename T>
RandomSourceT<T>::RandomSourceT(RandomDistribution distribution,
                                int num_outputs, double sampling_interval_sec)
    : LeafSystem<T>(
          SystemScalarConverter::MakeWithoutSubtypeChecking<RandomSourceT>()),
      distribution_(distribution),
      sampling_interval_sec_{sampling_interval_sec},
      instance_seed_{get_next_seed()} {
  this->DeclareDiscreteState(num_outputs);
  this->DeclareAbstractState(Value<SampleGenerator>());
  this->DeclarePeriodicUnrestrictedUpdateEvent(
      sampling_interval_sec, 0., &RandomSourceT<T>::UpdateSamples);
  this->DeclareVectorOutputPort(
      "output", BasicVector<T>(num_outputs),
      [](const Context<T>& context, BasicVector<T>* output) {
        const auto& values = context.get_discrete_state(0);
        output->SetFrom(values);
      });
}

template <typename T>
RandomSourceT<T>::~RandomSourceT() {}

template <typename T>
template <typename U>
RandomSourceT<T>::RandomSourceT(const RandomSourceT<U>& other)
    : RandomSourceT<T>(other.get_distribution(),
                       other.get_output_port(0).size(),
                       other.sampling_interval_sec_) {}

template <typename T>
Seed RandomSourceT<T>::get_seed(const Context<double>& context) const {
  const auto& source = context.template get_abstract_state<SampleGenerator>(0);
  return source.seed();
}

template <typename T>
void RandomSourceT<T>::SetDefaultState(const Context<T>& context,
                                       State<T>* state) const {
  const Seed seed = fixed_seed_.value_or(instance_seed_);
  SetSeed(seed, context, state);
}

template <typename T>
void RandomSourceT<T>::SetRandomState(const Context<T>& context,
                                      State<T>* state,
                                      RandomGenerator* seed_generator) const {
  const Seed fresh_seed = (*seed_generator)();
  const Seed seed = fixed_seed_.value_or(fresh_seed);
  SetSeed(seed, context, state);
}

// Writes the given seed into abstract state (replacing the existing
// SampleGenerator) and then does `UpdateSamples`.
template <typename T>
void RandomSourceT<T>::SetSeed(Seed seed, const Context<T>& context,
                               State<T>* state) const {
  state->template get_mutable_abstract_state<SampleGenerator>(0) =
      SampleGenerator(seed, distribution_);
  UpdateSamples(context, state);
}

// Samples random values into the discrete state, using the SampleGenerator
// from the abstract state.  (Note that the generator's abstract state is also
// mutated as a side effect of this method.)
template <typename T>
void RandomSourceT<T>::UpdateSamples(const Context<T>&, State<T>* state) const {
  auto& source = state->template get_mutable_abstract_state<SampleGenerator>(0);
  auto& samples = state->get_mutable_discrete_state(0);
  for (int i = 0; i < samples.size(); ++i) {
    samples[i] = T(source.GenerateNext());
  }
}

}  // namespace internal

RandomSource::RandomSource(RandomDistribution distribution, int num_outputs,
                           double sampling_interval_sec)
    : internal::RandomSourceT<double>(distribution, num_outputs,
                                      sampling_interval_sec) {}

RandomSource::~RandomSource() {}

RandomDistribution RandomSource::get_distribution() const {
  return internal::RandomSourceT<double>::get_distribution();
}

Seed RandomSource::get_seed(const Context<double>& context) const {
  return internal::RandomSourceT<double>::get_seed(context);
}

std::optional<Seed> RandomSource::get_fixed_seed() const {
  return internal::RandomSourceT<double>::get_fixed_seed();
}

void RandomSource::set_fixed_seed(const std::optional<Seed>& seed) {
  return internal::RandomSourceT<double>::set_fixed_seed(seed);
}

template <typename T>
int AddRandomInputs(double sampling_interval_sec, DiagramBuilder<T>* builder) {
  int count = 0;
  // Note: the mutable assignment to const below looks odd, but
  // there is (currently) no builder->GetSystems() method.
  for (const auto* system : builder->GetMutableSystems()) {
    for (int i = 0; i < system->num_input_ports(); i++) {
      const systems::InputPort<T>& port = system->get_input_port(i);
      // Check for the random label.
      if (!port.is_random()) {
        continue;
      }

      if (builder->IsConnectedOrExported(port)) {
        continue;
      }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      using Source =
          typename std::conditional_t<std::is_same_v<T, double>, RandomSource,
                                      internal::RandomSourceT<T>>;
      const auto* const source = builder->template AddSystem<Source>(
          port.get_random_type().value(), port.size(), sampling_interval_sec);
#pragma GCC diagnostic pop

      builder->Connect(source->get_output_port(0), port);
      ++count;
    }
  }
  return count;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &AddRandomInputs<T>
))

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::internal::RandomSourceT)
