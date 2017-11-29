#pragma once

#include <memory>
#include <random>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

namespace internal {

template <typename Generator = std::mt19937>
typename Generator::result_type generate_unique_seed();

/// State for a given random distribution and generator. This owns both the
/// distribution and the generator.
template <typename Distribution, typename Generator = std::mt19937>
class RandomState {
 public:
  typedef typename Generator::result_type Seed;
  static constexpr Seed default_seed = Generator::default_seed;

  explicit RandomState(Seed seed) : generator_(seed) {}

  /// Generate the next random value with the given distribution.
  double GetNextValue() { return distribution_(generator_); }

 private:
  // TODO(russt): Obtain consistent results across multiple platforms (#4361).
  Generator generator_;
  Distribution distribution_;
};

/// A source block which generates random numbers at a fixed sampling interval,
/// with a zero-order hold between samples.  For continuous-time systems, this
/// can be interpreted as a band-limited approximation of continuous white noise
/// (with a power-spectral density of the form Ts * sinc^2( omega * Ts ), where
/// Ts is the sampling interval.
///
/// @tparam Distribution A class modeling the c++ RandomNumberDistribution
/// concept.
///   http://en.cppreference.com/w/cpp/concept/RandomNumberDistribution
///
/// @note User code should not instantiate this class directly, but
/// should use systems::UniformRandomSource, systems::GaussianRandomSource, and
/// systems::ExponentialRandomSource systems instead.
///
/// @note This system is only defined for the double scalar type.
///
/// @note The hard-coding of (default) distribution parameters is imposed
/// intentionally to simplify analysis (by forcing systems taking noise inputs
/// to implement the shifting/scaling, the system itself contains all of the
/// necessary information for stochastic analysis).
///
/// @see @ref stochastic_systems, UniformRandomSource, GaussianRandomSource,
/// ExponentialRandomSource.
///
/// @ingroup primitive_systems
template <typename Distribution, typename Generator = std::mt19937>
class RandomSource : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RandomSource)

  typedef internal::RandomState<Distribution, Generator> RandomState;
  typedef typename RandomState::Seed Seed;

  /// Constructs the RandomSource system.
  /// @param num_outputs The dimension of the (single) vector output port.
  /// @param sampling_interval_sec The sampling interval in seconds.
  RandomSource(int num_outputs, double sampling_interval_sec)
      : seed_(generate_unique_seed()) {
    this->DeclarePeriodicUnrestrictedUpdate(sampling_interval_sec, 0.);
    this->DeclareVectorOutputPort(BasicVector<double>(num_outputs),
                                  &RandomSource::CopyStateToOutput);
    this->DeclareDiscreteState(num_outputs);
    this->DeclareAbstractState(AbstractValue::Make(RandomState(seed_)));
  }

  /// Initializes the random number generator.  This must be set before
  /// the (abstract) state is allocated to take effect.
  void set_random_seed(Seed seed) { seed_ = seed; }

 private:
  // Computes a random number and stores it in the discrete state.
  void DoCalcUnrestrictedUpdate(
      const Context<double>&,
      const std::vector<const UnrestrictedUpdateEvent<double>*>&,
      State<double>* state) const override {
    auto& random_state =
        state->template get_mutable_abstract_state<RandomState>(0);
    auto& updates = state->get_mutable_discrete_state();
    const int N = updates.size();
    for (int i = 0; i < N; i++) {
      updates[i] = random_state.GetNextValue();
    }
  }

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    return std::make_unique<AbstractValues>(
        AbstractValue::Make(RandomState(seed_)));
  }

  // Output is the zero-order hold of the discrete state.
  void CopyStateToOutput(const Context<double>& context,
                         BasicVector<double>* output) const {
    output->SetFromVector(context.get_discrete_state(0).CopyToVector());
  }

  Seed seed_{RandomState::default_seed};
};

}  // namespace internal

/// Generates uniformly distributed random numbers in the interval [0,1].
///
/// @see internal::RandomSource
/// @ingroup primitive_systems
typedef internal::RandomSource<std::uniform_real_distribution<double>>
    UniformRandomSource;

/// Generates normally distributed random numbers with mean zero and unit
/// covariance.
///
/// @see internal::RandomSource
/// @ingroup primitive_systems
typedef internal::RandomSource<std::normal_distribution<double>>
    GaussianRandomSource;

/// Generates exponentially distributed random numbers with mean, standard
/// deviation, and scale parameter (aka 1/λ) set to one.
///
/// @see internal::RandomSource
/// @ingroup primitive_systems
typedef internal::RandomSource<std::exponential_distribution<double>>
    ExponentialRandomSource;

/// For each subsystem input port in @p builder that is (a) not yet connected
/// and (b) labeled as random in the InputPortDescriptor, this method will add a
/// new RandomSource system of the appropriate type and connect it to the
/// subsystem input port.
///
/// @param sampling_interval_sec interval to be used for all new sources.
/// @returns the total number of RandomSource systems added.
///
/// @see @ref stochastic_systems
int AddRandomInputs(double sampling_interval_sec,
                    DiagramBuilder<double>* builder);

}  // namespace systems
}  // namespace drake
