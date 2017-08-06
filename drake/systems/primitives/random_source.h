#pragma once

#include <memory>
#include <random>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// State for a given random distribution and generator. This owns both the
/// distrubtion and the generator.
template <typename Distribution, typename Generator = std::mt19937>
class RandomState {
 public:
  typedef typename Generator::result_type Seed;
  static constexpr Seed default_seed = Generator::default_seed;

  explicit RandomState(Seed seed = Generator::default_seed)
      : generator_(seed) {}

  /// Generate the next random value with the given distribution.
  double GetNextValue() {
    return distribution_(generator_);
  }

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
/// Note: This system is only defined for the double scalar type.
///
/// Note: The hard-coding of (default) distribution parameters is imposed
/// intentionally to simplify analysis (by forcing systems taking noise inputs
/// to implement the shifting/scaling, the system itself contains all of the
/// necessary information for stochastic analysis).
///
/// @ingroup primitive_systems
template <typename Distribution, typename Generator = std::mt19937>
class RandomSource : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RandomSource)

  typedef systems::RandomState<Distribution, Generator> RandomState;
  typedef typename RandomState::Seed Seed;

  /// Constructs the RandomSource system.
  /// @param num_outputs The dimension of the (single) vector output port.
  /// @param sampling_interval_sec The sampling interval in seconds.
  RandomSource(int num_outputs, double sampling_interval_sec) {
    this->DeclarePeriodicUnrestrictedUpdate(sampling_interval_sec, 0.);
    this->DeclareVectorOutputPort(BasicVector<double>(num_outputs),
                                  &RandomSource::CopyStateToOutput);
    this->DeclareDiscreteState(num_outputs);
    this->DeclareAbstractState(AbstractValue::Make(RandomState(seed_)));
  }

  /// Initializes the random number generator.
  void set_random_seed(Seed seed) { seed_ = seed; }

 private:
  // Computes a random number and stores it in the discrete state.
  void DoCalcUnrestrictedUpdate(const Context<double>& context,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>* state) const override {
    auto& random_state =
        state->template get_mutable_abstract_state<RandomState>(0);
    auto* updates = state->get_mutable_discrete_state();
    const int N = updates->size();
    for (int i = 0; i < N; i++) {
      (*updates)[i] = random_state.GetNextValue();
    }
  }

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    return std::make_unique<AbstractValues>(
        AbstractValue::Make(RandomState(seed_)));
  }

  // Output is the zero-order hold of the discrete state.
  void CopyStateToOutput(const Context<double>& context,
                         BasicVector<double>* output) const {
    output->SetFromVector(context.get_discrete_state(0)->CopyToVector());
  }

  Seed seed_{RandomState::default_seed};
};

namespace internal {
/// Defines a version of the std::uniform_real_distribution that uses the
/// interval [-1,1] with the default parameters.  This is a more natural
/// distribution for random signals.
class mean_zero_uniform_real_distribution
    : public std::uniform_real_distribution<double> {
 public:
  mean_zero_uniform_real_distribution()
      : std::uniform_real_distribution<double>(-1.0, 1.0) {}
};

}  // namespace internal

/// Generates uniformly distributed random numbers in the interval [-1,1].
///
/// @ingroup primitive_systems
typedef RandomSource<internal::mean_zero_uniform_real_distribution>
    UniformRandomSource;

/// Generates uniformly distributed random numbers with mean zero and unit
/// covariance.
///
/// @ingroup primitive_systems
typedef RandomSource<std::normal_distribution<double>> GaussianRandomSource;

}  // namespace systems
}  // namespace drake
