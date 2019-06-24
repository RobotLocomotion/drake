#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/random.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A source block which generates random numbers at a fixed sampling interval,
/// with a zero-order hold between samples.  For continuous-time systems, this
/// can be interpreted as a band-limited approximation of continuous white noise
/// (with a power-spectral density of the form Ts * sinc^2( omega * Ts ), where
/// Ts is the sampling interval.
///
/// @system{RandomSource,,@output_port{output}}
///
/// @note This system is only defined for the double scalar type.
///
/// @note The exact distribution results may vary across multiple platforms or
/// revisions of Drake, but will be consistent for all compilations on a given
/// platform and Drake revision.
///
/// @note The hard-coding of (default) distribution parameters is imposed
/// intentionally to simplify analysis (by forcing systems taking noise inputs
/// to implement the shifting/scaling, the system itself contains all of the
/// necessary information for stochastic analysis).
///
/// @see @ref stochastic_systems
///
/// @ingroup primitive_systems
class RandomSource : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RandomSource)

  /// An integer type for a random seed.
  using Seed = RandomGenerator::result_type;

  /// Constructs the RandomSource system.
  /// @param distribution The RandomDistribution used for each of the outputs.
  /// @param num_outputs The dimension of the (single) vector output port.
  /// @param sampling_interval_sec The sampling interval in seconds.
  RandomSource(RandomDistribution distribution, int num_outputs,
               double sampling_interval_sec);

  ~RandomSource() override;

 private:
  void SetDefaultState(const Context<double>&, State<double>*) const final;
  void SetRandomState(const Context<double>&, State<double>*,
                      RandomGenerator*) const final;
  void SetSeed(Seed, const Context<double>&, State<double>*) const;
  void UpdateSamples(const Context<double>&, State<double>*) const;

  const RandomDistribution distribution_;
  const Seed seed_;
};

/// For each subsystem input port in @p builder that is (a) not yet connected
/// and (b) labeled as random in the InputPort, this method will add a
/// new RandomSource system of the appropriate type and connect it to the
/// subsystem input port.
///
/// @param sampling_interval_sec interval to be used for all new sources.
/// @returns the total number of RandomSource systems added.
///
/// @see @ref stochastic_systems
int AddRandomInputs(double sampling_interval_sec,
                    DiagramBuilder<double>* builder);

namespace internal {
// TODO(jwnimmer-tri) Once this class disappears, update RandomSource to be
// declared final.
/// (Deprecated.) A RandomSource with a compile-time RandomDistribution.
template <RandomDistribution distribution>
class RandomSourceWithDistribution final : public RandomSource {
 public:
  RandomSourceWithDistribution(int num_outputs, double sampling_interval_sec)
      : RandomSource(distribution, num_outputs, sampling_interval_sec) {}
};
}  // namespace internal

/// (Deprecated.) Generates uniformly distributed random numbers in the
/// interval [0.0, 1.0).
/// @see RandomSource
using UniformRandomSource
    DRAKE_DEPRECATED("2019-10-01",
        "Use primitives::RandomSource(kUniform, ...) instead of "
        "primitives::UniformRandomSource.")
    = internal::RandomSourceWithDistribution<RandomDistribution::kUniform>;

/// (Deprecated.) Generates normally distributed random numbers with mean zero
/// and unit covariance.
/// @see RandomSource
using GaussianRandomSource
    DRAKE_DEPRECATED("2019-10-01",
        "Use primitives::RandomSource(kGaussian, ...) instead of "
        "primitives::GaussianRandomSource.")
    = internal::RandomSourceWithDistribution<RandomDistribution::kGaussian>;

/// (Deprecated.) Generates exponentially distributed random numbers with mean,
/// standard deviation, and scale parameter (aka 1/λ) set to one.
/// @see RandomSource
using ExponentialRandomSource
    DRAKE_DEPRECATED("2019-10-01",
        "Use primitives::RandomSource(kExponential, ...) instead of "
        "primitives::ExponentialRandomSource.")
    = internal::RandomSourceWithDistribution<RandomDistribution::kExponential>;

}  // namespace systems
}  // namespace drake
