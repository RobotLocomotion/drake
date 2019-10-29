#pragma once

#include <optional>
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
/// This system exposes a parameter named `seed` for the pseudo-random number
/// generator that determines the noise output.  The `seed` parameter behaves
/// as follows:
///
/// 1. Each newly-created RandomSource chooses a new `per_instance_seed` member
/// field value for itself.  The value will be unique within the current
/// executable process.
///
/// 2. By default, `source.CreateDefaultContext()` will set the returned
/// context's `seed` parameter to `per_instance_seed`.  Therefore, for a given
/// instance of this system, the parameters, state, and outputs will be
/// identical for all simulations that start from a default context.
///
/// 3. By default, `source.SetRandomContext()` will choose a new, arbitrary
/// value for the context's `seed` parameter, which means that the system's
/// parameters, state, and outputs will (almost certainly) differ from their
/// defaults.
///
/// 4. The user may call `source.set_fixed_seed(new_seed)` on this system.
/// When a `new_seed` value is provided, it is used by both
/// `CreateDefaultContext` and `SetRandomContext`.  Therefore, the system's
/// parameters, state, and outputs will be identical to any other instances
/// that share the same `new_seed` value for their `seed` context parameter.
/// Note that `set_fixed_seed` affects subsequently-created contexts; any
/// pre-existing contexts are unaffected.  The user may call
/// `source.set_fixed_seed(nullopt)` to revert to the default the behaviors
/// described in #2 and #3 again.
///
/// 5. The context returned by `source.AllocateContext()` does not contain a
/// valid `seed` parameter; that context should not be used until its values
/// are populated via another Context.
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
class RandomSource final : public LeafSystem<double> {
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

  /// Returns the `distribution` given at construction.
  RandomDistribution get_distribution() const { return distribution_; }

  /// Returns the value of the `seed` parameter in the given context.
  Seed get_seed(const Context<double>& context) const;

  /// Gets this system's fixed random seed (or else nullopt when the seed is
  /// not fixed).  Refer to the class overview documentation for details.
  std::optional<Seed> get_fixed_seed() const { return fixed_seed_; }

  /// Sets (or clears) this system's fixed random seed.  Refer to the class
  /// overview documentation for details.
  void set_fixed_seed(const std::optional<Seed>& seed) { fixed_seed_ = seed; }

 private:
  void SetDefaultState(const Context<double>&, State<double>*) const final;
  void SetRandomState(const Context<double>&, State<double>*,
                      RandomGenerator*) const final;
  void SetSeed(Seed, const Context<double>&, State<double>*) const;
  void UpdateSamples(const Context<double>&, State<double>*) const;

  const RandomDistribution distribution_;
  const Seed instance_seed_;
  std::optional<Seed> fixed_seed_;
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

}  // namespace systems
}  // namespace drake
