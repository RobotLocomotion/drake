#pragma once

#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "planning/parallelism.h"

namespace anzu {
namespace planning {

/// Helper to draw a uniformly-distributed double in [0.0, 1.0) from the
/// provided generator `generator`.
/// @pre generator != nullptr.
/// @return double in [0.0, 1.0).
double DrawUniformUnitReal(std::mt19937_64* generator);

/// Provides a per-thread source of random numbers, used in the PlanningSpace
/// API.
class PerThreadRandomSource {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PerThreadRandomSource);

  /// Constructor.
  /// @param seed Seed to use when constructing random number generators.
  /// @param parallelism How much query parallelism should be supported?
  PerThreadRandomSource(uint64_t seed, Parallelism parallelism);

  int num_generators() const { return static_cast<int>(generators_.size()); }

  /// Gets the generator associated with the specified thread.
  /// @param thread_number Thread number of the generator.
  /// @return Generator for the specified thread.
  std::mt19937_64& generator(int thread_number) {
    return generators_.at(thread_number);
  }

  /// Draw a uniformly distributed double in [0.0, 1.0).
  /// @param thread_number Thread number of the generator to use.
  /// @return Random double in [0.0, 1.0).
  double DrawUniformUnitReal(int thread_number);

  /// Draw a random value.
  /// @param thread_number Thread number of the generator to use.
  /// @return Random uint64_t.
  uint64_t DrawRaw(int thread_number);

  /// Reseed the per-thread generators using the provided initial seed.
  /// @param seed Initial seed to reseed per-thread generators.
  void ReseedGenerators(uint64_t seed);

  /// Reseed the generators, e.g. with the return value from a prior call to
  /// SnapshotGeneratorSeeds(), reproduce a known state of the generators.
  /// @param seeds Seeds to use in the per-thread generators. @pre number of
  /// seeds must match number of per-thread generators.
  void ReseedGeneratorsIndividually(const std::vector<uint64_t>& seeds);

  /// Capture seeds from the generators to produce a known state of the
  /// generators that can be reproduced later.
  /// @return Seeds for the per-thread generators.
  std::vector<uint64_t> SnapshotGeneratorSeeds();

 private:
  std::vector<std::mt19937_64> generators_;
};

}  // namespace planning
}  // namespace anzu
