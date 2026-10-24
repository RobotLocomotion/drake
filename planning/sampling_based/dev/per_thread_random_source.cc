#include "drake/planning/sampling_based/dev/per_thread_random_source.h"

#include <algorithm>
#include <limits>
#include <random>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace planning {

double DrawUniformUnitReal(std::mt19937_64* generator) {
  DRAKE_THROW_UNLESS(generator != nullptr);
  return std::generate_canonical<double, std::numeric_limits<double>::digits>(
      *generator);
}

PerThreadRandomSource::PerThreadRandomSource(const uint64_t seed,
                                             const Parallelism parallelism) {
  // Make a generator and distribution for each thread.
  drake::log()->info(
      "PerThreadRandomSource allocating random generators to support "
      "parallelism {}",
      parallelism.num_threads());
  std::mt19937_64 seed_dist(seed);
  for (int thread = 0; thread < parallelism.num_threads(); ++thread) {
    generators_.emplace_back(std::mt19937_64(seed_dist()));
  }
}

double PerThreadRandomSource::DrawUniformUnitReal(const int thread_number) {
  return drake::planning::DrawUniformUnitReal(&generator(thread_number));
}

uint64_t PerThreadRandomSource::DrawRaw(const int thread_number) {
  return generator(thread_number)();
}

void PerThreadRandomSource::ReseedGenerators(const uint64_t seed) {
  std::mt19937_64 seed_dist(seed);
  for (size_t index = 0; index < generators_.size(); ++index) {
    const uint64_t new_seed = seed_dist();
    generators_.at(index).seed(new_seed);
  }
}

void PerThreadRandomSource::ReseedGeneratorsIndividually(
    const std::vector<uint64_t>& seeds) {
  DRAKE_THROW_UNLESS(seeds.size() == generators_.size());
  for (size_t index = 0; index < generators_.size(); ++index) {
    generators_.at(index).seed(seeds.at(index));
  }
}

std::vector<uint64_t> PerThreadRandomSource::SnapshotGeneratorSeeds() {
  std::vector<uint64_t> snapshot_seeds(generators_.size(), 0);
  for (size_t index = 0; index < generators_.size(); ++index) {
    auto& generator = generators_.at(index);
    const uint64_t new_seed = generator();
    snapshot_seeds.at(index) = new_seed;
    generator.seed(new_seed);
  }
  return snapshot_seeds;
}

}  // namespace planning
}  // namespace drake
