#pragma once

#include <common_robotics_utilities/openmp_helpers.hpp>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace anzu {
namespace planning {

// TODO(calderpg) Move this class to drake/common.
/// Provides a common interface for specifying the desired degree of parallelism
/// to use in a parallelized operation, regardless of how the parallelization is
/// implemented (e.g. OpenMP, std::async, TBB, etc). Represents a fixed number
/// of threads to use; either 1 (no parallelism), a user-specified number of
/// threads (any number >= 1), or the maximum number of threads to use. The
/// maximum number of threads to use defaults to the hardware concurrency as
/// reported via std::thread::hardware_concurrency(), and may be further limited
/// via the environment variable DRAKE_NUM_THREADS prior to the first
/// construction of a Parallelism(true). To be safe, any configuration of this
/// variable after main() should be performed prior to calling any Drake code.
class Parallelism {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parallelism)

  /// Gets a Parallelism with no parallelism (i.e. num_threads=1).
  static Parallelism None() { return Parallelism(false); }

  /// Gets a Parallelism with the maximum number of threads.
  static Parallelism Max() { return Parallelism(true); }

  /// Default constructor, no parallelism (i.e. num_threads=1) by default.
  Parallelism() : Parallelism(false) {}

  /// Constructs a Parallelism with either no parallelism (i.e. num_threads=1)
  /// or the maximum number of threads, as selected by `parallelize`.
  Parallelism(bool parallelize);  // NOLINT(runtime/explicit)

  /// Constructs with the provided number of threads `num_threads`.
  /// @pre num_threads >= 1.
  /// @note Constructing and using a Parallelism with num_threads greater than
  /// the actual hardware concurrency may result in fewer than the specified
  /// number of threads actually being launched or poor performance due to CPU
  /// contention.
  explicit Parallelism(int num_threads) : num_threads_(num_threads) {
    DRAKE_THROW_UNLESS(num_threads >= 1);
  }

  int num_threads() const { return num_threads_; }

 private:
  int num_threads_ = 1;
};

/// Convenience helper to convert the provided `parallelism` to the equivalent
/// CRU DegreeOfParallelism value.
inline common_robotics_utilities::openmp_helpers::DegreeOfParallelism ToCRU(
    Parallelism parallelism) {
  return common_robotics_utilities::openmp_helpers::DegreeOfParallelism(
      parallelism.num_threads());
}

}  // namespace planning
}  // namespace anzu
