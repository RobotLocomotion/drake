#pragma once

#include <optional>
#include <string_view>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {

/** Specifies a desired degree of parallelism for a parallelized operation.

This class denotes a specific number of threads; either 1 (no parallelism), a
user-specified value (any number >= 1), or the maximum number. For convenience,
conversion from `bool` is provided so that `false` is no parallelism and `true`
is maximum parallelism.

Drake's API uses this class to allow users to control the degree of parallelism,
regardless of how the parallelization is implemented (e.g., `std::async`,
OpenMP, TBB, etc).

@section DRAKE_NUM_THREADS Configuring the process-wide maximum parallelism

The number of threads denoted by Parallelism::Max() is configurable with
environment variables, but will be invariant within a single process. The first
time it's accessed, the configured value will be latched into a global variable
indefinitely. To ensure your configuration is obeyed, any changes to the
environment variables that govern the max parallelism should be made prior to
importing or calling any Drake code.

The following recipe determines the value that Parallelism::Max().num_threads()
will report:

1. The default is the hardware limit from `std::thread::hardware_concurrency()`;
this will be used when none of the special cases below are in effect.

2. If the environment variable `DRAKE_NUM_THREADS` is set to a positive integer
less than `hardware_concurrency()`, the `num_threads` will be that value.

3. If the environment variable `DRAKE_NUM_THREADS` is not set but the
environment variable `OMP_NUM_THREADS` is set to a positive integer less than
`hardware_concurrency()`, the `num_threads` will be that value. (Note in
particular that a comma-separated `OMP_NUM_THREADS` value will be ignored, even
though that syntax might be valid for an OpenMP library).

The configuration recipe above does not require Drake to be built with OpenMP
enabled. The inspection of `OMP_NUM_THREADS` as a configuration value is
provided for convenience, regardless of whether OpenMP is enabled.

<b>A note for Drake developers</b>

In Drake's unit tests, `DRAKE_NUM_THREADS` is set to "1" by default. If your
unit test requires actual parallelism, use the `num_threads = N` attribute in
the `BUILD.bazel` file to declare a different value. */
class Parallelism {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parallelism);

  /** Constructs a %Parallelism with no parallelism (i.e., num_threads=1).
  Python note: This function is not bound in pydrake (due to its name); instead,
  use the default constructor (or pass either `1` or `False` to the 1-argument
  constructor). */
  static Parallelism None() { return Parallelism(); }

  /** Constructs a %Parallelism with the maximum number of threads. Refer to the
  class overview documentation for how to configure the maximum. */
  static Parallelism Max();

  /** Default constructs with no parallelism (i.e., num_threads=1). */
  Parallelism() = default;

  /** Constructs a %Parallelism with either no parallelism (i.e., num_threads=1)
  or the maximum number of threads (Max()), as selected by `parallelize`. This
  constructor allows for implicit conversion, for convenience. */
  Parallelism(bool parallelize) {  // NOLINT(runtime/explicit)
    if (parallelize) {
      *this = Max();
    }
  }

  /** Constructs with the provided number of threads `num_threads`.
  @pre num_threads >= 1.
  @note Constructing and using a %Parallelism with num_threads greater than the
  actual hardware concurrency may result in fewer than the specified number of
  threads actually being launched or poor performance due to CPU contention. */
  explicit Parallelism(int num_threads) : num_threads_(num_threads) {
    DRAKE_THROW_UNLESS(num_threads >= 1);
  }

  /** Returns the degree of parallelism. The result will always be >= 1. */
  int num_threads() const { return num_threads_; }

 private:
  int num_threads_ = 1;
};

namespace internal {
// Exposed for unit testing.
int ConfigureMaxNumThreads(const char* drake_num_threads,
                           const char* omp_num_threads);
}  // namespace internal

}  // namespace drake
