#include "drake/common/parallelism.h"

#include <charconv>
#include <cstdlib>
#include <thread>

#include "drake/common/text_logging.h"

namespace drake {
namespace internal {
namespace {

constexpr char kEnvDrakeNumThreads[] = "DRAKE_NUM_THREADS";
constexpr char kEnvOmpNumThreads[] = "OMP_NUM_THREADS";

/* If `input` is a positive integer, returns it; otherwise, returns null.
Integers must be in base 10 (no hex, no octal). */
std::optional<int> ParsePositiveInt(std::string_view input) {
  int parsed{};
  const char* const first = input.data();
  const char* const last = first + input.size();
  const auto [ptr, error] = std::from_chars(first, last, parsed);
  if (ptr == last && error == std::errc() && parsed >= 1) {
    return parsed;
  }
  return std::nullopt;
}

}  // namespace

// Parse the maximum number of threads as specified by the given environment
// variable values. In normal use, this function is only ever called once per
// process, with `drake_num_threads` set to `getenv("DRAKE_NUM_THREADS")` and
// `omp_num_threads` set to `getenv("OMP_NUM_THREADS")`. However, during unit
// testing, values NOT from the environment will be used instead.
int ConfigureMaxNumThreads(const char* const drake_num_threads,
                           const char* const omp_num_threads) {
  const int hardware_concurrency = std::thread::hardware_concurrency();
  if (drake_num_threads != nullptr) {
    const std::optional<int> parsed = ParsePositiveInt(drake_num_threads);
    if (!parsed) {
      drake::log()->error(
          "Failed to parse environment variable {}={}, falling back to "
          "initializing max threads from hardware concurrency {}",
          kEnvDrakeNumThreads, drake_num_threads, hardware_concurrency);
      return hardware_concurrency;
    } else if (*parsed > hardware_concurrency) {
      drake::log()->warn(
          "Environment variable {}={} is out of range [1, {}], falling back "
          "to initializing max threads from hardware concurrency {}",
          kEnvDrakeNumThreads, drake_num_threads, hardware_concurrency,
          hardware_concurrency);
      return hardware_concurrency;
    } else {
      drake::log()->debug(
          "Initializing max threads to {} from environment variable {}",
          *parsed, kEnvDrakeNumThreads);
      return *parsed;
    }
  } else if (omp_num_threads != nullptr) {
    const std::optional<int> parsed = ParsePositiveInt(omp_num_threads);
    if (!parsed || parsed > hardware_concurrency) {
      drake::log()->debug(
          "Cannot use environment variable {}={}, falling back to "
          "initializing max threads from hardware concurrency {}",
          kEnvOmpNumThreads, omp_num_threads, hardware_concurrency);
      return hardware_concurrency;
    } else {
      drake::log()->debug(
          "Initializing max threads to {} from environment variable {}",
          *parsed, kEnvOmpNumThreads);
      return *parsed;
    }
  } else {
    drake::log()->debug(
        "Environment variables {} and {} not found, initializing max threads "
        "from hardware concurrency {}",
        kEnvDrakeNumThreads, kEnvOmpNumThreads, hardware_concurrency);
    return hardware_concurrency;
  }
}

}  // namespace internal

Parallelism Parallelism::Max() {
  static const int latched_max = internal::ConfigureMaxNumThreads(
      std::getenv(internal::kEnvDrakeNumThreads),
      std::getenv(internal::kEnvOmpNumThreads));
  return Parallelism(latched_max);
}

}  // namespace drake
